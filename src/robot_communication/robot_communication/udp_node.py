import rclpy
import socket
import threading
import sys
from rclpy.node import Node
from std_msgs.msg import String

class UdpNode(Node):
    def __init__(self):
        super().__init__('udp_node')
        self.get_logger().info("UDP Node started")

        self.declare_parameter('udp_port_recv', 5005)
        self.declare_parameter('udp_port_send', 5005)
        self.declare_parameter('udp_ip', '192.168.4.1')
        self.declare_parameter('lidar_data', '/lidar_raw')
        self.declare_parameter('wheels_velocity_data', '/wheels_vel_raw')
        self.declare_parameter('control_movement_topic', '/keyboard_move')

        self.udp_port_recv = self.get_parameter('udp_port_recv').value
        self.udp_port_send = self.get_parameter('udp_port_send').value
        self.udp_ip = self.get_parameter('udp_ip').value
        self.lidar_data = self.get_parameter('lidar_data').value
        self.wheels_velocity_data = self.get_parameter('wheels_velocity_data').value
        self.control_movement_topic = self.get_parameter('control_movement_topic').value

        self.get_logger().info(f"UDP IP: {self.udp_ip}")
        self.get_logger().info(f"UDP Port (recv): {self.udp_port_recv}")
        self.get_logger().info(f"UDP Port (send): {self.udp_port_send}")
        self.get_logger().info(f"Lidar raw data topic: {self.lidar_data}")
        self.get_logger().info(f"Wheels velocity raw data topic: {self.wheels_velocity_data}")
        self.get_logger().info(f"Movement controller topic: {self.control_movement_topic}")

        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(('', self.udp_port_recv))  
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.lidar_publisher_ = self.create_publisher(String, self.lidar_data, 100)
        self.vel_publisher_ = self.create_publisher(String, self.wheels_velocity_data, 100)
        self.subscription = self.create_subscription(String, self.control_movement_topic, self.control_callback, 100)

        self.running = True
        self.recv_thread = threading.Thread(target=self.receive_udp, daemon=True)
        self.recv_thread.start()

    def receive_udp(self):
        while self.running:
            try:
                data, addr = self.sock_recv.recvfrom(1024)
                if not data:
                    continue

                msg = String()
                msg.data = data.decode('utf-8')

                if msg.data.startswith("lidar/"):
                    msg.data = msg.data.replace("lidar/", "", 1).strip()
                    self.lidar_publisher_.publish(msg)
                elif msg.data.startswith("wheels/"):
                    msg.data = msg.data.replace("wheels/", "", 1).strip()
                    self.vel_publisher_.publish(msg)
                else:
                    self.get_logger().info(f"Nieznany typ danych: {msg.data}")

                self.get_logger().info(f"Odebrano z {addr}: {msg.data}")
            except OSError:
                break  
            except Exception as e:
                self.get_logger().error(f"Błąd UDP (recv): {e}")

    def control_callback(self, msg):
        try:
            self.sock_send.sendto(msg.data.encode('utf-8'), (self.udp_ip, self.udp_port_send))
            self.get_logger().info(f"Wysłano do {self.udp_ip}:{self.udp_port_send}: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Błąd UDP (send): {e}")

    def shutdown(self):
        """ Zamykanie węzła i zasobów """
        if not self.running:
            return  # Zapobiega wielokrotnemu zamykaniu

        self.get_logger().info("Zamykanie UDP Node...")
        self.running = False
        self.sock_recv.close()
        self.sock_send.close()
        self.recv_thread.join(timeout=1)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UdpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Odebrano Ctrl+C, zamykanie...")
    finally:
        node.shutdown()
        if rclpy.ok():  # Sprawdzamy, czy shutdown nie był już wywołany
            rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
