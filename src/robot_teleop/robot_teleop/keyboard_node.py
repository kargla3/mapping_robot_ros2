import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from time import time

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.get_logger().info("Keyboard Node started")
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('move_topic', '/keyboard_move')
        self.cmd_vel = self.get_parameter('cmd_vel').value
        self.cm = self.get_parameter('move_topic').value

        self.publisher_ = self.create_publisher(String, self.cm, 100)
        self.subscriber_ = self.create_subscription(Twist, self.cmd_vel, self.velocity_callback, 100)
        self.get_logger().info(f"Cmd_vel topic: {self.cmd_vel}")
        self.get_logger().info(f"Move topic: {self.cm}")

        self.last_received_time = time()  # Czas ostatniego odbioru
        self.timer = self.create_timer(0.1, self.timer_callback)  # Co 1 sekundę sprawdzaj
        self.msg = String()

        self.get_logger().info("Use arrow keys to move the robot")
        
    def velocity_callback(self, msg):
        self.get_logger().info(f"Received message: {msg}")
        vel = f"keyboard/{msg.linear.x} {msg.angular.z}"
        msg = String()
        msg.data = vel
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published message: {msg}")
        self.last_received_time = time()  # Zaktualizuj czas ostatniego odbioru

    def timer_callback(self):
        """Jeśli minęła więcej niż 1 sekunda od ostatniego odbioru, wyślij 0.0 0.0"""
        if time() - self.last_received_time > 0.2:  # Sprawdź, czy minęła sekunda
            self.msg.data = "keyboard/0.0 0.0"
            self.publisher_.publish(self.msg)
            self.get_logger().info("Published 0.0 0.0 because no message received")

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    rclpy.spin(keyboard_node)
    keyboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()