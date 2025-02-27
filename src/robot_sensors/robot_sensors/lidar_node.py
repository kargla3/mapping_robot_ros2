import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
import tf2_ros
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info("Lidar Node started")
        self.declare_parameter('lidar_data', '/lidar_raw')
        self.declare_parameter('laser_scan', '/scan')
        self.declare_parameter('angle_min', math.pi)
        self.declare_parameter('angle_max', -math.pi)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 8.0)
        self.lidar_data = self.get_parameter('lidar_data').value
        self.laser_scan = self.get_parameter('laser_scan').value

        self.subscriber_ = self.create_subscription(String, self.lidar_data, self.lidar_callback, 100)
        self.publisher_ = self.create_publisher(LaserScan, self.laser_scan, 100)

        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.get_logger().info(f"Lidar raw data topic: {self.lidar_data}")
        self.get_logger().info(f"Laser scan topic: {self.laser_scan}")
        self.get_logger().info(f"Angle_min {self.angle_min}")
        self.get_logger().info(f"Angle_max {self.angle_max}")
        self.get_logger().info(f"Range_min {self.range_min}")
        self.get_logger().info(f"Range_max {self.range_max}")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def lidar_callback(self, msg):
        scan = LaserScan()
        ranges = msg.data.split(' ')  # Rozdzielanie po spacji
        scan.ranges = [float(x) / 100.0 for x in ranges]  # Przekształcenie do wartości w metrach
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = 2 * math.pi / len(scan.ranges)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges.reverse()

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'  # Ramka bazowa
        transform.child_frame_id = 'laser_frame'  # Ramka liDAR
        transform.transform.translation.x = 0.1  # Przykładowe przesunięcie
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 1.0
        transform.transform.rotation.w = 0.0

        # Publikowanie transformacji
        self.tf_broadcaster.sendTransform(transform)

        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()