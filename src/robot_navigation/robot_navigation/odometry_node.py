import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("Odometry Node started")
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('wheels_velocity_data', '/wheels_vel_raw')
        self.declare_parameter('wheel_base', 0.3)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_L = 0.0
        self.v_R = 0.0

        self.last_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.wheels_velocity_data = self.get_parameter('wheels_velocity_data').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publisher_ = self.create_publisher(Odometry, self.odometry_topic, 100)
        self.subscription = self.create_subscription(String, self.wheels_velocity_data, self.odometry_callback, 100)
        self.get_logger().info(f"Odometry topic: {self.odometry_topic}")
        self.get_logger().info(f"Wheels velocity raw data topic: {self.wheels_velocity_data}")

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for updating odometry at fixed interval (20 Hz)
        self.create_timer(0.2, self.update_odometry)
        self.create_timer(0.2, self.publish_tf)

    def odometry_callback(self, msg):
        """Callback for receiving velocity data"""
        try:
            data = msg.data.split(' ')
            self.v_L = float(data[0])
            self.v_R = float(data[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error parsing velocity data: {e}")
            return

    def update_odometry(self):
        """Update odometry and publish data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        linear = (self.v_L + self.v_R) / 2
        angular = (self.v_R - self.v_L) / self.wheel_base

        delta_x = linear * math.cos(self.theta) * dt
        delta_y = linear * math.sin(self.theta) * dt
        delta_theta = angular * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        """Publish odometry data to the topic"""
        odometry = Odometry()
        odometry.header.stamp = current_time.to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'

        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw

        odometry.twist.twist.linear.x = (self.v_L + self.v_R) / 2
        odometry.twist.twist.angular.z = (self.v_R - self.v_L) / self.wheel_base

        self.publisher_.publish(odometry)

    def publish_tf(self):
        """Publish transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
