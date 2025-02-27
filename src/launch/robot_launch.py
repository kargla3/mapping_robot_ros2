from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_communication',
            namespace='communication',
            executable='udp_node',
            name='sim'
        ),
        Node(
            package='robot_sensors',
            namespace='sensor',
            executable='lidar_node',
            name='sim'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='visualization',
            output='screen',
        ),
        Node(
            package='robot_teleop',
            namespace='control',
            executable='keyboard_node',
            name='sim'
        ),
    ])