import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_udp = os.path.join(
        get_package_share_directory('robot_communication'), 'config', 'udp_params.yaml'
    )
    config_lidar = os.path.join(
        get_package_share_directory('robot_sensors'), 'config', 'lidar_params.yaml'
    )
    config_keyboard = os.path.join(
        get_package_share_directory('robot_teleop'), 'config', 'keyboard_params.yaml'
    )
    config_joystick = os.path.join(
        get_package_share_directory('robot_teleop'), 'config', 'joystick_params.yaml'
    )
    config_odometry = os.path.join(
        get_package_share_directory('robot_navigation'), 'config', 'odometry_params.yaml'
    )
    config_slam = os.path.join(
        get_package_share_directory('robot_navigation'), 'config', 'slam_params.yaml'
    )
    config_rviz = os.path.join(
        get_package_share_directory('robot_navigation'), 'config', 'rviz_config.rviz'
    )
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config_slam],
            output='screen',
        ),
        Node(
            package='robot_communication',
            executable='udp_node',
            name='UDP',
            parameters=[config_udp],
        ),
        Node(
            package='robot_sensors',
            executable='lidar_node',
            name='lidar',
            parameters=[config_lidar],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='visualization',
            output='screen',
            arguments=['-d', config_rviz]
        ),
        Node(
            package='robot_teleop',
            executable='keyboard_node',
            name='keyboard',
            parameters=[config_keyboard],
        ),
        Node(
            package='robot_navigation',
            executable='odometry_node',
            name='odometry',
            parameters=[config_odometry],
        )
    ])