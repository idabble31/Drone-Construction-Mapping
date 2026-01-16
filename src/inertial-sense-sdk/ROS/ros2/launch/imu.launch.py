import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your specific YAML file
    # You can point this to the one in your src folder or the installed one
    config_file = '/home/scanar/scan_ar/src/inertial-sense-sdk/ROS/ros2/launch/imu_config.yaml'

    return LaunchDescription([
        Node(
            package='inertial_sense_ros2',
            executable='inertial_sense_ros2_node',
            name='inertial_sense_ros2',  # This MUST match the top level of your YAML
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])