import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- File Paths ---
    # RSLidar paths
    rviz_config = os.path.join(get_package_share_directory('rslidar_sdk'), 'rviz', 'rviz2.rviz')
    rslidar_config = '/home/scanar/scan_ar/src/rslidar_sdk/config/config.yaml'
    
    # IMU paths
    imu_config = '/home/scanar/scan_ar/src/inertial-sense-sdk/ROS/ros2/launch/imu_config.yaml'

    # --- Node Definitions ---
    
    # 1. RSLidar Node
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        namespace='rslidar_sdk',
        output='screen',
        parameters=[{'config_path': rslidar_config}]
    )

    # 2. InertialSense IMU Node
    imu_node = Node(
        package='inertial_sense_ros2',
        executable='inertial_sense_ros2_node',
        name='inertial_sense_ros2',
        output='screen',
        parameters=[imu_config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 3. USB Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'time_per_frame': [1, 30]
        }]
    )

    # 4. Rviz2 Node (Optional: You can comment this out if you want to run headless)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        rslidar_node,
        imu_node,
        camera_node,
        # rviz_node
    ])