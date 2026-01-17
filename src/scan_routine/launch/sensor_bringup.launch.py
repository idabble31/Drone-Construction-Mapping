import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- File Paths ---
    # Descriptions
    pkg_description = get_package_share_directory('scan_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'scanar_rig.xacro')
    
    # Process Xacro to XML string
    # Note: Xacro handles the YAML loading internally because of your xacro:load_yaml tag
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Hardware Configs (Absolute paths are okay, but package paths are better)
    rslidar_config = '/home/scanar/scan_ar/src/rslidar_sdk/config/config.yaml'
    imu_config = '/home/scanar/scan_ar/src/inertial-sense-sdk/ROS/ros2/launch/imu_config.yaml'

    rviz_config = '/home/scanar/scan_ar/src/scan_routine/rviz/default.rviz'

    # --- Node Definitions ---

    # 1. Robot State Publisher (Static TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # 2. RSLidar
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        namespace='rslidar_sdk',
        output='screen',
        parameters=[{'config_path': rslidar_config}]
    )

    # 3. InertialSense IMU
    imu_node = Node(
        package='inertial_sense_ros2',
        executable='inertial_sense_ros2_node',
        name='inertial_sense_ros2',
        output='screen',
        parameters=[imu_config]
    )

    # 4. USB Camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'time_per_frame': [1,30],
            'camera_frame_id': 'camera_link',
            'camera_name': 'default_cam'
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        rslidar_node,
        imu_node,
        camera_node,
        rviz_node
    ])