from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Path to your existing launch files
    # (Assuming you put them in a package called 'scan_bringup')
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('scan_routine'), 'launch', 'sensor_bringup.launch.py')
        ])
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_livo'), 'launch', 'mapping_robosense.launch.py')
        ]),
        launch_arguments={'use_rviz': 'True'}.items()
    )

    return LaunchDescription([
        sensor_launch,
        slam_launch
    ])