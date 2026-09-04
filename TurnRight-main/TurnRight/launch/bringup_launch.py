import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    car_base_dir = get_package_share_directory('car_base')
    lidar_dir = get_package_share_directory('lslidar_driver')

    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_base_dir, 'launch', 'base_serial.launch.py')
        )
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lsn10_launch.py')
        )
    ))
    ld.add_action(Node(
        package='mylio',           
        executable='mylio_node',   
        name='mylio_node',        
        output='screen',
    ))

    return ld