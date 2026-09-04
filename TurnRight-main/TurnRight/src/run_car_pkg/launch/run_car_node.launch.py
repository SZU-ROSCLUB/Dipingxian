import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('run_car_pkg'),
        'params',
        'run_car_node.yaml'                         
    )
    run_car_node = Node(
        package='run_car_pkg',       
        executable='run_car_node',
        output='screen',
        name='run_car_node',
        parameters=[param_file_path],
        arguments=['--ros-args', '--log-level', 'info']
    )
    return LaunchDescription([
        run_car_node
    ])
