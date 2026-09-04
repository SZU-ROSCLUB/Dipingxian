from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'cmd_vel': 'cmd_vel',}
    ]

    return LaunchDescription([
        launch_ros.actions.Node(
            package='car_base',
            executable='car_base_node',
            parameters=robot_parameters,
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),
    ])
