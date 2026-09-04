import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Copy config files
    # dnn_node_example_path = os.path.join(get_package_prefix('dnn_node_example'), "lib/dnn_node_example")
    # os.system(f"cp -r /userdata/lrj_test_mppi/config .")
    # test
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument("dnn_example_config_file", default_value=TextSubstitution(text="config/yolov5xworkconfig.json")),
        DeclareLaunchArgument("dnn_example_dump_render_img", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("dnn_example_image_width", default_value=TextSubstitution(text="480")),
        DeclareLaunchArgument("dnn_example_image_height", default_value=TextSubstitution(text="272")),
        DeclareLaunchArgument("dnn_example_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection")),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '1920', 'usb_image_height': '1080',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    nv12_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_img'}.items())

    jpeg_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/image'}.items())
    # web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
    #                                     launch_arguments={'websocket_image_topic': '/image', 'websocket_image_type': 'mjpeg',
    #                                                       'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")}.items())
    
    # depth_camera_node = IncludeLaunchDescription(PythonLaunchDescriptionSource("/userdata/lrj_test_mppi/launch/aurora930_launch.py"))
    
    # depth_camera_node = ExecuteProcess(
    #     cmd=['ros2', 'launch', '/userdata/lrj_test_mppi/launch/aurora930_launch.py'],
    #     output='log',  # �������־�ļ��������ն���ʾ
    #     # output='none',  # ��ȫ��ֹ�����ROS 2 Foxy+ ֧�֣�
    # )
    
    # Algorithm node
    dnn_node_example_node = Node(
        package='dnn_node_example',
        executable='example',
        output='screen',
        parameters=[
            {"config_file": LaunchConfiguration('dnn_example_config_file')},
            {"dump_render_img": LaunchConfiguration('dnn_example_dump_render_img')},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": LaunchConfiguration("dnn_example_msg_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    qrcode_node = Node(
        package='qrcode',
        executable='qrcode_node',
        output='screen',
    )
    
    return LaunchDescription(launch_args + [
        # depth_camera_node,
        qrcode_node,
        usb_node,
        nv12_codec_node,
        dnn_node_example_node,
        image_transport_node,
        # web_node,
    ])
