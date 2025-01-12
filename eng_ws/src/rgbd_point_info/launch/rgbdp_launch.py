from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rgbdp_info',
            executable='rgbdp_info_node',
            name='rgbdp_info_node',
            parameters=[
                {'camera_info_file': '/home/ubuntu/tools/tools_ws/src/rgbd_point_info/config/camera_info.yaml'}
            ]
        ),
    ])
