import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # params_file = os.path.join(get_package_share_directory('engineer_detector'), 'config', 'detector_params.yaml')

    detector_dir = get_package_share_directory('engineer_detect')
    v4l2_camera_params_file = os.path.join('./src/engineer_detect/config/v4l2_camera.yaml')

    return LaunchDescription([
        # DeclareLaunchArgument(name='params_file', default_value=params_file),
        DeclareLaunchArgument(name='detector_dir', default_value=detector_dir),

        Node(
            package='engineer_detect',
            executable='engineer_cv',
            output='both',
            emulate_tty=True,
            # parameters=[LaunchConfiguration('params_file'), {
            #     'detector_dir': LaunchConfiguration('detector_dir'),
            # }],
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            parameters=[v4l2_camera_params_file],
        )
    ])
