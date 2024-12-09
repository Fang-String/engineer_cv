from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory
    ci_share_dir = FindPackageShare('color_icp')

    return LaunchDescription([
        # Launch RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', PathJoinSubstitution([gcopter_share_dir, 'config', 'global_planning.rviz'])]
        # ),
        
        # Launch the global_planning node
        Node(
            package='color_icp',
            executable='color_icp_node',
            name='color_icp',
            output='both',
            parameters=[PathJoinSubstitution([ci_share_dir, 'config', 'params.yaml'])]
        ),
    ])
