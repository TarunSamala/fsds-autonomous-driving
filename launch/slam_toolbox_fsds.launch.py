import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_driving')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_fsds.yaml')

    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params,
        description='Full path to slam_toolbox params YAML file.'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    return LaunchDescription([
        declare_slam_params,
        slam_node,
    ])

