import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_driving')
    
    # Nodes
    scan_republisher = Node(
        package='autonomous_driving',
        executable='scan_republisher',
        name='scan_republisher',
        output='screen',
    )
    
    odom_publisher = Node(
        package='autonomous_driving',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )
    
    grid_mapper = Node(
        package='autonomous_driving',
        executable='grid_mapper',
        name='grid_mapper',
        output='screen',
    )
    
    mcl_localizer = Node(
        package='autonomous_driving',
        executable='mcl_localizer',
        name='mcl_localizer',
        output='screen',
    )
    
    lidar_detector = Node(
        package='autonomous_driving',
        executable='lidar_cone_detector',
        name='lidar_cone_detector',
        output='screen',
    )
    
    slam_node = Node(
        package='autonomous_driving',
        executable='slam_node',
        name='slam_node',
        output='screen',
    )
    
    return LaunchDescription([
        scan_republisher,
        odom_publisher,
        grid_mapper,
        mcl_localizer,
        lidar_detector,
        slam_node,
    ])

