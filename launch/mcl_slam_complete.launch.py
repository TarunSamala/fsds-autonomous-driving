import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_driving')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_fsds.yaml')
    
    nodes = [
        # Core perception pipeline
        Node(
            package='autonomous_driving',
            executable='scan_republisher',
            name='scan_republisher',
            output='screen',
        ),
        Node(
            package='autonomous_driving',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
        ),
        
        # Cone detection + SLAM
        Node(
            package='autonomous_driving',
            executable='lidar_cone_detector',
            name='lidar_cone_detector',
            output='screen',
        ),
        Node(
            package='autonomous_driving',
            executable='slam_node',
            name='slam_node',
            output='screen',
        ),
        
        # Localization
        Node(
            package='autonomous_driving',
            executable='mcl_localizer_v2',
            name='mcl_localizer_v2',
            output='screen',
        ),
        
        # Map persistence
        Node(
            package='autonomous_driving',
            executable='map_manager',
            name='map_manager',
            output='screen',
        ),
        
        # Control
        Node(
            package='autonomous_driving',
            executable='keyboard_controller',
            name='keyboard_controller',
            output='screen',
        ),
        
        # Pure pursuit steering
        Node(
            package='autonomous_driving',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
        ),
        
        # Waypoint recording for learning
        Node(
            package='autonomous_driving',
            executable='waypoint_recorder',
            name='waypoint_recorder',
            output='screen',
        ),
        
        # Autonomous waypoint following
        Node(
            package='autonomous_driving',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
        ),
    ]
    
    return LaunchDescription(nodes)

