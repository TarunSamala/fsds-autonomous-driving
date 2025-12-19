import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('autonomous_driving'),
        'config'
    )
    
    hector_mapping_config = os.path.join(config_dir, 'hector_mapping.yaml')
    
    return LaunchDescription([
        # Hector Mapping Node (SLAM)
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[hector_mapping_config],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        ),
    ])

