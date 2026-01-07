from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='autonomous_driving',
    maintainer_email='autonomous@fsds.com',
    description='FSDS Autonomous Racing Stack - LiDAR + SLAM + Pure Pursuit',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_cone_detector = autonomous_driving.lidar_listener:main',
            'slam_node = autonomous_driving.slam_node:main',
            'keyboard_controller = autonomous_driving.keyboard_control:main',
            'odom_publisher = autonomous_driving.odom_publisher:main',
            'scan_republisher = autonomous_driving.scan_republisher:main',
            'waypoint_recorder_perfect = autonomous_driving.waypoint_recorder_perfect:main',
            'waypoint_follower = autonomous_driving.waypoint_follower:main',
            'slam = autonomous_driving.slam:main',
            'slam_map_publisher = autonomous_driving.slam_map_publisher:main',
            'cone_follower = autonomous_driving.cone_follower:main'
        ],
    },
)

