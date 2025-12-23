from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_imu_listener = autonomous_driving.listener:main',
            'lidar_cone_detector = autonomous_driving.lidar_listener:main',
            'keyboard_controller = autonomous_driving.keyboard_control:main',
            'control_test = autonomous_driving.control_test:main',
            'slam_node = autonomous_driving.slam_node:main',
            'planner_node = autonomous_driving.planner_node:main',
            'waypoint_recorder = autonomous_driving.waypoint_recorder_auto:main',
            'pure_pursuit = autonomous_driving.pure_pursuit_controller:main',
            'localization_node = autonomous_driving.localization_node:main',
            'waypoint_perfect = autonomous_driving.waypoint_recorder_perfect:main',
            'odom_publisher = autonomous_driving.odom_publisher:main',
            'scan_republisher = autonomous_driving.scan_republisher:main',
            'grid_mapper = autonomous_driving.grid_mapper:main',
            'mcl_localizer = autonomous_driving.mcl_localizer:main'
        ],
    },
)

