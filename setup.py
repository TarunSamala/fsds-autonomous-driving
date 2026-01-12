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
            'keyboard_controller = autonomous_driving.keyboard_control:main',
            'scan_republisher = autonomous_driving.scan_republisher:main',
            'fsds_state_estimator = autonomous_driving.fsds_state_estimator:main',
            'fsds_cone_detector = autonomous_driving.fsds_cone_detector:main',
            'local_track_model = autonomous_driving.local_track_model:main',
            'fsds_local_track_model = autonomous_driving.fsds_local_track_model:main'
        ],
    },
)

