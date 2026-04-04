import os
from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), [
            'launch/bringup_slam.launch.py',
            'launch/bringup_localization.launch.py',
            'launch/bringup_navigation.launch.py',
        ]),

        # Config files (IMPORTANT: include ALL yaml + rviz)
        (os.path.join('share', package_name, 'config'), [
            'config/slam_params_online_async.yaml',
            'config/localization_params.yaml',
            'config/nav2_params.yaml',
            'config/ydlidar_slam_view.rviz',
            'config/navigation_view.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waesco704',
    maintainer_email='waesco704@nchu.edu.tw',
    description='Autonomous mobile robot control using YDLIDAR and ESP32-S3 bridge (ROS 2 Humble)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'serial_bridge = my_robot.serial_bridge:main',
            'cmd_vel_bridge = my_robot.cmd_vel_bridge:main',
            'reactive_avoidance = my_robot.reactive_avoidance:main',
        ],
    },
)

