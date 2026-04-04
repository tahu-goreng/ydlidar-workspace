from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'baudrate': 230400,
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 9,
                'auto_reconnect': True,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': False,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 10.0,
                'range_min': 0.1,
                'frequency': 8.0,
                'debug': False
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

