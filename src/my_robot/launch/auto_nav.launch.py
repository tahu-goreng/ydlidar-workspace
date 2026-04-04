from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        )
    ])
