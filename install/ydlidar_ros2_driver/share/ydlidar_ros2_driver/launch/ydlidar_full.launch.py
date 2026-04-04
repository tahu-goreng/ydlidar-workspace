from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    rviz_config = os.path.expanduser(
        '~/ydlidar_ws/src/ydlidar_ros2_driver/rviz/ydlidar_auto_view.rviz'
    )
    params_file = os.path.expanduser(
        '~/ydlidar_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml'
    )

    return LaunchDescription([
        # Jalankan driver LiDAR
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[params_file]
        ),

        # Static transform (laser_frame → base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
        ),

        # RViz auto-view
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])

