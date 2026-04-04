from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # ==========================
        # Simulation Time (optional)
        # ==========================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # ==========================
        # YDLIDAR X4 Pro Driver
        # ==========================
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver',
            output='screen',
            parameters=[
                '/home/waesco704/ydlidar_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/X4-Pro.yaml'
            ]
        ),

        # ==========================
        # ESP32 Serial Bridge (ODOM)
        # ==========================
        Node(
            package='my_robot',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200
            }]
        ),

        # ==========================
        # SLAM Toolbox (LOCALIZATION)
        # ==========================
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                '/home/waesco704/ydlidar_ws/src/my_robot/config/localization_params.yaml'
            ]
        ),

        # ==========================
        # Static TF: base_link → laser_frame
        # ==========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=[
                '0.25', '0.0', '0.65',
                '0', '0', '0',
                'base_link', 'laser_frame'
            ]
        ),

        # ==========================
        # RViz (Localization View)
        # ==========================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                '/home/waesco704/ydlidar_ws/src/my_robot/config/navigation_view.rviz'
            ]
        ),
    ])

