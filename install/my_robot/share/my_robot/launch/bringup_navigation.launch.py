from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Paths
    nav2_launch = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch',
        'bringup_launch.py'
    )

    map_file = '/home/waesco704/maps/final_map.yaml'
    nav2_params = '/home/waesco704/ydlidar_ws/src/my_robot/config/nav2_params.yaml'
    lidar_params = '/home/waesco704/ydlidar_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/X4-Pro.yaml'
    rviz_cfg = '/home/waesco704/ydlidar_ws/src/my_robot/config/navigation_view.rviz'
    slam_loc_params = '/home/waesco704/ydlidar_ws/src/my_robot/config/localization_params.yaml'

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
        # YDLIDAR X4 Pro
        # ==========================
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver',
            output='screen',
            parameters=[lidar_params]
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
            }],
            # menerima perintah dari jalur aman
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
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
                slam_loc_params
            ]
        ),

        # ==========================
        # cmd_vel Safety Bridge
        # ==========================
        Node(
            package='my_robot',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'min_distance_stop': 0.25,
                'slowdown_start': 0.8,
                'scan_topic': '/scan',
                'input_cmd': '/cmd_vel',
                'output_cmd': '/cmd_vel_safe'
            }]
        ),

        # ==========================
        # Nav2 Bringup
        # ==========================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': nav2_params
            }.items()
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
        # RViz (Navigation View)
        # ==========================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])

