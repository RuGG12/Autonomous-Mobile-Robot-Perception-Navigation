#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Real Robot = default false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. TIMESTAMP FIXER (REQUIRED for your unsynced Ouster)
    # This aligns the Ouster's "Boot Time" (700s) to the Robot's "Wall Time" (2025s)
    timestamp_fixer_node = Node(
        package='inspector',
        executable='timestamp_fixer',
        name='timestamp_fixer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. CONVERTER
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_scan_real',
        remappings=[
            # LISTEN TO THE FIXED TOPIC from the fixer node
            ('cloud_in', '/ouster/points_fixed'), 
            ('scan', '/scan') 
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'os_sensor', 
            'range_min': 0.8,         
            'range_max': 90.0,
            'min_height': -0.10,
            'max_height': 0.10,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        output='screen'
    )

    # 3. STATIC TF PUBLISHER (REQUIRED)
    # Manually connects the lidar to the robot.
    # *** UPDATE THESE NUMBERS with your physical measurements ***
    static_lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'os_sensor'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        timestamp_fixer_node,
        pointcloud_to_scan_node,
        static_lidar_tf_publisher
    ])