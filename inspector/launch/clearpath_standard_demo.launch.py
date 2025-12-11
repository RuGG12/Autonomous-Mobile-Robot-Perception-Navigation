#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():
    setup_path = LaunchConfiguration('setup_path', default=os.path.join(os.path.expanduser('~'), 'clearpath/'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=os.path.join(os.path.expanduser('~'), 'clearpath/'),
        description='Path to the setup folder'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('inspector'),
            'rviz',
            'husky.rviz'
        ])],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Use the LOCAL slam launch file we just created
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('inspector'),
                'launch',
                'local_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan'
        }.items()
    )

    nav2_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('inspector'),
                    'launch',
                    'custom_nav2_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('inspector'),
                    'config',
                    'nav2_params.yaml'
                ]),
            }.items()
        )
    ])

    velocity_smoother_node = Node(
        package='inspector',
        executable='inspector',
        name='AZ_velocity_smoother',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    rviz_timer = TimerAction(
        period=5.0,
        actions=[LogInfo(msg='Launching RViz...'), rviz_launch]
    )

    slam_timer = TimerAction(
        period=10.0,
        actions=[LogInfo(msg='Launching SLAM...'), slam_launch]
    )

    velocity_smoother_timer = TimerAction(
        period=15.0,  
        actions=[LogInfo(msg='Launching AZ Velocity Smoother...'), velocity_smoother_node]
    )

    nav2_timer = TimerAction(
        period=20.0,
        actions=[LogInfo(msg='Launching Nav2...'), nav2_launch]
    )

    return LaunchDescription([
        declare_setup_path_arg,
        declare_use_sim_time_arg,
        rviz_timer,
        slam_timer,
        velocity_smoother_timer,
        nav2_timer,
    ])