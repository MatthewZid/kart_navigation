#!/usr/bin/env python3
import os
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Your launch configuration here
    declared_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
    
    slam_argument = DeclareLaunchArgument(name='slam_params_file', default_value=PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/slam.yaml']),
                                            description='Slam yaml config file')
    
    nav_argument = DeclareLaunchArgument(name='nav_params_file', default_value=PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/nav2_params.yaml']),
                                            description='Nav2 yaml config file')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch/online_async_launch.py'])
        ),
        launch_arguments={'slam_params_file': LaunchConfiguration('slam_params_file')}.items()
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch/navigation_launch.py'])
        ),
        launch_arguments={'params_file': LaunchConfiguration('nav_params_file'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        declared_argument,
        slam_argument,
        nav_argument,
        robot_localization_node,
        slam_launch,
        nav_launch
    ])

