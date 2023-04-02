#!/usr/bin/env python3
import os
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Your launch configuration here
    declared_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
    
    kart_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kart_simulation'), 'launch/kart_kartland_simulation.launch.py'])
        )
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
        kart_sim_launch,
        robot_localization_node
    ])

