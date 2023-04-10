#!/usr/bin/env python3
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Your launch configuration here
    declared_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
    
    slam_argument = DeclareLaunchArgument(name='slam_params_file', default_value=PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/slam.yaml']),
                                            description='Slam yaml config file')
    
    run_slam_arg = DeclareLaunchArgument(name='slam', default_value='False',
                                            description='Whether run SLAM')
    
    track_argument = DeclareLaunchArgument(name='track', default_value='kartland',
                                         description='Track name')
    
    nav_params_file = PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config', PythonExpression(["'", LaunchConfiguration('track'), "_nav2_params.yaml'"])])

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch/online_async_launch.py'])
        ),
        launch_arguments={'slam_params_file': LaunchConfiguration('slam_params_file')}.items()
    )

    # nav_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch/navigation_launch.py'])
    #     ),
    #     launch_arguments={'params_file': LaunchConfiguration('nav_params_file'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch/bringup_launch.py'])
        ),
        launch_arguments={'params_file': nav_params_file, 'use_sim_time': LaunchConfiguration('use_sim_time'), 'autostart': 'True',
                            'map': PathJoinSubstitution([FindPackageShare('kart_navigation'),
                                'maps', PythonExpression(["'", LaunchConfiguration('track'), "_map.yaml'"])]),
                            'slam_params_file': LaunchConfiguration('slam_params_file'), 'slam': LaunchConfiguration('slam')}.items()
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[PathJoinSubstitution([FindPackageShare('kart_navigation'), 'config/twist_mux.yaml'])],
        remappings=[('cmd_vel_out', 'racer_01/cmd_vel')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='kart_rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz/nav2_default_view.rviz'])]
    )

    return launch.LaunchDescription([
        declared_argument,
        slam_argument,
        run_slam_arg,
        track_argument,
        # rviz_node,
        # robot_localization_node,
        # slam_launch,
        nav_launch,
        twist_mux_node
    ])

