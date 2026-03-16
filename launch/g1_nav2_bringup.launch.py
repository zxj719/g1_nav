#!/usr/bin/env python3
"""
g1_nav2_bringup.launch.py - Unitree G1 navigation bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    enable_exploration = LaunchConfiguration('enable_exploration')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    explorer_impl = LaunchConfiguration('explorer_impl')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    launch_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/msg_MID360_launch.py',
        ])
    )

    livox_converter_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_converter',
        output='screen',
    )

    launch_octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('octomap_server2'),
            '/launch/octomap_server_launch.py',
        ])
    )

    launch_cloud2scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g1_lidar_processing'),
            '/launch/cloud_to_scan.launch.py',
        ])
    )

    launch_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g1_sim'),
            '/launch/g1_urdf2tf.launch.py',
        ])
    )

    frontier_explorer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='g1_nav',
                executable='frontier_explorer.py',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(
                    PythonExpression([
                        '"', enable_exploration, '" == "true" and "',
                        explorer_impl, '" == "python"'
                    ])
                ),
            ),
            Node(
                package='g1_nav',
                executable='frontier_explorer_cpp',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(
                    PythonExpression([
                        '"', enable_exploration, '" == "true" and "',
                        explorer_impl, '" == "cpp"'
                    ])
                ),
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    g1_move_node = Node(
        package='g1_cmd',
        executable='g1_move',
        name='g1_move',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'enable_exploration',
            default_value='false',
            description='Launch frontier explorer after Nav2 bringup',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_g1_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'explorer_impl',
            default_value='cpp',
            description='Explorer implementation: cpp or python',
        ),
        launch_urdf,
        launch_livox,
        livox_converter_node,
        launch_octomap,
        launch_cloud2scan,
        nav2_launch,
        frontier_explorer,
        rviz_node,
        g1_move_node,
    ])
