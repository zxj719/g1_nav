#!/usr/bin/env python3
"""
g1_nav2_bringup.launch.py - Unitree G1 导航系统启动文件
"""

import os
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """生成导航系统启动描述"""
    # ==================== 基础路径配置 ====================
    pkg_g1_nav = get_package_share_directory('g1_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_g1_slam = get_package_share_directory('g1_slam_algorithm')
    
    # ==================== 参数声明 ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'))
    
    # ==================== SLAM启动配置 ====================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_slam, 'launch', 'g1_slam_toolbox.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ==================== Nav2启动配置 ====================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # ==================== 可视化与控制节点 ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    g1_move_node = Node(
        package='g1_cmd',
        executable='g1_move',
        name='g1_move',
        output='screen'
    )

    # ==================== 传感器与处理节点 ====================
    launch_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/msg_MID360_launch.py'
        ])
    )

    livox_converter_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_converter',
        output='screen'
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('pcl_localization_ros2'),
            '/launch/pcl_localization.launch.py'
        ])
    )

    
    pcd_to_pointcloud2_node = Node(
        package='pcl_localization_ros2',
        executable='pcd_to_pointcloud2',
        name='pcd_to_pointcloud2',
        output='screen'
    )


    launch_octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('octomap_server2'),
            '/launch/octomap_server_launch.py'
        ])
    )

    launch_cloud2scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g1_lidar_processing'),
            '/launch/cloud_to_scan.launch.py'
        ])
    )

    launch_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g1_sim'),
            '/launch/g1_urdf2tf.launch.py'
        ])
    )

    # ==================== 构建启动描述 ====================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Nav2 parameters file'
        ),
        
        # 硬件接口
        # launch_urdf,
        launch_livox,
        livox_converter_node,
        
        # 感知处理
        # launch_localization,
        # pcd_to_pointcloud2_node,
        # launch_octomap,
        # launch_cloud2scan,
        
        # 导航核心
        nav2_launch,
        rviz_node,
        # g1_move_node
    ])