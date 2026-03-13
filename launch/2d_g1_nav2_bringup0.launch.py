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
            '/launch_ROS2/rviz_MID360_launch.py'
        ])
    )

    # livox_converter_node = Node(
    #     package='livox_to_pointcloud2',
    #     executable='livox_to_pointcloud2_node',
    #     name='livox_converter',
    #     output='screen'
    # )

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

    

    # ==================== Lightning SLAM 适配节点 ====================
    # 将 3D 点云转换为 2D LaserScan（Lightning SLAM 不提供 /scan）
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/utlidar/cloud_livox_mid360'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base',
            'transform_tolerance': 0.1,
            'min_height': -0.3,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
        }],
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

        # Lightning SLAM 适配
        pointcloud_to_scan_node,
        Node(
            package='g1_nav',
            executable='tf_to_odom',
            name='tf_to_odom',
            output='screen',
        ),

        # 导航核心
        nav2_launch,
        #rviz_node,
        g1_move_node
    ])








