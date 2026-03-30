#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    explore_nav_to_pose_bt_xml = os.path.join(
        pkg_g1_nav, 'behavior_trees', 'navigate_to_pose_w_g1_explore_fail_fast.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    enable_realsense_scan_bridge = LaunchConfiguration('enable_realsense_scan_bridge')
    realsense_depth_image_topic = LaunchConfiguration('realsense_depth_image_topic')
    realsense_depth_camera_info_topic = LaunchConfiguration('realsense_depth_camera_info_topic')
    realsense_scan_topic = LaunchConfiguration('realsense_scan_topic')
    realsense_scan_output_frame = LaunchConfiguration('realsense_scan_output_frame')
    realsense_scan_time = LaunchConfiguration('realsense_scan_time')
    realsense_scan_range_min = LaunchConfiguration('realsense_scan_range_min')
    realsense_scan_range_max = LaunchConfiguration('realsense_scan_range_max')
    realsense_scan_height = LaunchConfiguration('realsense_scan_height')
    enable_map_warmup_spin = LaunchConfiguration('enable_map_warmup_spin')
    map_warmup_min_live_area_m2 = LaunchConfiguration('map_warmup_min_live_area_m2')
    map_warmup_angular_speed = LaunchConfiguration('map_warmup_angular_speed')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_nav, 'launch', '2d_g1_nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'publish_robot_state': publish_robot_state,
            'params_file': params_file,
            'nav_to_pose_bt_xml': explore_nav_to_pose_bt_xml,
            'enable_exploration': 'true',
            'explorer_params_file': explorer_params_file,
            'enable_scan_bridge': enable_scan_bridge,
            'enable_realsense_scan_bridge': enable_realsense_scan_bridge,
            'realsense_depth_image_topic': realsense_depth_image_topic,
            'realsense_depth_camera_info_topic': realsense_depth_camera_info_topic,
            'realsense_scan_topic': realsense_scan_topic,
            'realsense_scan_output_frame': realsense_scan_output_frame,
            'realsense_scan_time': realsense_scan_time,
            'realsense_scan_range_min': realsense_scan_range_min,
            'realsense_scan_range_max': realsense_scan_range_max,
            'realsense_scan_height': realsense_scan_height,
            'enable_map_warmup_spin': enable_map_warmup_spin,
            'map_warmup_min_live_area_m2': map_warmup_min_live_area_m2,
            'map_warmup_angular_speed': map_warmup_angular_speed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz together with Nav2 and frontier exploration.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Publish robot_description so RViz can show the robot model.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_g1_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        DeclareLaunchArgument(
            'enable_realsense_scan_bridge',
            default_value='false',
            description='Launch depthimage_to_laserscan to publish /scan from Realsense depth.',
        ),
        DeclareLaunchArgument(
            'realsense_depth_image_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='Depth image topic consumed by depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_depth_camera_info_topic',
            default_value='/camera/camera/depth/camera_info',
            description='Depth camera info topic consumed by depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_topic',
            default_value='/scan',
            description='LaserScan topic published by the Realsense depth bridge.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_output_frame',
            default_value='camera_depth_frame',
            description='Frame id assigned to the LaserScan generated from Realsense depth.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_time',
            default_value='0.2',
            description='scan_time parameter passed to depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_range_min',
            default_value='0.2',
            description='Minimum valid range used when converting Realsense depth to LaserScan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_range_max',
            default_value='3.0',
            description='Maximum valid range used when converting Realsense depth to LaserScan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_height',
            default_value='3',
            description='Number of depth-image rows fused into the output LaserScan.',
        ),
        DeclareLaunchArgument(
            'enable_map_warmup_spin',
            default_value='false',
            description='Rotate in place until the first live SLAM map grows to a usable size.',
        ),
        DeclareLaunchArgument(
            'map_warmup_min_live_area_m2',
            default_value='12.0',
            description='Minimum live /map area before exploration resumes.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        bringup_launch,
    ])
