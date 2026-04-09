#!/usr/bin/env python3
"""Standalone Realsense depth-to-LaserScan bridge for Nav2 local obstacle use."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    realsense_depth_image_topic = LaunchConfiguration('realsense_depth_image_topic')
    realsense_depth_camera_info_topic = LaunchConfiguration('realsense_depth_camera_info_topic')
    realsense_scan_topic = LaunchConfiguration('realsense_scan_topic')
    realsense_scan_output_frame = LaunchConfiguration('realsense_scan_output_frame')
    realsense_scan_time = LaunchConfiguration('realsense_scan_time')
    realsense_scan_range_min = LaunchConfiguration('realsense_scan_range_min')
    realsense_scan_range_max = LaunchConfiguration('realsense_scan_range_max')
    realsense_scan_height = LaunchConfiguration('realsense_scan_height')

    realsense_depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='realsense_depth_to_scan',
        output='screen',
        remappings=[
            ('depth', realsense_depth_image_topic),
            ('depth_camera_info', realsense_depth_camera_info_topic),
            ('scan', realsense_scan_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': realsense_scan_output_frame,
            'scan_time': realsense_scan_time,
            'range_min': realsense_scan_range_min,
            'range_max': realsense_scan_range_max,
            'scan_height': realsense_scan_height,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
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
            default_value='2.5',
            description='Maximum valid range used when converting Realsense depth to LaserScan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_height',
            default_value='3',
            description='Number of depth-image rows fused into the output LaserScan.',
        ),
        realsense_depth_to_scan_node,
    ])
