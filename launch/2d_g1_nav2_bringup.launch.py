#!/usr/bin/env python3
"""2D Nav2 bringup for Unitree G1 with external Lightning SLAM topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_g1_sim = get_package_share_directory('g1_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    enable_exploration = LaunchConfiguration('enable_exploration')
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
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_bt_xml_filename': nav_to_pose_bt_xml,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        },
        convert_types=True,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true',
        }.items(),
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_sim, 'launch', 'g1_urdf2tf.launch.py')
        ),
        condition=IfCondition(publish_robot_state),
    )

    base_alias_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'base_link'],
        output='screen',
    )

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
        condition=IfCondition(enable_scan_bridge),
    )

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
        condition=IfCondition(enable_realsense_scan_bridge),
    )

    map_warmup_spin_node = Node(
        package='g1_nav',
        executable='map_warmup_spin',
        name='map_warmup_spin',
        output='screen',
        parameters=[{
            'map_topic': '/lightning/grid_map',
            'live_map_ready_topic': '/map_live_ready',
            'cmd_vel_nav_topic': '/cmd_vel_nav',
            'resume_topic': 'explore/resume',
            'spin_angular_speed': map_warmup_angular_speed,
            'min_live_map_area_m2': map_warmup_min_live_area_m2,
        }],
        condition=IfCondition(enable_map_warmup_spin),
    )

    frontier_explorer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='g1_nav',
                executable='frontier_explorer_cpp',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_exploration),
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
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
            'use_rviz',
            default_value='false',
            description='Launch RViz. Requires a valid graphical display.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Launch robot_state_publisher for the RViz RobotModel display.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(
                pkg_g1_nav, 'behavior_trees',
                'navigate_to_pose_w_g1_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateToPose.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=os.path.join(
                pkg_g1_nav, 'behavior_trees',
                'navigate_through_poses_w_g1_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateThroughPoses.',
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
            description='Minimum live /map area before exploration starts.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        robot_state_launch,
        base_alias_tf_node,
        pointcloud_to_scan_node,
        realsense_depth_to_scan_node,
        nav2_launch,
        map_warmup_spin_node,
        frontier_explorer,
        rviz_node,
        g1_move_node,
    ])
