

#!/usr/bin/env python3
"""
g1_nav2_bringup.launch.py - Unitree G1 导航系统启动文件
""" 

import os
import launch
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

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
    map_yaml_path = LaunchConfiguration(
    'map', default=os.path.join(pkg_g1_nav, 'maps', '14.yaml'))

    
    # 获取launch文件和定义地址
    bringup_dir = get_package_share_directory("g1_slam_algorithm")

    # 获取rviz配置文件路径
    rviz_file = os.path.join(
        bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )  ###nav2_default_view   g1_slam


    # ==================== Nav2启动配置 ====================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')###navigation_launch
        ),
        launch_arguments={
            'map': map_yaml_path,
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
        arguments=['-d', os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz')], #nav2_go2_view.rviz缺失
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz2节点
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
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

    launch_livox_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/3d_cloud_MID360_launch.py'
        ])
    )

    livox_converter_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_converter',
        output='screen'
    )

    #通过 domain_bridge 工具在不同 ROS 2 域（Domain）之间桥接话题 / 服务
    #将 ROS 2 域 0 中发布的 odommodestate 话题（类型为 unitree_go/msg/SportModeState），
    #通过可靠传输方式（确保不丢包）桥接到域 1，并设置消息缓存队列长度为 10，将 Unitree 机器人的运动状态从一个域共享到另一个域的场景
    domain_bridge_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'domain_bridge', 'domain_bridge',
            PathJoinSubstitution([
                FindPackageShare('domain_bridge'),
                'examples',
                'example_bridge_config.yaml'
            ])
        ],
        output='screen'
    ) 

    # 使用自定义odom进行tf关系发布 
    start_cus_tftree_node = Node(
        #parameters=[{'use_sim_time': use_sim_time}],
        package='g1_slam_algorithm',
        executable='d3_motion_to_tf',
        name='d3_motion_to_tf',
        output='screen',
    )

    # 激光点云转LaserScan（确保输出话题与lidar_loc的输入匹配）
    launch_cloud2scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g1_lidar_processing'),
            '/launch/3d_cloud_to_scan.launch.py'
        ])
    )

    # URDF及对应的TF变换 发布
    g1_sim_dir = get_package_share_directory('g1_sim')
    start_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(g1_sim_dir, 'launch', 'g1_urdf2tf.launch.py')
        ])
    )
    
    pcd_to_pointcloud2_node = Node(
        package='pcl_localization_ros2',
        executable='pcd_to_pointcloud2',
        name='pcd_to_pointcloud2',
        output='screen'
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('pcl_localization_ros2'),
            '/launch/pcl_localization.launch.py'
        ])
    )

    launch_octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('octomap_server2'),
            '/launch/octomap_server_launch.py'
        ])
    )


    # ==================== lidar_loc定位节点 ====================
    lidar_loc_node = Node(
        package='lidar_loc',
        executable='lidar_loc',     
        name='lidar_loc',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},  
            {'laser_topic': '/scan'},       
            {'base_frame': 'base'}, 
            {'laser_frame': 'mid360_link'},        
            {'odom_frame': 'odom'}           
        ],
        remappings=[
            ('map', '/map')  # 订阅全局地图（与地图服务器发布的话题一致）
        ]
    )

    # ==================== 构建启动描述 ====================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='use_sim_time',
            description='Use simulation clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Nav2 parameters file'
        ),
 
        # 硬件接口
        launch_livox,
        #launch_livox_pointcloud,
        livox_converter_node,
        launch_cloud2scan,
        domain_bridge_node,
        start_urdf_launch,
        #start_cus_tftree_node,


        #start_rviz_node,
        # 感知处理
        # launch_localization,
        # pcd_to_pointcloud2_node,
        # launch_octomap,
        # launch_cloud2scan,
        #lidar_loc_node,       # 启动激光定位
        
        # 导航核心
        ## rviz_node,

        #nav2_launch,
        #start_rviz_node,
        g1_move_node
     ])