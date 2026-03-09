from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义地图YAML文件的绝对路径
    map_yaml_path = '/home/unitree/unitree_ros2/unitree_g1_ros2_function_pkg/src/g1_nav/maps/map.yaml'

    # 创建地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path}  # 指定地图文件路径
        ]
    )

    # 创建生命周期管理节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'node_names': ['map_server']},  # 需要管理的节点列表
            {'autostart': True}              # 自动启动
        ]
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager,
    ])