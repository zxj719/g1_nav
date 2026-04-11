import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module(relative_path: str, module_name: str):
    launch_path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: f'/tmp/{package}'
    return module


def _text_value(substitution):
    if isinstance(substitution, (list, tuple)):
        return ''.join(_text_value(item) for item in substitution)

    class_name = type(substitution).__name__
    if class_name == 'TextSubstitution':
        return substitution._TextSubstitution__text
    if class_name == 'LaunchConfiguration':
        name_parts = substitution._LaunchConfiguration__variable_name
        return ''.join(_text_value(part) for part in name_parts)
    return str(substitution)


def _node_remappings(node):
    return {
        _text_value(source): _text_value(target)
        for source, target in node._Node__remappings
    }


def _node_parameters(node):
    parameters = {}
    for parameter_set in node._Node__parameters:
        for key_parts, value in parameter_set.items():
            parameters[_text_value(key_parts)] = _text_value(value)
    return parameters


def _include_source_location(include_action):
    source = include_action._IncludeLaunchDescription__launch_description_source
    return _text_value(source._LaunchDescriptionSource__location)


def _declared_launch_arguments(launch_description):
    return {
        entity.name: entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'DeclareLaunchArgument'
    }


def _named_sequence(relative_path: str, sequence_name: str):
    root = ET.parse(REPO_ROOT / relative_path).getroot()
    return next(
        node
        for node in root.iter('Sequence')
        if node.attrib.get('name') == sequence_name
    )


def test_realsense_depth_to_scan_launch_declares_bridge_arguments():
    module = _load_launch_module(
        'launch/realsense_depth_to_scan.launch.py',
        'g1_nav_realsense_depth_to_scan_launch',
    )
    launch_description = module.generate_launch_description()
    arguments = _declared_launch_arguments(launch_description)

    expected_defaults = {
        'use_sim_time': 'false',
        'realsense_depth_image_topic': '/camera/camera/depth/image_rect_raw',
        'realsense_depth_camera_info_topic': '/camera/camera/depth/camera_info',
        'realsense_scan_topic': '/scan',
        'realsense_scan_output_frame': 'camera_depth_frame',
        'realsense_scan_time': '0.2',
        'realsense_scan_range_min': '0.2',
        'realsense_scan_range_max': '2.5',
        'realsense_scan_height': '3',
    }

    assert set(expected_defaults).issubset(arguments)

    for name, expected in expected_defaults.items():
        assert _text_value(arguments[name].default_value) == expected


def test_realsense_depth_to_scan_launch_contains_depthimage_to_laserscan_node():
    module = _load_launch_module(
        'launch/realsense_depth_to_scan.launch.py',
        'g1_nav_realsense_depth_to_scan_launch_node',
    )
    launch_description = module.generate_launch_description()
    matching_nodes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'Node'
        and entity._Node__package == 'depthimage_to_laserscan'
        and entity._Node__node_executable == 'depthimage_to_laserscan_node'
    ]

    assert len(matching_nodes) == 1

    node = matching_nodes[0]
    assert _node_remappings(node) == {
        'depth': 'realsense_depth_image_topic',
        'depth_camera_info': 'realsense_depth_camera_info_topic',
        'scan': 'realsense_scan_topic',
    }
    assert _node_parameters(node) == {
        'use_sim_time': 'use_sim_time',
        'output_frame': 'realsense_scan_output_frame',
        'scan_time': 'realsense_scan_time',
        'range_min': 'realsense_scan_range_min',
        'range_max': 'realsense_scan_range_max',
        'scan_height': 'realsense_scan_height',
    }


def test_bringup_declares_realsense_scan_bridge_argument():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch',
    )
    launch_description = module.generate_launch_description()
    argument_names = set(_declared_launch_arguments(launch_description))

    assert 'enable_realsense_scan_bridge' in argument_names


def test_bringup_does_not_declare_realsense_camera_arguments():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_camera_args',
    )
    launch_description = module.generate_launch_description()
    argument_names = set(_declared_launch_arguments(launch_description))

    removed_arguments = {
        'enable_realsense_camera',
        'realsense_camera_name',
        'realsense_camera_namespace',
        'realsense_camera_serial_no',
        'realsense_camera_config_file',
        'realsense_json_file_path',
        'realsense_camera_log_level',
        'realsense_camera_output',
        'realsense_enable_color',
        'realsense_enable_depth',
        'realsense_pointcloud_enable',
        'realsense_align_depth_enable',
        'realsense_publish_tf',
        'realsense_tf_publish_rate',
    }

    assert removed_arguments.isdisjoint(argument_names)


def test_bringup_does_not_include_realsense_camera_launch():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_for_camera_include',
    )
    launch_description = module.generate_launch_description()
    matching_includes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith('/realsense2_camera/launch/rs_launch.py')
    ]

    assert matching_includes == []


def test_bringup_includes_realsense_depth_to_scan_launch():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_for_depth_to_scan_include',
    )
    launch_description = module.generate_launch_description()
    matching_includes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith('/g1_nav/launch/realsense_depth_to_scan.launch.py')
    ]

    assert len(matching_includes) == 1

    include_action = matching_includes[0]
    launch_arguments = dict(include_action.launch_arguments)
    assert launch_arguments.keys() == {'use_sim_time'}
    assert _text_value(launch_arguments['use_sim_time']) == 'use_sim_time'


def test_bringup_declares_collision_monitor_params_argument():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_collision_monitor_arg',
    )
    launch_description = module.generate_launch_description()
    arguments = _declared_launch_arguments(launch_description)

    assert 'collision_monitor_params_file' in arguments
    assert _text_value(arguments['collision_monitor_params_file'].default_value).endswith(
        '/g1_nav/config/collision_monitor_params.yaml'
    )


def test_bringup_routes_smoothed_cmd_vel_through_collision_monitor():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_collision_monitor_nodes',
    )
    launch_description = module.generate_launch_description()

    collision_monitor_nodes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'Node'
        and entity._Node__package == 'nav2_collision_monitor'
        and entity._Node__node_executable == 'collision_monitor'
    ]
    assert len(collision_monitor_nodes) == 1

    g1_move_nodes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'Node'
        and entity._Node__package == 'g1_cmd'
        and entity._Node__node_executable == 'g1_move'
    ]
    assert len(g1_move_nodes) == 1
    assert _node_remappings(g1_move_nodes[0]) == {
        'cmd_vel': 'cmd_vel_safe',
    }


def test_bringup_does_not_declare_realsense_bridge_parameter_arguments():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_without_bridge_params',
    )
    launch_description = module.generate_launch_description()
    argument_names = set(_declared_launch_arguments(launch_description))

    removed_arguments = {
        'realsense_depth_image_topic',
        'realsense_depth_camera_info_topic',
        'realsense_scan_topic',
        'realsense_scan_output_frame',
        'realsense_scan_time',
        'realsense_scan_range_min',
        'realsense_scan_range_max',
        'realsense_scan_height',
    }

    assert removed_arguments.isdisjoint(argument_names)


def test_auto_explore_passes_realsense_scan_bridge_toggle_through():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch',
    )
    launch_description = module.generate_launch_description()
    include_action = next(
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith('/g1_nav/launch/2d_g1_nav2_bringup.launch.py')
    )
    launch_arguments = dict(include_action.launch_arguments)

    assert 'enable_realsense_scan_bridge' in launch_arguments
    assert _text_value(launch_arguments['enable_realsense_scan_bridge']) == (
        'enable_realsense_scan_bridge'
    )


def test_auto_explore_enables_realsense_scan_bridge_by_default():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_defaults',
    )
    launch_description = module.generate_launch_description()
    argument = _declared_launch_arguments(launch_description)['enable_realsense_scan_bridge']

    assert _text_value(argument.default_value) == 'true'


def test_auto_explore_does_not_pass_realsense_camera_arguments_through():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_camera_args',
    )
    launch_description = module.generate_launch_description()
    include_action = next(
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith('/g1_nav/launch/2d_g1_nav2_bringup.launch.py')
    )
    launch_arguments = dict(include_action.launch_arguments)

    removed_arguments = {
        'enable_realsense_camera': 'enable_realsense_camera',
        'realsense_camera_name': 'realsense_camera_name',
        'realsense_camera_namespace': 'realsense_camera_namespace',
        'realsense_camera_serial_no': 'realsense_camera_serial_no',
        'realsense_camera_config_file': 'realsense_camera_config_file',
        'realsense_json_file_path': 'realsense_json_file_path',
        'realsense_camera_log_level': 'realsense_camera_log_level',
        'realsense_camera_output': 'realsense_camera_output',
        'realsense_enable_color': 'realsense_enable_color',
        'realsense_enable_depth': 'realsense_enable_depth',
        'realsense_pointcloud_enable': 'realsense_pointcloud_enable',
        'realsense_align_depth_enable': 'realsense_align_depth_enable',
        'realsense_publish_tf': 'realsense_publish_tf',
        'realsense_tf_publish_rate': 'realsense_tf_publish_rate',
    }

    for name in removed_arguments:
        assert name not in launch_arguments


def test_auto_explore_does_not_pass_realsense_bridge_parameter_arguments_through():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_bridge_args',
    )
    launch_description = module.generate_launch_description()
    include_action = next(
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith('/g1_nav/launch/2d_g1_nav2_bringup.launch.py')
    )
    launch_arguments = dict(include_action.launch_arguments)

    removed_arguments = {
        'realsense_depth_image_topic',
        'realsense_depth_camera_info_topic',
        'realsense_scan_topic',
        'realsense_scan_output_frame',
        'realsense_scan_time',
        'realsense_scan_range_min',
        'realsense_scan_range_max',
        'realsense_scan_height',
    }

    for name in removed_arguments:
        assert name not in launch_arguments


def test_local_costmap_uses_scan_obstacle_layer():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    local_costmap = config['local_costmap']['local_costmap']['ros__parameters']

    assert local_costmap['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]

    obstacle_layer = local_costmap['obstacle_layer']
    assert obstacle_layer['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert obstacle_layer['observation_sources'] == 'scan'
    assert obstacle_layer['scan'] == {
        'topic': '/scan',
        'max_obstacle_height': 2.0,
        'clearing': True,
        'marking': True,
        'data_type': 'LaserScan',
        'inf_is_valid': True,
        'obstacle_min_range': 0.2,
        'obstacle_max_range': 2.5,
        'raytrace_min_range': 0.2,
        'raytrace_max_range': 3.0,
    }


def test_global_costmap_uses_scan_obstacle_layer_for_dynamic_replanning():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']

    assert global_costmap['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]

    obstacle_layer = global_costmap['obstacle_layer']
    assert obstacle_layer['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert obstacle_layer['observation_sources'] == 'scan'
    assert obstacle_layer['scan'] == {
        'topic': '/scan',
        'max_obstacle_height': 2.0,
        'clearing': True,
        'marking': True,
        'data_type': 'LaserScan',
        'inf_is_valid': True,
        'obstacle_min_range': 0.2,
        'obstacle_max_range': 2.5,
        'raytrace_min_range': 0.2,
        'raytrace_max_range': 3.0,
    }


def test_collision_monitor_uses_scan_and_safe_cmd_vel_topics():
    with (REPO_ROOT / 'config/collision_monitor_params.yaml').open() as stream:
        config = yaml.safe_load(stream)

    params = config['collision_monitor']['ros__parameters']

    assert params['base_frame_id'] == 'base'
    assert params['odom_frame_id'] == 'odom'
    assert params['cmd_vel_in_topic'] == 'cmd_vel'
    assert params['cmd_vel_out_topic'] == 'cmd_vel_safe'
    assert params['polygons'] == ['Slowdown', 'Stop']
    assert params['observation_sources'] == ['scan']
    assert params['scan'] == {
        'type': 'scan',
        'topic': '/scan',
        'enabled': True,
    }

    assert params['Slowdown']['action_type'] == 'slowdown'
    assert params['Slowdown']['slowdown_ratio'] == 0.3
    assert params['Slowdown']['points'] == [0.95, 0.32, 0.95, -0.32, 0.45, -0.32, 0.45, 0.32]
    assert params['Slowdown']['max_points'] == 8
    assert params['Stop']['action_type'] == 'stop'
    assert params['Stop']['points'] == [0.65, 0.24, 0.65, -0.24, 0.30, -0.24, 0.30, 0.24]
    assert params['Stop']['max_points'] == 6


def test_frontier_explorer_uses_global_costmap_snap_parameters():
    with (REPO_ROOT / 'config/frontier_explorer_params.yaml').open() as stream:
        config = yaml.safe_load(stream)

    params = config['frontier_explorer']['ros__parameters']

    assert params['global_costmap_topic'] == '/global_costmap/costmap'
    assert params['frontier_snap_radius'] == 0.5
    assert params['frontier_fallback_snap_radius'] == 1.0
    assert params['goal_clearance_radius'] == 0.1
    assert 'direct_frontier_goal_search_radius' not in params


def test_nav2_costmaps_use_tuned_circular_robot_radii():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    local_params = config['local_costmap']['local_costmap']['ros__parameters']
    global_params = config['global_costmap']['global_costmap']['ros__parameters']

    for params in (local_params, global_params):
        assert params['footprint_padding'] == 0.0
        assert 'footprint' not in params

    assert local_params['robot_radius'] == 0.2
    assert global_params['robot_radius'] == 0.3
    assert local_params['inflation_layer']['inflation_radius'] == 0.20
    assert global_params['inflation_layer']['inflation_radius'] == 0.3
    assert global_params['inflation_layer']['inflation_radius'] == global_params['robot_radius']
    assert local_params['inflation_layer']['inflation_radius'] < (
        global_params['inflation_layer']['inflation_radius']
    )


def test_dwb_follow_path_raises_obstacle_avoidance_priority():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    follow_path = config['controller_server']['ros__parameters']['FollowPath']

    assert follow_path['BaseObstacle.scale'] == 0.2


def test_bt_navigator_loads_escape_obstacle_plugin():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    plugin_names = config['bt_navigator']['ros__parameters']['plugin_lib_names']

    assert 'g1_escape_obstacle_action_bt_node' in plugin_names


def test_follow_path_disables_lateral_navigation_velocity():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    follow_path = config['controller_server']['ros__parameters']['FollowPath']
    velocity_smoother = config['velocity_smoother']['ros__parameters']

    assert follow_path['max_vel_y'] == 0.0
    assert follow_path['vy_samples'] == 1
    assert velocity_smoother['max_velocity'][1] == 0.0
    assert velocity_smoother['min_velocity'][1] == 0.0


def test_navigate_to_pose_embedded_recovery_uses_escape_obstacle():
    sequence = _named_sequence(
        'behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml',
        'EmbeddedRecoveryLevel1',
    )

    assert [child.tag for child in sequence] == [
        'CancelControl',
        'EscapeObstacle',
        'ClearEntireCostmap',
        'ClearEntireCostmap',
    ]
    escape_obstacle = sequence[1]
    required_attributes = {
        'clear_confirmations': '3',
        'cmd_vel_topic': 'cmd_vel',
        'costmap_topic': 'local_costmap/costmap_raw',
        'footprint_topic': 'local_costmap/published_footprint',
        'robot_base_frame': 'base',
        'longitudinal_speed': '0.25',
        'lateral_speed': '0.20',
        'lethal_cost_threshold': '254',
        'transform_tolerance': '0.2',
    }
    for key, value in required_attributes.items():
        assert escape_obstacle.attrib.get(key) == value


def test_navigate_through_poses_embedded_recovery_uses_escape_obstacle():
    sequence = _named_sequence(
        'behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml',
        'EmbeddedRecoveryLevel1',
    )

    assert [child.tag for child in sequence] == [
        'CancelControl',
        'EscapeObstacle',
        'ClearEntireCostmap',
        'ClearEntireCostmap',
    ]
