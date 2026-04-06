import importlib.util
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
        'realsense_scan_range_max': '3.0',
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


def test_frontier_explorer_uses_global_costmap_snap_parameters():
    with (REPO_ROOT / 'config/frontier_explorer_params.yaml').open() as stream:
        config = yaml.safe_load(stream)

    params = config['frontier_explorer']['ros__parameters']

    assert params['global_costmap_topic'] == '/global_costmap/costmap'
    assert params['frontier_snap_radius'] == 0.3
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
