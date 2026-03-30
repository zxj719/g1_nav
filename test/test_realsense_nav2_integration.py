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
    if isinstance(substitution, tuple):
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


def test_bringup_declares_realsense_scan_bridge_arguments():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch',
    )
    launch_description = module.generate_launch_description()
    argument_names = {
        entity.name
        for entity in launch_description.entities
        if type(entity).__name__ == 'DeclareLaunchArgument'
    }

    expected_arguments = {
        'enable_realsense_scan_bridge',
        'realsense_depth_image_topic',
        'realsense_depth_camera_info_topic',
        'realsense_scan_topic',
        'realsense_scan_output_frame',
        'realsense_scan_time',
        'realsense_scan_range_min',
        'realsense_scan_range_max',
        'realsense_scan_height',
    }

    assert expected_arguments.issubset(argument_names)


def test_bringup_contains_realsense_depth_to_scan_node():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_for_node',
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


def test_auto_explore_passes_realsense_scan_arguments_through():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch',
    )
    launch_description = module.generate_launch_description()
    include_action = next(
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
    )
    launch_arguments = dict(include_action.launch_arguments)

    expected_arguments = {
        'enable_realsense_scan_bridge': 'enable_realsense_scan_bridge',
        'realsense_depth_image_topic': 'realsense_depth_image_topic',
        'realsense_depth_camera_info_topic': 'realsense_depth_camera_info_topic',
        'realsense_scan_topic': 'realsense_scan_topic',
        'realsense_scan_output_frame': 'realsense_scan_output_frame',
        'realsense_scan_time': 'realsense_scan_time',
        'realsense_scan_range_min': 'realsense_scan_range_min',
        'realsense_scan_range_max': 'realsense_scan_range_max',
        'realsense_scan_height': 'realsense_scan_height',
    }

    for name, expected in expected_arguments.items():
        assert name in launch_arguments
        assert _text_value(launch_arguments[name]) == expected


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
    assert params['frontier_snap_radius'] == 1.0
    assert params['goal_clearance_radius'] == 0.35
    assert 'direct_frontier_goal_search_radius' not in params


def test_nav2_costmaps_use_circular_robot_radius():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    for costmap_name in ('local_costmap', 'global_costmap'):
        params = config[costmap_name][costmap_name]['ros__parameters']

        assert params['robot_radius'] == 0.35
        assert params['footprint_padding'] == 0.0
        assert 'footprint' not in params
