from g1_nav.navigation_protocol import (
    build_mark_poi_success_event,
    build_progress_event,
    parse_command,
)
from g1_nav.navigation_types import PoseRecord, StoredPoi


def test_parse_navigate_to_command_accepts_go2w_flat_schema():
    command = parse_command(
        {
            "action": "navigate_to",
            "request_id": "req_001",
            "sub_id": 7,
            "target_id": "POI_001",
        }
    )

    assert command.action == "navigate_to"
    assert command.request_id == "req_001"
    assert command.sub_id == 7
    assert command.target_id == "POI_001"


def test_parse_mark_current_poi_requires_flat_poi_object():
    command = parse_command(
        {
            "action": "mark_current_poi",
            "request_id": "req_mark",
            "sub_id": 1,
            "poi": {"id": "POI_006", "name": "会议室门口"},
        }
    )

    assert command.action == "mark_current_poi"
    assert command.poi_id == "POI_006"
    assert command.poi_name == "会议室门口"


def test_build_progress_event_rounds_remaining_distance_to_three_decimals():
    payload = build_progress_event("req_001", 7, 1.23456, "running")

    assert payload == {
        "event_type": "on_progress",
        "request_id": "req_001",
        "sub_id": 7,
        "remaining_distance": 1.235,
        "status": "running",
    }


def test_build_mark_poi_success_event_keeps_pose_as_metadata():
    poi = StoredPoi(
        poi_id="POI_006",
        name="会议室门口",
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        slam_session_id="session_a",
    )

    payload = build_mark_poi_success_event("req_mark", 1, poi)

    assert payload["event_type"] == "on_mark_poi_success"
    assert payload["poi"]["id"] == "POI_006"
    assert payload["poi"]["frame_id"] == "map"
    assert payload["poi"]["position"] == {"x": 12.3, "y": 4.5, "z": 0.0}
