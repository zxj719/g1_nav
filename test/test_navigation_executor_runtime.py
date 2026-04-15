import asyncio
from pathlib import Path

from g1_nav.navigation_executor_core import ExecutorState, NavigationExecutorCore
from g1_nav.navigation_types import PoseRecord
from g1_nav.poi_store import LocalPoiStore


class FakeBridge:
    def __init__(self):
        self.started = []
        self.canceled = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))

    async def cancel_current(self):
        self.canceled += 1


class FakePoseProvider:
    async def capture_for_poi(self):
        raise AssertionError("mark_current_poi is not part of this test")


def test_navigate_to_starts_bridge_and_progress_event_is_forwarded(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        outbound = await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        state_after_start = core.state
        forwarded = await core.handle_bridge_event(
            {
                "kind": "progress",
                "request_id": "req_1",
                "sub_id": 1,
                "remaining_distance": 2.25,
                "status": "running",
            }
        )
        state_after_progress = core.state
        return outbound, forwarded, state_after_start, state_after_progress

    outbound, forwarded, state_after_start, state_after_progress = asyncio.run(
        run_scenario()
    )

    assert outbound == []
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert state_after_start == ExecutorState.STARTING
    assert state_after_progress == ExecutorState.NAVIGATING
    assert forwarded == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 2.25,
            "status": "running",
        }
    ]


def test_abort_navigation_emits_paused_progress(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        return await core.handle_message(
            {
                "action": "abort_navigation",
                "request_id": "req_1",
            }
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.canceled == 1
    assert outbound == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 0.0,
            "status": "paused",
        }
    ]
