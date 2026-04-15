from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from g1_nav.navigation_protocol import (
    build_arrived_event,
    build_error_event,
    build_progress_event,
    parse_command,
)


class ExecutorState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ABORTING = "aborting"
    ERROR = "error"


@dataclass
class ActiveTask:
    request_id: str
    sub_id: int
    target_id: str
    remaining_distance: float = 0.0


class NavigationExecutorCore:
    def __init__(self, bridge, pose_provider, poi_store):
        self.bridge = bridge
        self.pose_provider = pose_provider
        self.poi_store = poi_store
        self.state = ExecutorState.IDLE
        self.active_task: ActiveTask | None = None

    async def handle_message(self, payload: dict) -> list[dict]:
        command = parse_command(payload)
        if command.action == "navigate_to":
            poi = self.poi_store.get_required(command.target_id)
            if self.active_task is not None:
                await self.bridge.cancel_current()
            self.active_task = ActiveTask(
                command.request_id,
                command.sub_id,
                command.target_id,
            )
            self.state = ExecutorState.STARTING
            await self.bridge.start_navigation(
                command.request_id,
                command.sub_id,
                poi,
            )
            return []
        if command.action == "abort_navigation":
            if self.active_task is None:
                return []
            if self.active_task.request_id != command.request_id:
                return []
            await self.bridge.cancel_current()
            self.state = ExecutorState.IDLE
            paused = build_progress_event(
                self.active_task.request_id,
                self.active_task.sub_id,
                self.active_task.remaining_distance,
                "paused",
            )
            self.active_task = None
            return [paused]
        raise ValueError(f"unsupported command in core: {command.action}")

    async def handle_bridge_event(self, event: dict) -> list[dict]:
        if self.active_task is None:
            return []
        if event["kind"] == "progress":
            self.state = ExecutorState.NAVIGATING
            self.active_task.remaining_distance = float(event["remaining_distance"])
            return [
                build_progress_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    self.active_task.remaining_distance,
                    event.get("status", "running"),
                )
            ]
        if event["kind"] == "arrived":
            outbound = [
                build_arrived_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                )
            ]
            self.active_task = None
            self.state = ExecutorState.IDLE
            return outbound
        if event["kind"] == "error":
            outbound = [
                build_error_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    event["error_message"],
                )
            ]
            self.active_task = None
            self.state = ExecutorState.ERROR
            return outbound
        return []
