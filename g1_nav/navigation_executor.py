#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import sys


DEFAULT_SERVER_URI = "ws://192.168.0.131:8100/ws/navigation/executor"
DEFAULT_POI_STORE_FILE = os.path.expanduser("~/.ros/g1_nav/poi_store.yaml")
DEFAULT_HEARTBEAT_INTERVAL = 30.0
DEFAULT_LOOP_CLOSURE_POLL_SEC = 1.0
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_SERVER_TIMEOUT = 10.0
DEFAULT_ROBOT_BASE_FRAME = "base_link"
DEFAULT_ODOM_TOPIC = "/lightning/odometry"
DEFAULT_SLAM_SESSION_ID = "live_session"


def _build_help_text() -> str:
    return "\n".join(
        [
            "usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] [ROS args...]",
            "",
            "Robot-side WebSocket navigation executor for g1_nav.",
            "",
            "options:",
            "  --server-uri URI        WebSocket endpoint for /ws/navigation/executor",
            "  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs",
            "  -h, --help              Show this help message and exit",
        ]
    )


def _parse_cli_args(raw_args):
    config = {
        "server_uri": DEFAULT_SERVER_URI,
        "poi_store_file": DEFAULT_POI_STORE_FILE,
        "heartbeat_interval": DEFAULT_HEARTBEAT_INTERVAL,
        "loop_closure_poll_sec": DEFAULT_LOOP_CLOSURE_POLL_SEC,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout": DEFAULT_SERVER_TIMEOUT,
        "robot_base_frame": DEFAULT_ROBOT_BASE_FRAME,
        "odom_topic": DEFAULT_ODOM_TOPIC,
        "slam_session_id": DEFAULT_SLAM_SESSION_ID,
    }
    ros_args = []
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--server-uri" and i + 1 < len(raw_args):
            config["server_uri"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--poi-store-file" and i + 1 < len(raw_args):
            config["poi_store_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        ros_args.append(arg)
        i += 1
    return config, ros_args


async def _send_heartbeat(websocket, interval_sec: float):
    while True:
        await asyncio.sleep(interval_sec)
        await websocket.send(json.dumps({"type": "ping"}, ensure_ascii=False))


async def _watch_loop_closure(pose_provider, poi_store, poll_sec: float):
    last_transform = None
    while True:
        await asyncio.sleep(poll_sec)
        try:
            current_transform = pose_provider.current_map_from_odom()
        except Exception:
            continue
        if current_transform == last_transform:
            continue
        poi_store.reproject_session(
            slam_session_id=pose_provider.current_slam_session_id(),
            map_from_odom=current_transform,
            translation_threshold=0.05,
            yaw_threshold=0.03,
        )
        last_transform = current_transform


async def main_async(raw_args):
    import rclpy
    import websockets
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    from g1_nav.navigation_executor_core import NavigationExecutorCore
    from g1_nav.nav2_action_bridge import Nav2ActionBridge
    from g1_nav.poi_store import LocalPoiStore
    from g1_nav.pose_provider import TfPoseProvider

    config, ros_args = _parse_cli_args(raw_args)
    rclpy.init(args=ros_args)
    node = Node("navigation_executor")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    poi_store = LocalPoiStore(config["poi_store_file"])
    if hasattr(poi_store, "load"):
        poi_store.load()

    core = None
    outbound_queue: asyncio.Queue = asyncio.Queue()

    async def on_bridge_event(event: dict):
        if core is None:
            return
        for payload in await core.handle_bridge_event(event):
            await outbound_queue.put(payload)

    bridge = Nav2ActionBridge(
        node=node,
        action_name=config["action_name"],
        server_timeout=config["server_timeout"],
        event_callback=on_bridge_event,
    )
    pose_provider = TfPoseProvider(
        node=node,
        tf_buffer=tf_buffer,
        robot_base_frame=config["robot_base_frame"],
        odom_topic=config["odom_topic"],
        slam_session_id_fn=lambda: config["slam_session_id"],
    )
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=pose_provider,
        poi_store=poi_store,
    )

    try:
        async with websockets.connect(config["server_uri"], ping_interval=None) as websocket:
            heartbeat_task = asyncio.create_task(
                _send_heartbeat(websocket, config["heartbeat_interval"])
            )
            loop_closure_task = asyncio.create_task(
                _watch_loop_closure(
                    pose_provider,
                    poi_store,
                    config["loop_closure_poll_sec"],
                )
            )
            try:
                async for raw_message in websocket:
                    payload = json.loads(raw_message)
                    if payload.get("type") == "pong":
                        continue
                    if payload.get("type") == "ping":
                        await websocket.send(json.dumps({"type": "pong"}, ensure_ascii=False))
                        continue
                    for outbound in await core.handle_message(payload):
                        await websocket.send(json.dumps(outbound, ensure_ascii=False))
                    while not outbound_queue.empty():
                        await websocket.send(
                            json.dumps(await outbound_queue.get(), ensure_ascii=False)
                        )
            finally:
                heartbeat_task.cancel()
                loop_closure_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await heartbeat_task
                with contextlib.suppress(asyncio.CancelledError):
                    await loop_closure_task
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)
    if any(arg in ("-h", "--help") for arg in raw_args):
        print(_build_help_text())
        return
    asyncio.run(main_async(raw_args))


if __name__ == "__main__":
    main()
