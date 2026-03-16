#!/usr/bin/env python3
"""Cache a volatile map topic and republish it as a latched /map topic."""

from __future__ import annotations

import os
import pickle
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class MapCacheBridge(Node):
    def __init__(self):
        super().__init__('map_cache_bridge')

        self.declare_parameter('source_topic', '/lightning/grid_map')
        self.declare_parameter('target_topic', '/map')
        self.declare_parameter('live_map_ready_topic', '/map_live_ready')
        self.declare_parameter('target_frame_id', 'map')
        self.declare_parameter('republish_period', 2.0)
        self.declare_parameter(
            'cache_path',
            os.path.expanduser('~/.ros/g1_nav_lightning_map_cache.pkl'),
        )
        self.declare_parameter('load_cache_on_startup', False)
        self.declare_parameter('save_cache_on_update', True)
        self.declare_parameter('auto_load_cache_when_source_silent', True)
        self.declare_parameter('source_silence_before_cache_load', 3.0)

        self.source_topic = self.get_parameter('source_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.live_map_ready_topic = self.get_parameter('live_map_ready_topic').value
        self.target_frame_id = self.get_parameter('target_frame_id').value
        self.republish_period = float(
            self.get_parameter('republish_period').value)
        self.cache_path = self.get_parameter('cache_path').value
        self.load_cache_on_startup = bool(
            self.get_parameter('load_cache_on_startup').value)
        self.save_cache_on_update = bool(
            self.get_parameter('save_cache_on_update').value)
        self.auto_load_cache_when_source_silent = bool(
            self.get_parameter('auto_load_cache_when_source_silent').value)
        self.source_silence_before_cache_load = float(
            self.get_parameter('source_silence_before_cache_load').value)

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid, self.target_topic, target_qos)
        self.live_map_ready_pub = self.create_publisher(
            Bool, self.live_map_ready_topic, target_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.source_topic, self.map_callback, source_qos)

        self.latest_map = None
        self.start_time = self.get_clock().now()
        self.live_map_received = False
        self.cache_fallback_loaded = False
        self.publish_live_map_ready()

        if self.load_cache_on_startup:
            self.load_cache()

        if self.republish_period > 0.0:
            self.republish_timer = self.create_timer(
                self.republish_period, self.republish_cached_map)
        else:
            self.republish_timer = None

        self.warn_timer = self.create_timer(5.0, self.warn_if_empty)

        self.get_logger().info(
            'map_cache_bridge ready: '
            f'{self.source_topic} -> {self.target_topic}, '
            f'cache={self.cache_path}'
        )

    def map_callback(self, msg: OccupancyGrid):
        if self.target_frame_id:
            msg.header.frame_id = self.target_frame_id
        self.latest_map = msg
        self.live_map_received = True
        self.publish_live_map_ready()
        self.publish_map(msg, reason='live')
        if self.save_cache_on_update:
            self.save_cache(msg)

    def republish_cached_map(self):
        if self.latest_map is None:
            return
        self.publish_map(self.latest_map, reason='cached')

    def warn_if_empty(self):
        if self.latest_map is not None:
            return
        if self.try_bootstrap_from_cache():
            return
        self.get_logger().warning(
            'No map cached yet. If SLAM started before this bridge, '
            'wait for the next /lightning/grid_map update or restart SLAM.')

    def try_bootstrap_from_cache(self) -> bool:
        if self.live_map_received or self.cache_fallback_loaded:
            return False
        if not self.auto_load_cache_when_source_silent:
            return False
        age = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if age < self.source_silence_before_cache_load:
            return False
        if not os.path.exists(self.cache_path):
            return False
        self.load_cache(reason='source-silence-cache')
        self.cache_fallback_loaded = self.latest_map is not None
        return self.cache_fallback_loaded

    def publish_map(self, msg: OccupancyGrid, reason: str):
        self.map_pub.publish(msg)
        self.get_logger().info(
            f'Published {reason} map '
            f'({msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f})',
            throttle_duration_sec=5.0,
        )

    def publish_live_map_ready(self):
        msg = Bool()
        msg.data = self.live_map_received
        self.live_map_ready_pub.publish(msg)

    def save_cache(self, msg: OccupancyGrid):
        try:
            os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
            payload = {
                'header_frame_id': msg.header.frame_id,
                'header_stamp_sec': msg.header.stamp.sec,
                'header_stamp_nanosec': msg.header.stamp.nanosec,
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin_position': (
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    msg.info.origin.position.z,
                ),
                'origin_orientation': (
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w,
                ),
                'data': list(msg.data),
            }
            with open(self.cache_path, 'wb') as fp:
                pickle.dump(payload, fp)
        except Exception as exc:  # pragma: no cover - runtime diagnostic path
            self.get_logger().warning(f'Failed to save map cache: {exc}')

    def load_cache(self, reason: str = 'startup-cache'):
        if not os.path.exists(self.cache_path):
            return

        try:
            with open(self.cache_path, 'rb') as fp:
                payload = pickle.load(fp)
            msg = self.deserialize_map(payload)
            if self.target_frame_id:
                msg.header.frame_id = self.target_frame_id
            self.latest_map = msg
            self.publish_map(msg, reason=reason)
        except Exception as exc:  # pragma: no cover - runtime diagnostic path
            self.get_logger().warning(f'Failed to load map cache: {exc}')

    def deserialize_map(self, payload: dict[str, Any]) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = payload['header_frame_id']
        msg.header.stamp.sec = int(payload['header_stamp_sec'])
        msg.header.stamp.nanosec = int(payload['header_stamp_nanosec'])
        msg.info.width = int(payload['width'])
        msg.info.height = int(payload['height'])
        msg.info.resolution = float(payload['resolution'])
        (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.position.z,
        ) = payload['origin_position']
        (
            msg.info.origin.orientation.x,
            msg.info.origin.orientation.y,
            msg.info.origin.orientation.z,
            msg.info.origin.orientation.w,
        ) = payload['origin_orientation']
        msg.data = payload['data']
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MapCacheBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
