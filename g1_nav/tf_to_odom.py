#!/usr/bin/env python3
"""Bridge Lightning odometry or TF into a Nav2-friendly /odom stream."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_topic', '/odom')
        self.declare_parameter('source_odom_topic', '/lightning/odometry')
        self.declare_parameter('source_timeout', 0.5)
        self.declare_parameter('enable_tf_fallback', True)
        self.declare_parameter('publish_tf_from_source', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_topic = self.get_parameter('publish_topic').value
        self.source_odom_topic = self.get_parameter('source_odom_topic').value
        self.source_timeout = float(self.get_parameter('source_timeout').value)
        self.enable_tf_fallback = bool(
            self.get_parameter('enable_tf_fallback').value)
        self.publish_tf_from_source = bool(
            self.get_parameter('publish_tf_from_source').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = None
        if self.publish_tf_from_source:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, self.publish_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0), self.timer_callback)

        self.prev_transform = None
        self.prev_time = None
        self.last_source_rx = None
        self.last_source_msg = None
        self._warned_source_frame = False
        self._warned_source_child = False

        if self.source_odom_topic:
            self.source_sub = self.create_subscription(
                Odometry,
                self.source_odom_topic,
                self.source_odom_callback,
                20,
            )
        else:
            self.source_sub = None

        self.get_logger().info(
            'tf_to_odom ready: '
            f'source_topic={self.source_odom_topic or "<disabled>"} '
            f'fallback_tf={self.enable_tf_fallback} '
            f'publish_tf_from_source={self.publish_tf_from_source}'
        )

    def source_odom_callback(self, msg: Odometry):
        self.last_source_rx = self.get_clock().now()
        self.last_source_msg = msg
        self.publish_from_odom_msg(
            msg, stamp_override=self.get_clock().now().to_msg())

    def source_is_fresh(self) -> bool:
        if self.last_source_rx is None:
            return False
        age = (self.get_clock().now() - self.last_source_rx).nanoseconds / 1e9
        return age <= self.source_timeout

    def publish_from_odom_msg(self, msg: Odometry, stamp_override=None):
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            if not self._warned_source_frame:
                self.get_logger().warning(
                    'Rewriting source odom frame_id from '
                    f'{msg.header.frame_id!r} to {self.odom_frame!r}')
                self._warned_source_frame = True

        if msg.child_frame_id and msg.child_frame_id != self.base_frame:
            if not self._warned_source_child:
                self.get_logger().warning(
                    'Rewriting source odom child_frame_id from '
                    f'{msg.child_frame_id!r} to {self.base_frame!r}')
                self._warned_source_child = True

        stamp = stamp_override or msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def timer_callback(self):
        if self.last_source_msg is not None:
            self.publish_from_odom_msg(
                self.last_source_msg,
                stamp_override=self.get_clock().now().to_msg(),
            )
            return

        if not self.enable_tf_fallback:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation = transform.transform.rotation

        if self.prev_transform is not None and self.prev_time is not None:
            dt = (
                transform.header.stamp.sec
                + transform.header.stamp.nanosec * 1e-9
                - self.prev_time.sec
                - self.prev_time.nanosec * 1e-9
            )
            if dt > 0.001:
                odom.twist.twist.linear.x = (
                    transform.transform.translation.x
                    - self.prev_transform.translation.x
                ) / dt
                odom.twist.twist.linear.y = (
                    transform.transform.translation.y
                    - self.prev_transform.translation.y
                ) / dt

        self.prev_transform = transform.transform
        self.prev_time = transform.header.stamp
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
