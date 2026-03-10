#!/usr/bin/env python3
"""
tf_to_odom - 从 TF (odom -> base) 生成 nav_msgs/Odometry 话题

Lightning SLAM 只发布 TF 不发布 /odom 话题，
Nav2 的 bt_navigator 需要 odom 话题获取机器人速度。
此节点监听 TF 并以固定频率发布 /odom。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('publish_rate', 30.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('publish_rate').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.prev_transform = None
        self.prev_time = None

        self.get_logger().info(
            f'TF->Odom: {self.odom_frame} -> {self.base_frame} => /odom'
        )

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z
        odom.pose.pose.orientation = t.transform.rotation

        # 简单速度估算
        if self.prev_transform is not None and self.prev_time is not None:
            dt = (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9) - \
                 (self.prev_time.sec + self.prev_time.nanosec * 1e-9)
            if dt > 0.001:
                odom.twist.twist.linear.x = (
                    t.transform.translation.x -
                    self.prev_transform.translation.x
                ) / dt
                odom.twist.twist.linear.y = (
                    t.transform.translation.y -
                    self.prev_transform.translation.y
                ) / dt

        self.prev_transform = t.transform
        self.prev_time = t.header.stamp

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
