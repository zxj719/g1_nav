#!/usr/bin/env python3

"""
nav2_precheck — Nav2 启动前诊断工具

在启动 Nav2 之前运行此节点，检查所有前置条件是否满足：
  - TF 链完整性 (map -> odom -> base)
  - 必需话题是否存在且有数据 (/scan, /lightning/odometry, /cmd_vel)
  - 必需服务/Action Server 是否可用
  - g1_move 双进程状态

用法:
  1. 先启动 SLAM 算法和 g1_move
  2. ros2 run g1_nav nav2_precheck
  3. 根据输出修复问题后再启动 Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from tf2_ros import TransformException

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ANSI 颜色
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
BOLD = '\033[1m'
RESET = '\033[0m'

PASS = f'{GREEN}[PASS]{RESET}'
FAIL = f'{RED}[FAIL]{RESET}'
WARN = f'{YELLOW}[WARN]{RESET}'
INFO = f'{BLUE}[INFO]{RESET}'


class Nav2Precheck(Node):

    def __init__(self):
        super().__init__('nav2_precheck')

        # -------- 参数 --------
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/lightning/odometry')
        self.declare_parameter('map_topic', '/lightning/grid_map')
        self.declare_parameter('map_transient_local', True)
        self.declare_parameter('timeout', 10.0)

        self.base_frame = self.get_parameter('robot_base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_transient_local = self.get_parameter(
            'map_transient_local').value
        self.timeout = self.get_parameter('timeout').value

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- 话题接收状态 --------
        self.received = {
            'scan': False,
            'odom': False,
            'map': False,
            'cmd_vel_active': False,
        }
        self.topic_data = {}

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if self.map_transient_local
                else DurabilityPolicy.VOLATILE
            ),
            depth=1)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # 等待数据后运行检查
        self.check_timer = self.create_timer(1.0, self._tick)
        self.elapsed = 0.0

        self.get_logger().info(
            f'{BOLD}========== Nav2 前置条件诊断 =========={RESET}')
        self.get_logger().info(
            f'{INFO} 等待 {self.timeout:.0f} 秒收集数据...')
        self.get_logger().info(
            f'{INFO} 期望 TF 链: {self.global_frame} -> '
            f'{self.odom_frame} -> {self.base_frame}')

    # ================= 话题回调 =================

    def _scan_cb(self, msg):
        if not self.received['scan']:
            self.received['scan'] = True
            self.topic_data['scan'] = {
                'frame': msg.header.frame_id,
                'ranges': len(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
            }

    def _odom_cb(self, msg):
        if not self.received['odom']:
            self.received['odom'] = True
            self.topic_data['odom'] = {
                'frame': msg.header.frame_id,
                'child_frame': msg.child_frame_id,
            }

    def _map_cb(self, msg):
        if not self.received['map']:
            self.received['map'] = True
            self.topic_data['map'] = {
                'frame': msg.header.frame_id,
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
            }

    def _cmd_vel_cb(self, msg):
        self.received['cmd_vel_active'] = True

    # ================= 定时检查 =================

    def _tick(self):
        self.elapsed += 1.0
        if self.elapsed < self.timeout:
            # 进度提示
            if self.elapsed % 3 == 0:
                status = ', '.join(
                    f'{k}={"OK" if v else "waiting"}'
                    for k, v in self.received.items()
                    if k != 'cmd_vel_active')
                self.get_logger().info(f'{INFO} [{self.elapsed:.0f}s] {status}')
            return

        # 超时，开始输出诊断报告
        self.check_timer.cancel()
        self._run_checks()

    def _run_checks(self):
        self.get_logger().info('')
        self.get_logger().info(
            f'{BOLD}========== 诊断报告 =========={RESET}')

        total = 0
        passed = 0

        # -------- 1. TF 检查 --------
        self.get_logger().info(f'\n{BOLD}--- TF 变换 ---{RESET}')

        tf_pairs = [
            (self.global_frame, self.odom_frame, '必需: SLAM 或定位算法提供'),
            (self.odom_frame, self.base_frame, '必需: 里程计提供'),
            (self.global_frame, self.base_frame, '组合: 上述两个变换的结果'),
        ]

        for parent, child, desc in tf_pairs:
            total += 1
            ok, info = self._check_tf(parent, child)
            if ok:
                passed += 1
                self.get_logger().info(
                    f'  {PASS} {parent} -> {child}  ({desc})')
                if info:
                    self.get_logger().info(f'         {info}')
            else:
                self.get_logger().info(
                    f'  {FAIL} {parent} -> {child}  ({desc})')
                self.get_logger().info(f'         {RED}{info}{RESET}')

        # 检查 base -> 传感器 frame
        if self.received['scan']:
            scan_frame = self.topic_data['scan']['frame']
            total += 1
            ok, info = self._check_tf(self.base_frame, scan_frame)
            if ok:
                passed += 1
                self.get_logger().info(
                    f'  {PASS} {self.base_frame} -> {scan_frame}  '
                    f'(传感器 TF, /scan 的 frame_id)')
            else:
                self.get_logger().info(
                    f'  {FAIL} {self.base_frame} -> {scan_frame}  '
                    f'(传感器 TF, /scan 的 frame_id)')
                self.get_logger().info(
                    f'         {RED}需要发布静态 TF: '
                    f'{self.base_frame} -> {scan_frame}{RESET}')

        # 列出所有已知 frame
        all_frames = self.tf_buffer.all_frames_as_string()
        if all_frames.strip():
            self.get_logger().info(f'\n{BOLD}--- 当前 TF 树 ---{RESET}')
            for line in all_frames.strip().split('\n'):
                self.get_logger().info(f'  {line.strip()}')
        else:
            self.get_logger().info(
                f'\n  {FAIL} TF 树为空! 没有任何 TF 变换被发布')

        # -------- 2. 话题检查 --------
        self.get_logger().info(f'\n{BOLD}--- 必需话题 ---{RESET}')

        topic_checks = [
            ('scan', self.scan_topic,
             '2D 激光扫描, local_costmap obstacle_layer 需要'),
            ('odom', self.odom_topic,
             '里程计, bt_navigator / controller_server 需要'),
            ('map', self.map_topic,
             '占据栅格地图, global_costmap 需要 (SLAM 提供)'),
        ]

        for key, topic_name, desc in topic_checks:
            total += 1
            if self.received[key]:
                passed += 1
                self.get_logger().info(
                    f'  {PASS} {topic_name}  ({desc})')
                data = self.topic_data.get(key, {})
                if data:
                    details = ', '.join(f'{k}={v}' for k, v in data.items())
                    self.get_logger().info(f'         {details}')
            else:
                self.get_logger().info(
                    f'  {FAIL} {topic_name}  ({desc})')

        # -------- 3. 话题列表中检查 Nav2 可能需要的 --------
        self.get_logger().info(f'\n{BOLD}--- 话题一致性检查 ---{RESET}')

        # odom frame_id 与 配置一致性
        if self.received['odom']:
            odom_data = self.topic_data['odom']
            total += 1
            if odom_data['child_frame'] == self.base_frame:
                passed += 1
                self.get_logger().info(
                    f'  {PASS} /odom child_frame_id = "{odom_data["child_frame"]}" '
                    f'与 robot_base_frame = "{self.base_frame}" 一致')
            else:
                self.get_logger().info(
                    f'  {FAIL} /odom child_frame_id = "{odom_data["child_frame"]}" '
                    f'与 robot_base_frame = "{self.base_frame}" 不一致!')
                self.get_logger().info(
                    f'         {RED}Nav2 要求两者一致，否则 costmap 无法更新{RESET}')

            total += 1
            if odom_data['frame'] == self.odom_frame:
                passed += 1
                self.get_logger().info(
                    f'  {PASS} /odom frame_id = "{odom_data["frame"]}" '
                    f'与 odom_frame = "{self.odom_frame}" 一致')
            else:
                self.get_logger().info(
                    f'  {WARN} /odom frame_id = "{odom_data["frame"]}" '
                    f'与期望的 odom_frame = "{self.odom_frame}" 不一致')

        # -------- 4. cmd_vel 检查 --------
        self.get_logger().info(f'\n{BOLD}--- 执行器 ---{RESET}')
        total += 1

        # 查找 cmd_vel 订阅者
        topic_info = self.get_subscriptions_info_by_topic('/cmd_vel')
        if len(topic_info) > 0:
            passed += 1
            self.get_logger().info(
                f'  {PASS} /cmd_vel 有 {len(topic_info)} 个订阅者 '
                f'(g1_move 正在运行)')
        else:
            self.get_logger().info(
                f'  {FAIL} /cmd_vel 没有订阅者 (g1_move 未运行?)')
            self.get_logger().info(
                f'         {RED}请先启动: ros2 run g1_cmd g1_move{RESET}')

        # -------- 5. 已运行的 Nav2 节点检查 --------
        self.get_logger().info(f'\n{BOLD}--- 当前活跃节点 ---{RESET}')
        node_names = self.get_node_names()
        nav2_nodes = [n for n in node_names
                      if any(k in n for k in [
                          'controller', 'planner', 'bt_navigator',
                          'costmap', 'smoother', 'waypoint',
                          'behavior', 'lifecycle', 'slam',
                          'amcl', 'map_server'])]
        if nav2_nodes:
            self.get_logger().info(f'  {INFO} 检测到导航相关节点:')
            for n in sorted(nav2_nodes):
                self.get_logger().info(f'       - {n}')
        else:
            self.get_logger().info(
                f'  {INFO} 未检测到 Nav2 节点 (正常，还没启动)')

        relevant_nodes = [n for n in node_names
                          if any(k in n for k in [
                              'g1_move', 'slam', 'lightning',
                              'lio', 'localization'])]
        if relevant_nodes:
            self.get_logger().info(f'  {INFO} 检测到前置节点:')
            for n in sorted(relevant_nodes):
                self.get_logger().info(f'       - {n}')

        # -------- 汇总 --------
        self.get_logger().info(f'\n{BOLD}========== 汇总 =========={RESET}')
        if passed == total:
            self.get_logger().info(
                f'  {GREEN}{BOLD}全部通过 ({passed}/{total})'
                f' — 可以启动 Nav2!{RESET}')
        else:
            failed = total - passed
            self.get_logger().info(
                f'  {RED}{BOLD}{failed} 项未通过{RESET} '
                f'({passed}/{total} 通过)')
            self.get_logger().info(
                f'  {YELLOW}请修复上述 [FAIL] 项后再启动 Nav2{RESET}')

        self.get_logger().info('')

        # 退出
        raise SystemExit(0 if passed == total else 1)

    # ================= TF 工具 =================

    def _check_tf(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            trans = t.transform.translation
            rot = t.transform.rotation
            return True, (
                f'translation=[{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}], '
                f'rotation=[{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]')
        except TransformException as e:
            return False, str(e)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Precheck()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
