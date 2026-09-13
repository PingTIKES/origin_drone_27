"""
VFH+ 局部避障规划器（每机一个实例，命名空间 uavN）。

数据流（航点链路中的一环）：
  goal_planner / swarm_coordinator  →  waypoint_in（原始航点，本机 NED）
  stereo_depth_node                →  obstacles（障碍点云，机体 FLU）
  PX4 /px4_N/fmu/out/vehicle_local_position → 本机位姿（坐标换算用）
  本节点                           →  waypoint（避障修正后航点 → offboard）

算法（VFH+ 经典三步）：
  1) 极直方图：点云按方位分 bin（默认 72 bin = 5°），每 bin 记最近障碍距离，
     只统计与机体高差 |z| <= z_window 的点（同一高度层才算障碍）；
  2) 阈值化：bin_min < safe_dist 的方位判为"堵死"，连续开放 bin 组成候选谷；
  3) 选向：在与目标方向角差最小的候选谷里，选最靠近目标侧的边缘方位作为
     新航向，等距旋转原航点输出；目标方向本身边界开放则原样直通。
  四周全堵（360° 无候选谷）→ 发布当前位置原地悬停并告警。

升级路径：当前是"最近距离"直方图（保守、简单）。可升级为 VFH+ 原文的
障碍密度直方图（h = (a - b*d^2) 累加）+ 双阈值迟滞（减少边界抖动）。

注意：本节点不做路径记忆，是纯反应式局部避障；全局绕障靠上游规划。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleLocalPosition


def px4_qos() -> QoSProfile:
    # MicroXRCEAgent 转发 PX4 话题用的是 BEST_EFFORT + VOLATILE；
    # 订阅请求 TRANSIENT_LOCAL 会被 DDS 判不兼容，静默零消息（无报错）
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _ang_diff(a, b):
    """a-b 归一化到 [-pi, pi]。"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class VfhPlanner(Node):
    def __init__(self):
        super().__init__('vfh_planner')

        self.declare_parameter('px4_ns', 'px4_1')   # 本机飞控命名空间（读位姿用）
        self.declare_parameter('num_bins', 72)      # 方位分 bin 数（72 → 5°/bin）
        self.declare_parameter('safe_dist', 1.5)    # 安全距离 m（小于则该方位判堵）
        self.declare_parameter('z_window', 1.0)     # 障碍高差窗口 m（只统计同高度层）
        self.declare_parameter('min_range', 0.4)    # 点云近界（滤自身螺旋桨/噪声）
        self.declare_parameter('max_range', 8.0)    # 点云远界
        self.declare_parameter('hold_log_throttle', 2.0)  # 悬停告警节流 s

        self.px4_ns = str(self.get_parameter('px4_ns').value)
        self.num_bins = int(self.get_parameter('num_bins').value)
        self.safe_dist = float(self.get_parameter('safe_dist').value)
        self.z_win = float(self.get_parameter('z_window').value)
        self.min_r = float(self.get_parameter('min_range').value)
        self.max_r = float(self.get_parameter('max_range').value)
        self._hold_log_ns = float(
            self.get_parameter('hold_log_throttle').value) * 1e9

        self.create_subscription(
            PointCloud2, 'obstacles', self._cb_cloud, 5)
        self.create_subscription(
            Point, 'waypoint_in', self._cb_wp, 10)
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.px4_ns}/fmu/out/vehicle_local_position',
            self._cb_pos, px4_qos())
        self.pub = self.create_publisher(Point, 'waypoint', 10)

        self.bin_min = [float('inf')] * self.num_bins  # 机体 FLU 方位 → 最近障碍
        self.pos = None        # (x, y, heading) 本机 NED
        self._last_hold_log = 0
        self.get_logger().info(
            f'VFH+ 就绪：{self.num_bins} bin，安全距离 {self.safe_dist} m，'
            f'高差窗口 ±{self.z_win} m；等待 obstacles + waypoint_in')

    # ---------------- 回调 ----------------
    def _cb_pos(self, msg: VehicleLocalPosition):
        self.pos = (msg.x, msg.y, msg.heading)

    def _cb_cloud(self, msg: PointCloud2):
        """点云（机体 FLU）→ 方位直方图（每 bin 最近障碍距离）。"""
        bins = [float('inf')] * self.num_bins
        for x, y, z in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if abs(z) > self.z_win:
                continue
            d = math.hypot(x, y)
            if d < self.min_r or d > self.max_r:
                continue
            b = int((math.atan2(y, x) + math.pi)
                    / (2 * math.pi) * self.num_bins) % self.num_bins
            if d < bins[b]:
                bins[b] = d
        self.bin_min = bins

    def _cb_wp(self, msg: Point):
        """对原始航点做 VFH+ 选向修正后转发（本机 NED 系）。"""
        if self.pos is None:
            # 还没有位姿，无从换算坐标系，原样直通（起飞前的航点不受影响）
            self.pub.publish(msg)
            return

        px, py, heading = self.pos
        dx, dy = msg.x - px, msg.y - py
        goal_dist = math.hypot(dx, dy)
        if goal_dist < 0.3:
            self.pub.publish(msg)   # 已到跟前，直通
            return

        # 本机 NED 偏移 → 机体 FLU 偏移（h 为 NED 航向）
        ch, sh = math.cos(heading), math.sin(heading)
        bx = dx * ch + dy * sh          # 前
        by = dx * sh - dy * ch          # 左
        alpha_goal = math.atan2(by, bx)  # 目标方位（机体 FLU）

        goal_bin = self._bin_of(alpha_goal)
        if self.bin_min[goal_bin] >= self.safe_dist:
            self.pub.publish(msg)   # 目标方向开放，直通
            return

        # ---- 候选谷：连续开放 bin 段 ----
        open_bin = [self.bin_min[b] >= self.safe_dist
                    for b in range(self.num_bins)]
        if not any(open_bin):
            self._hold(msg)
            return
        valleys = self._find_valleys(open_bin)
        if not valleys:
            self._hold(msg)
            return

        # ---- 选谷：目标方位角差最小；谷内取最靠目标侧的边缘 ----
        best_valley = min(
            valleys,
            key=lambda v: min(abs(_ang_diff(self._bin_angle(b), alpha_goal))
                              for b in range(v[0], v[1] + 1)))
        lo, hi = best_valley
        # 谷内离目标最近的 bin
        best_bin = min(
            range(lo, hi + 1),
            key=lambda b: abs(_ang_diff(self._bin_angle(b), alpha_goal)))
        alpha_steer = self._bin_angle(best_bin)

        # 机体 FLU 方位 → 本机 NED 方向，等距旋转原航点
        dir_ned = heading - alpha_steer   # FLU(y左) 与 NED(y右) 方位角反号
        out = Point()
        out.x = px + goal_dist * math.cos(dir_ned)
        out.y = py + goal_dist * math.sin(dir_ned)
        out.z = msg.z
        self.pub.publish(out)
        self.get_logger().info(
            f'目标方位被堵（{self.bin_min[goal_bin]:.1f} m < '
            f'{self.safe_dist:.1f} m），绕向 {math.degrees(alpha_steer):.0f}°')

    # ---------------- 工具 ----------------
    def _bin_of(self, alpha):
        return int((alpha + math.pi) / (2 * math.pi)
                   * self.num_bins) % self.num_bins

    def _bin_angle(self, b):
        return (b + 0.5) / self.num_bins * 2 * math.pi - math.pi

    def _find_valleys(self, open_bin):
        """返回开放 bin 的连续段 [(lo, hi), ...]（处理 0 点环绕）。"""
        n = self.num_bins
        # 从一个关闭点开始线性扫描，避免把跨界的谷劈成两段
        try:
            start = open_bin.index(False)
        except ValueError:
            return []  # 全开（调用方已拦全关）
        valleys = []
        b = (start + 1) % n
        cur = None
        for _ in range(n):
            if open_bin[b]:
                if cur is None:
                    cur = [b, b]
                else:
                    cur[1] = b
            else:
                if cur is not None:
                    valleys.append((cur[0], cur[1]))
                    cur = None
            b = (b + 1) % n
        if cur is not None:
            valleys.append((cur[0], cur[1]))
        # 段可能跨界（lo > hi），展开成线性索引便于 range()
        return [(lo, hi if hi >= lo else hi + n) for lo, hi in valleys]

    def _hold(self, msg: Point):
        """360° 全堵：发布当前位置悬停（z 取原航点高度）。"""
        now = self.get_clock().now().nanoseconds
        if now - self._last_hold_log > self._hold_log_ns:
            self._last_hold_log = now
            self.get_logger().warn('四周障碍全堵，原地悬停等待')
        out = Point()
        out.x, out.y, out.z = self.pos[0], self.pos[1], msg.z
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VfhPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
