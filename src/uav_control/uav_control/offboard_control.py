"""
PX4 Offboard 控制节点（每架无人机一个实例）。

职责（对应框架文档 4.1 节）：
  - 持续 10 Hz 发布 OffboardControlMode + TrajectorySetpoint（Offboard 心跳）
  - 自动完成：进入 Offboard -> 解锁 -> 起飞到指定高度 -> 悬停
  - 悬停后跟踪上层（集群调度）通过 ~/waypoint 下发的航点（本机本地 NED 系）
  - 收到 ~/command = "land" 后降落并上锁
  - 通过 ~/state 向集群调度汇报状态

坐标约定：全部使用 PX4 本地 NED 系（北 x / 东 y / 下 z，高度 = -z）。
话题：飞控侧在 px4_ns（默认 px4_1，多机为 px4_1..px4_4）命名空间下；
      上层接口在本节点自身命名空间（launch 中设为 uavN）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)


def px4_qos() -> QoSProfile:
    # MicroXRCEAgent 转发 PX4 话题用的是 BEST_EFFORT + VOLATILE；
    # 订阅请求 TRANSIENT_LOCAL 会被 DDS 判不兼容，静默零消息（无报错）
    """PX4 uXRCE-DDS 要求的 QoS。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# PX4 自定义模式 / 命令常量
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2
CMD_DO_SET_MODE = 176          # MAV_CMD_DO_SET_MODE
CMD_COMPONENT_ARM_DISARM = 400  # MAV_CMD_COMPONENT_ARM_DISARM
CMD_NAV_LAND = 21              # MAV_CMD_NAV_LAND
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


class OffboardControl(Node):
    """状态机：INIT -> ARMING -> TAKEOFF -> MISSION(跟踪航点) -> LAND -> DONE"""

    def __init__(self):
        super().__init__('offboard_control')

        # ---- 参数 ----
        self.declare_parameter('px4_ns', 'px4_1')       # 飞控命名空间
        self.declare_parameter('px4_instance', 1)       # SITL 实例号（sysid = instance + 1）
        self.declare_parameter('takeoff_alt', 2.0)      # 起飞高度 m（正数）
        self.declare_parameter('takeoff_reach_time', 1.0)  # 连续到达时间，防止估计瞬态误判
        self.declare_parameter('auto_takeoff', True)    # 是否自动起飞（仿真默认开）
        self.declare_parameter('reach_tol', 0.25)       # 航点到达容差 m
        self.declare_parameter('waypoint_timeout', 1.0)
        self.declare_parameter('forward_heading_limit_deg', 35.0)
        self.declare_parameter('max_yaw_rate_deg_s', 45.0)
        self.declare_parameter('pose_timeout', .5)
        self.declare_parameter('require_vio', False)
        self.declare_parameter('target_system', 0)  # 0 preserves SITL instance+1

        self.px4_ns = self.get_parameter('px4_ns').value
        self.sysid = int(self.get_parameter('px4_instance').value) + 1
        if int(self.get_parameter('target_system').value)>0:
            self.sysid=int(self.get_parameter('target_system').value)
        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.takeoff_reach_time = float(self.get_parameter('takeoff_reach_time').value)
        self.auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        self.wp_timeout = float(self.get_parameter('waypoint_timeout').value)
        self.forward_heading_limit = math.radians(float(self.get_parameter('forward_heading_limit_deg').value))
        self.max_yaw_rate = math.radians(float(self.get_parameter('max_yaw_rate_deg_s').value))
        if not 0 < self.forward_heading_limit < math.pi/2:
            raise ValueError('forward heading limit must be below 90 degrees')
        if not 0 < self.max_yaw_rate <= 2*math.pi:
            raise ValueError('max_yaw_rate_deg_s must be between 0 and 360')
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.require_vio = bool(self.get_parameter('require_vio').value)
        self.pos_at = self.wp_at = self.vio_at = self.yaw_at = -math.inf
        self.vio_ok = False
        self.hold_target = self.desired_yaw = self.yaw_setpoint = self.pose_reset = None
        self.heading_hold_target = None
        self.last_yaw_setpoint_at = None
        self.takeoff_xy = None
        self.takeoff_reached_since = None

        qos = px4_qos()

        # ---- 飞控接口（px4_ns/fmu/...）----
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, f'/{self.px4_ns}/fmu/in/trajectory_setpoint', qos)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand, f'/{self.px4_ns}/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position',
            self._cb_local_pos, qos)
        self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status',
            self._cb_status, qos)

        # ---- 上层接口（本节点命名空间 uavN 下）----
        self.create_subscription(Point, 'waypoint', self._cb_waypoint, 10)
        self.create_subscription(String, 'command', self._cb_command, 10)
        self.create_subscription(String, 'vio_health', self._cb_vio, 1)
        self.create_subscription(Float32, 'desired_yaw', self._cb_yaw, 1)
        self.create_service(Trigger, 'start_mission', self._start_mission)
        self.pub_state = self.create_publisher(String, 'state', 10)

        # ---- 内部状态 ----
        self.state = 'INIT'
        self.heartbeat_count = 0
        self.local_pos = VehicleLocalPosition()
        self.status = VehicleStatus()
        self.have_pos = False
        # 当前目标航点（NED），None 表示保持当前位置
        self.target = None

        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            f'Offboard 节点就绪：px4_ns=/{self.px4_ns}, sysid={self.sysid}, '
            f'起飞高度 {self.takeoff_alt} m')

    # ---------------- 回调 ----------------
    def _cb_local_pos(self, msg: VehicleLocalPosition):
        reset=(msg.xy_reset_counter,msg.z_reset_counter,msg.heading_reset_counter)
        if self.pose_reset is not None and reset != self.pose_reset and self.state in ('ARMING','TAKEOFF','MISSION'):
            self.state='FAULT'
        self.pose_reset=reset
        self.local_pos = msg
        self.have_pos = msg.xy_valid and msg.z_valid and all(math.isfinite(v) for v in (msg.x,msg.y,msg.z))
        self.pos_at = msg.timestamp*1e-6

    def _cb_status(self, msg: VehicleStatus):
        self.status = msg

    def _cb_waypoint(self, msg: Point):
        """上层下发航点（本机本地 NED）。"""
        if not all(math.isfinite(v) for v in (msg.x,msg.y,msg.z)):return
        # Navigation may publish a ground-level HOLD before takeoff. This
        # controller only accepts cruise-altitude waypoints; landing has its
        # own command path.
        if abs(msg.z + self.takeoff_alt) > self.reach_tol:return
        self.target = (msg.x, msg.y, msg.z)
        self.wp_at=self.get_clock().now().nanoseconds*1e-9
        self.hold_target=None

    def _cb_vio(self,msg):
        self.vio_ok=msg.data=='VALID'
        self.vio_at=self.get_clock().now().nanoseconds*1e-9

    def _cb_yaw(self,msg):
        if math.isfinite(msg.data):
            self.desired_yaw=float(msg.data)
            self.yaw_at=self.get_clock().now().nanoseconds*1e-9

    def _start_mission(self,request,response):
        response.success=self.state=='INIT'
        if response.success:self.auto_takeoff=True
        response.message='Start requested; waiting for valid position/VIO' if response.success else 'Restart node after resolving fault/landing'
        return response

    def _cb_command(self, msg: String):
        if msg.data == 'land' and self.state in ('MISSION', 'TAKEOFF'):
            self.get_logger().info('收到降落指令')
            self.state = 'LAND'

    # ---------------- 飞控指令 ----------------
    def _send_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = command
        msg.target_system = self.sysid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    def _engage_offboard(self):
        self._send_vehicle_command(
            CMD_DO_SET_MODE, p1=1.0, p2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def _arm(self):
        self._send_vehicle_command(CMD_COMPONENT_ARM_DISARM, p1=1.0)

    def _land(self):
        self._send_vehicle_command(CMD_NAV_LAND)

    # ---------------- 心跳与设定点 ----------------
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def _publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')]*3
        msg.acceleration = [float('nan')]*3
        msg.yawspeed = float('nan')
        msg.yaw = float('nan')
        if self.yaw_setpoint is None and math.isfinite(self.local_pos.heading):
            self.yaw_setpoint=float(self.local_pos.heading)
        now=self.get_clock().now().nanoseconds*1e-9
        if self.desired_yaw is not None and 0<=now-self.yaw_at<1. and self.yaw_setpoint is not None:
            delta=(self.desired_yaw-self.yaw_setpoint+math.pi)%(2*math.pi)-math.pi
            dt=.1 if self.last_yaw_setpoint_at is None else max(0.,min(.2,now-self.last_yaw_setpoint_at))
            limit=self.max_yaw_rate*dt
            self.yaw_setpoint+=max(-limit,min(limit,delta))
            msg.yaw=self.yaw_setpoint
        self.last_yaw_setpoint_at=now
        self.pub_setpoint.publish(msg)

    def _dist_to(self, x, y, z):
        return math.sqrt((self.local_pos.x - x) ** 2 +
                         (self.local_pos.y - y) ** 2 +
                         (self.local_pos.z - z) ** 2)

    # ---------------- 主循环 10 Hz ----------------
    def _tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        valid=self.have_pos and 0<=now-self.pos_at<=self.pose_timeout
        valid=valid and (not self.require_vio or (self.vio_ok and 0<=now-self.vio_at<=.5))
        if not valid and self.state in ('ARMING','TAKEOFF','MISSION'):
            self.state='FAULT'
            self.get_logger().error('Position/VIO invalid; relinquishing Offboard to configured PX4 failsafe')
        if self.state=='FAULT' or (self.state=='INIT' and not valid):
            self.pub_state.publish(String(data=self.state))
            return
        # Offboard 心跳必须始终发布（否则 0.5 s 后飞控退出 Offboard）
        self._publish_offboard_mode()

        if self.state == 'INIT':
            if not self.have_pos:
                return
            # 先原地保持
            self._publish_setpoint(self.local_pos.x, self.local_pos.y, self.local_pos.z)
            self.heartbeat_count += 1
            # 官方推荐：先发约 1 s 心跳再切 Offboard + 解锁
            if self.heartbeat_count >= 10 and self.auto_takeoff:
                self._engage_offboard()
                self._arm()
                self.state = 'ARMING'
                self.get_logger().info('请求 Offboard + 解锁')

        elif self.state == 'ARMING':
            self._publish_setpoint(self.local_pos.x, self.local_pos.y, self.local_pos.z)
            if (self.status.arming_state == ARMING_STATE_ARMED and
                    self.status.nav_state == NAV_STATE_OFFBOARD):
                self.state = 'TAKEOFF'
                self.takeoff_xy = (self.local_pos.x,self.local_pos.y)
                self.takeoff_reached_since = None
                self.get_logger().info('已解锁并进入 Offboard，开始起飞')
            else:
                # 未成功则重发
                self._engage_offboard()
                self._arm()

        elif self.state == 'TAKEOFF':
            x, y = self.takeoff_xy or (self.local_pos.x,self.local_pos.y)
            z = -self.takeoff_alt
            self._publish_setpoint(x, y, z)
            if self._dist_to(x, y, z) < self.reach_tol:
                if self.takeoff_reached_since is None:
                    self.takeoff_reached_since = now
                elif now-self.takeoff_reached_since >= self.takeoff_reach_time:
                    self.state = 'MISSION'
                    self.hold_target = (x, y, z)
                    self.get_logger().info(f'稳定到达起飞高度 {self.takeoff_alt} m，进入任务模式')
            else:
                self.takeoff_reached_since = None

        elif self.state == 'MISSION':
            if self.target is not None and 0<=now-self.wp_at<=self.wp_timeout:
                dx = self.target[0] - self.local_pos.x
                dy = self.target[1] - self.local_pos.y
                if math.hypot(dx,dy) > .05 and (
                        not math.isfinite(self.local_pos.heading) or
                        abs((math.atan2(dy,dx)-self.local_pos.heading+math.pi)%(2*math.pi)-math.pi)
                        > self.forward_heading_limit):
                    # Reject a translation toward the side or rear even if a
                    # stale waypoint arrives while the navigator is turning.
                    if self.heading_hold_target is None:
                        self.heading_hold_target=(self.local_pos.x,self.local_pos.y,self.target[2])
                    self._publish_setpoint(*self.heading_hold_target)
                else:
                    self.heading_hold_target=None
                    self._publish_setpoint(*self.target)
            else:
                self.heading_hold_target=None
                if self.hold_target is None:
                    self.hold_target=(self.local_pos.x,self.local_pos.y,self.local_pos.z)
                self._publish_setpoint(*self.hold_target)

        elif self.state == 'LAND':
            self._land()
            self.state = 'DONE'

        elif self.state == 'DONE':
            if self.status.arming_state != ARMING_STATE_ARMED:
                self.get_logger().info('已降落上锁')
                self.state = 'IDLE'

        # 状态上报（供集群调度聚合）
        s = String()
        s.data = self.state
        self.pub_state.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
