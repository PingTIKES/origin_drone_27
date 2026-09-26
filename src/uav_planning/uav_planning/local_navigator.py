"""Single output arbiter: local A* -> bounded VFH -> fixed-position hold.

Input/output waypoints are PX4 local NED; all map computation uses NWU.
No command is forwarded without a fresh valid pose and local obstacle map.
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Float32
from px4_msgs.msg import VehicleLocalPosition
from uav_planning.local_grid_planner import LocalGridPlanner, bounded_step


class LocalNavigator(Node):
    def __init__(self):
        super().__init__('local_navigator')
        defaults = dict(uav_id=1, px4_ns='px4_1', map_timeout=.6, pose_timeout=.3,
                        goal_timeout=1., step_distance=.4, planning_budget=.025,
                        max_speed=.6, braking_accel=.8, reaction_time=.3,
                        height_tolerance=.12, safety_priority_time=2.2,
                        align_enter_deg=30., align_exit_deg=15., align_speed=.15)
        for key, val in defaults.items(): self.declare_parameter(key, val)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if not 0 < self.p['align_exit_deg'] < self.p['align_enter_deg'] < 90 or \
                self.p['align_speed'] < 0:
            raise ValueError('path alignment thresholds must enforce forward travel')
        self.frame = f'uav{self.p["uav_id"]}_odom'
        self.pose = self.goal = self.safety_goal = self.map = None
        self.pose_at = self.goal_at = self.safety_at = self.map_at = -math.inf
        self.reset = None
        self.hold_point = None
        self.previous = None
        self.aligning = False
        self.state = None
        self.pub = self.create_publisher(Point, 'waypoint', 1)
        self.yaw_pub = self.create_publisher(Float32, 'desired_yaw', 1)
        self.state_pub = self.create_publisher(String, 'navigation_state', 1)
        self.path_pub = self.create_publisher(Path, 'local_path', 1)
        self.create_subscription(Point, 'waypoint_in', self.target, 1)
        self.create_subscription(Point, 'safety_waypoint', self.safety, 1)
        self.create_subscription(OccupancyGrid, 'local_map', self.grid, 1)
        self.create_subscription(VehicleLocalPosition,
                                 f'/{self.p["px4_ns"]}/fmu/out/vehicle_local_position', self.position, qos_profile_sensor_data)
        self.create_timer(.2, self.tick)

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def target(self, msg):
        if all(math.isfinite(v) for v in (msg.x,msg.y,msg.z)):
            self.goal, self.goal_at = msg, self.now()

    def safety(self, msg):
        if all(math.isfinite(v) for v in (msg.x,msg.y,msg.z)):
            self.safety_goal, self.safety_at = msg, self.now()

    def position(self, msg):
        reset = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if reset != self.reset:
            self.map, self.hold_point, self.goal, self.safety_goal = None, None, None, None
            self.aligning, self.previous = False, None
            self.reset = reset
        valid = msg.xy_valid and msg.z_valid and msg.v_xy_valid and all(
            math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.vx,msg.vy,msg.heading))
        self.pose = msg if valid else None
        self.pose_at = self.now()

    def grid(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id != self.frame or not 0 <= self.now()-stamp <= self.p['map_timeout']: return
        if stamp <= self.map_at or msg.info.resolution <= 0: return
        if msg.info.width*msg.info.height != len(msg.data): return
        self.map, self.map_at = msg, stamp

    def status(self, value):
        self.state_pub.publish(String(data=value))
        if value != self.state:
            self.get_logger().info(value)
            self.state = value

    def publish_path(self, path, height):
        msg = Path()
        msg.header.frame_id, msg.header.stamp = self.frame, self.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), height
            ps.pose.orientation.w = 1.
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def hold(self, reason, path=()):
        if self.hold_point is None:
            self.hold_point = Point(x=float(self.pose.x), y=float(self.pose.y), z=float(self.pose.z))
        self.pub.publish(self.hold_point)
        self.publish_path(path, -float(self.pose.z))
        self.status(reason)

    def tick(self):
        now = self.now()
        if self.pose is None or not 0 <= now-self.pose_at <= self.p['pose_timeout']:
            self.hold_point = None
            self.status('POSE_INVALID')
            return  # offboard independently stops its heartbeat for invalid position
        pose = self.pose
        safety = self.safety_goal is not None and 0 <= now-self.safety_at <= self.p['safety_priority_time']
        goal = self.safety_goal if safety else self.goal
        if goal is None or (not safety and not 0 <= now-self.goal_at <= self.p['goal_timeout']):
            self.hold('HOLD_NO_GOAL'); return
        if math.hypot(pose.vx, pose.vy) > self.p['max_speed']:
            self.hold('HOLD_OVERSPEED'); return
        if abs(pose.z-goal.z) > self.p['height_tolerance']:
            self.hold('HOLD_ALTITUDE_CHANGE_UNSUPPORTED'); return
        if self.map is None or not 0 <= now-self.map_at <= self.p['map_timeout']:
            self.hold('HOLD_MAP_STALE'); return
        if abs(self.map.info.origin.position.z + pose.z) > self.p['height_tolerance']:
            self.hold('HOLD_MAP_ALTITUDE'); return
        grid = LocalGridPlanner(np.array(self.map.data).reshape(self.map.info.height,self.map.info.width),
                                self.map.info.resolution,
                                (self.map.info.origin.position.x,self.map.info.origin.position.y))
        start, target = (pose.x,-pose.y), (goal.x,-goal.y)
        status, path = grid.plan(start, target, self.p['planning_budget'])
        # A plan that consumed the remaining sensor freshness budget cannot be sent.
        if self.now()-self.map_at > self.p['map_timeout']:
            self.hold('HOLD_MAP_STALE'); return
        step = self.p['step_distance']
        if status == 'BUDGET':
            speed = math.hypot(pose.vx,pose.vy)
            clearance = step + speed*self.p['reaction_time'] + speed*speed/(2*self.p['braking_accel'])
            out = grid.vfh(start,target,step,clearance,self.previous)
            if out is None:
                self.hold('HOLD_BUDGET_NO_CORRIDOR'); return
            path, status = [start,out], 'VFH_FALLBACK'
        elif not path:
            self.hold('HOLD_'+status)
            return
        out = bounded_step(start,path,step)
        if not grid.line_free(start,out):
            self.hold('HOLD_CORRIDOR_BLOCKED'); return
        if math.dist(start,out) > .05:
            direction = math.atan2(out[1]-start[1],out[0]-start[0])
            desired_yaw = -direction  # NWU path -> PX4 local NED heading
            error = (desired_yaw-pose.heading+math.pi)%(2*math.pi)-math.pi
            if abs(error) > math.radians(self.p['align_enter_deg']):
                if not self.aligning:
                    self.hold_point = None
                    self.previous = direction
                self.aligning = True
            self.yaw_pub.publish(Float32(data=float(desired_yaw)))
            if self.aligning:
                if abs(error) > math.radians(self.p['align_exit_deg']) or \
                        math.hypot(pose.vx,pose.vy) > self.p['align_speed']:
                    self.hold('ALIGNING_PATH', path)
                    return
                self.aligning = False
            self.previous = direction
        self.hold_point = None
        self.pub.publish(Point(x=float(out[0]), y=float(-out[1]), z=float(goal.z)))
        self.status(('SAFETY_' if safety else '')+status)
        self.publish_path(path, -float(goal.z))


def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
