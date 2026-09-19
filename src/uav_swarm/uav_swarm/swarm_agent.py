"""Per-vehicle swarm adapter and distributed predictive collision avoidance."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from uav_msgs.msg import SwarmState, TrajectoryIntent, SwarmCommand, SwarmAck
from uav_swarm.avoidance import (local_to_common, common_to_local, rotate_xy,
                                 select_safe_velocity)


def state_qos():
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def command_qos():
    return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=20)


def time_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)*1e-9


class SwarmAgent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        defaults = dict(uav_id=1, px4_ns='px4_1', offset_x=0., offset_y=0., yaw_offset=0.,
                        state_rate=10., intent_rate=5., avoidance_rate=10., state_timeout=.45,
                        command_timeout=1., max_speed=.6, time_horizon=2., safe_radius=.65,
                        delay_margin=.25, lookahead=.7, intent_steps=8, intent_dt=.25,
                        position_variance=.01)
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.p = {key:self.get_parameter(key).value for key in defaults}
        self.uid = int(self.p['uav_id'])
        self.offset = np.array([self.p['offset_x'], self.p['offset_y']], dtype=float)
        self.yaw = float(self.p['yaw_offset'])
        self.position_msg = None
        self.position_at = self.vio_at = -math.inf
        self.vio_valid = False
        self.mission_state = 'INIT'
        self.reset_counters = None
        self.frame_epoch = self.seq = self.intent_seq = 0
        self.goal_common = None
        self.planned_waypoint = None
        self.planned_at = -math.inf
        self.command_id = self.mission_id = 0
        self.neighbors = {}
        self.intents = {}
        self.last_safe_velocity = np.zeros(2)
        self.have_safe_velocity = False

        sq, cq = state_qos(), command_qos()
        self.state_pub = self.create_publisher(SwarmState, '/swarm/uav_state', sq)
        self.intent_pub = self.create_publisher(TrajectoryIntent, '/swarm/trajectory_intent', sq)
        self.ack_pub = self.create_publisher(SwarmAck, '/swarm/ack', cq)
        self.safety_pub = self.create_publisher(Point, 'safety_waypoint', 1)
        self.goal_pub = self.create_publisher(Point, 'waypoint_in', 1)
        self.command_pub = self.create_publisher(String, 'command', 10)
        self.create_subscription(VehicleLocalPosition,
                                 f'/{self.p["px4_ns"]}/fmu/out/vehicle_local_position',
                                 self.position, sq)
        self.create_subscription(String, 'vio_health', self.vio, 1)
        self.create_subscription(String, 'state', self.flight_state, 10)
        self.create_subscription(Point, 'waypoint', self.planned, 1)
        self.create_subscription(SwarmState, '/swarm/uav_state', self.peer_state, sq)
        self.create_subscription(TrajectoryIntent, '/swarm/trajectory_intent', self.peer_intent, sq)
        self.create_subscription(SwarmCommand, '/swarm/command', self.swarm_command, cq)
        self.create_timer(1./float(self.p['state_rate']), self.publish_state)
        self.create_timer(1./float(self.p['intent_rate']), self.publish_intent)
        self.create_timer(1./float(self.p['avoidance_rate']), self.avoid)
        self.get_logger().info(
            f'swarm_agent uav{self.uid}: offset={self.offset.tolist()}, yaw={self.yaw:.3f}')

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def pose_valid(self):
        msg = self.position_msg
        return msg is not None and 0 <= self.now()-self.position_at <= self.p['state_timeout'] and \
            msg.xy_valid and msg.z_valid and msg.v_xy_valid and all(
                math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.vx,msg.vy,msg.vz))

    def position(self, msg):
        reset = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.reset_counters is not None and reset != self.reset_counters:
            self.frame_epoch += 1
            self.goal_common = None
            self.get_logger().warn('PX4 local frame reset; waiting for a fresh swarm command')
        self.reset_counters = reset
        self.position_msg, self.position_at = msg, self.now()

    def vio(self, msg):
        self.vio_valid, self.vio_at = msg.data == 'VALID', self.now()

    def flight_state(self, msg): self.mission_state = msg.data

    def planned(self,msg):
        if all(math.isfinite(v) for v in (msg.x,msg.y,msg.z)):
            self.planned_waypoint=np.array([msg.x,msg.y,msg.z],dtype=float)
            self.planned_at=self.now()

    def common_position(self):
        m = self.position_msg
        return local_to_common(np.array([m.x,m.y,m.z]), self.offset, self.yaw)

    def common_velocity(self):
        m = self.position_msg
        xy = rotate_xy((m.vx,m.vy), self.yaw)
        return np.array([xy[0],xy[1],m.vz])

    def peer_state(self, msg):
        if int(msg.uav_id) == self.uid:return
        if msg.pose_valid and not all(math.isfinite(v) for v in (
                msg.position.x,msg.position.y,msg.position.z,msg.velocity.x,msg.velocity.y,
                msg.velocity.z,msg.position_variance)):return
        self.neighbors[int(msg.uav_id)] = (msg, self.now())

    def peer_intent(self,msg):
        if int(msg.uav_id)==self.uid or not msg.velocities:return
        velocity=msg.velocities[0]
        if not all(math.isfinite(v) for v in (velocity.x,velocity.y,velocity.z)):return
        self.intents[int(msg.uav_id)]=(np.array([velocity.x,velocity.y,velocity.z]),
                                      time_seconds(msg.valid_until),self.now())

    def publish_state(self):
        out = SwarmState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'swarm_ned'
        out.uav_id, out.seq = self.uid, self.seq
        self.seq += 1
        out.pose_valid = self.pose_valid()
        out.vio_valid = self.vio_valid and 0 <= self.now()-self.vio_at <= .5
        out.mission_state = self.mission_state
        out.frame_epoch = self.frame_epoch
        out.battery_remaining = float('nan')
        if out.pose_valid:
            p, v = self.common_position(), self.common_velocity()
            out.position = Point(x=float(p[0]),y=float(p[1]),z=float(p[2]))
            out.velocity = Vector3(x=float(v[0]),y=float(v[1]),z=float(v[2]))
            eph = getattr(self.position_msg, 'eph', math.nan)
            out.position_variance = float(eph*eph if math.isfinite(eph) and eph>0 else self.p['position_variance'])
        self.state_pub.publish(out)

    def preferred_velocity(self):
        if not self.pose_valid():return np.zeros(2)
        if self.planned_waypoint is not None and 0<=self.now()-self.planned_at<=.4:
            target=local_to_common(self.planned_waypoint,self.offset,self.yaw)
        elif self.goal_common is not None:target=self.goal_common
        else:return np.zeros(2)
        delta = target[:2]-self.common_position()[:2]
        distance = np.linalg.norm(delta)
        return np.zeros(2) if distance < .05 else delta/distance*min(distance,float(self.p['max_speed']))

    def publish_intent(self):
        if not self.pose_valid(): return
        out = TrajectoryIntent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'swarm_ned'
        out.uav_id, out.seq = self.uid, self.intent_seq
        self.intent_seq += 1
        dt = float(self.p['intent_dt'])
        out.time_step = dt
        valid = self.now()+dt*int(self.p['intent_steps'])+.2
        out.valid_until.sec, out.valid_until.nanosec = int(valid), int((valid-int(valid))*1e9)
        p = self.common_position()
        v = self.last_safe_velocity if self.have_safe_velocity else self.preferred_velocity()
        for k in range(1,int(self.p['intent_steps'])+1):
            q = p.copy(); q[:2] += v*dt*k
            out.points.append(Point(x=float(q[0]),y=float(q[1]),z=float(q[2])))
            out.velocities.append(Vector3(x=float(v[0]),y=float(v[1]),z=0.))
        self.intent_pub.publish(out)

    def swarm_command(self, msg):
        if int(msg.target_uav) not in (0,self.uid): return
        now = self.now()
        if time_seconds(msg.valid_until) < now:
            self.ack(msg, False, 'expired'); return
        if int(msg.mission_id) < self.mission_id or (int(msg.mission_id)==self.mission_id and
                                                     int(msg.command_id)<self.command_id):
            self.ack(msg, False, 'superseded'); return
        if msg.command not in ('goto','hold','land'):
            self.ack(msg, False, 'unsupported command'); return
        new_command=(int(msg.mission_id),int(msg.command_id)) != (self.mission_id,self.command_id)
        self.mission_id, self.command_id = int(msg.mission_id), int(msg.command_id)
        if msg.command == 'land':
            self.command_pub.publish(String(data='land'))
        else:
            if msg.command == 'hold':
                if not self.pose_valid(): self.ack(msg,False,'pose invalid'); return
                self.goal_common = self.common_position()
            else:
                values = (msg.target.x,msg.target.y,msg.target.z)
                if not all(math.isfinite(v) for v in values): self.ack(msg,False,'nonfinite target'); return
                self.goal_common = np.array(values,dtype=float)
            local = common_to_local(self.goal_common,self.offset,self.yaw)
            self.goal_pub.publish(Point(x=float(local[0]),y=float(local[1]),z=float(local[2])))
        if new_command:self.ack(msg, True, 'accepted')

    def ack(self, command, accepted, reason):
        out = SwarmAck()
        out.header.stamp = self.get_clock().now().to_msg()
        out.uav_id, out.mission_id, out.command_id = self.uid, command.mission_id, command.command_id
        out.accepted, out.reason, out.execution_state = accepted, reason, self.mission_state
        self.ack_pub.publish(out)

    def avoid(self):
        if not self.pose_valid(): return
        now = self.now()
        active=[]
        for uid,(msg,received) in list(self.neighbors.items()):
            age=now-received
            if not msg.pose_valid or not 0 <= age <= self.p['state_timeout']:
                if self.mission_state in ('TAKEOFF','MISSION'):
                    m=self.position_msg
                    self.safety_pub.publish(Point(x=float(m.x),y=float(m.y),z=float(m.z)))
                continue
            velocity=(msg.velocity.x,msg.velocity.y,msg.velocity.z)
            intent=self.intents.get(uid)
            if intent is not None and intent[1]>=now and 0<=now-intent[2]<=self.p['state_timeout']:
                velocity=intent[0]
            active.append(dict(uav_id=uid,position=(msg.position.x,msg.position.y,msg.position.z),
                               velocity=velocity,age=age,
                               variance=msg.position_variance))
        preferred=self.preferred_velocity()
        selected,feasible,margin=select_safe_velocity(
            self.common_position(),self.common_velocity(),preferred,active,
            max_speed=float(self.p['max_speed']),horizon=float(self.p['time_horizon']),
            base_radius=float(self.p['safe_radius']),delay_margin=float(self.p['delay_margin']),
            own_id=self.uid)
        self.last_safe_velocity=selected
        self.have_safe_velocity=True
        if active and (not feasible or margin < .15 or np.linalg.norm(selected-preferred)>.05):
            local_velocity=rotate_xy(selected,-self.yaw)
            m=self.position_msg
            dt=float(self.p['lookahead'])
            self.safety_pub.publish(Point(x=float(m.x+local_velocity[0]*dt),
                                          y=float(m.y+local_velocity[1]*dt),z=float(m.z)))


def main(args=None):
    rclpy.init(args=args)
    node=SwarmAgent()
    try:rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()
