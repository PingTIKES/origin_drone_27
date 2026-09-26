#!/usr/bin/env python3
"""Guarded OpenVINS odomimu -> PX4 external vision. No ground-truth input."""
import math
import json
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
from tf2_ros import TransformBroadcaster
from uav_localization.vio_geometry import convert
from uav_localization.vio_tf_geometry import (FLIP, VioOdomAlignment,
                                              vio_body_pose, px4_body_pose)
from uav_localization.calibration import transform
from uav_localization.vio_recovery import VioRecovery


class VioBridge(Node):
    def __init__(self):
        super().__init__('vio_to_px4')
        for k,v in dict(uav_id=1,px4_ns='px4_1',max_age=.25,max_position_variance=1.,
                        max_orientation_variance=.25,max_velocity_variance=1.,
                        max_speed=4.,max_jump=.4,expected_world='global',expected_imu='imu',
                        recovery_stable_time=.5,recovery_timeout=2.,recovery_max_correction=.75,
                        recovery_max_angle_deg=20.,recovery_sample_gap=.1,recovery_residual=.03,
                        recovery_max_source_gap=3.,recovery_gap_max_correction=2.5,
                        recovery_gap_max_angle_deg=90.,
                        tf_pose_slop=.15,tf_alignment_wait=1.,
                        t_body_imu=np.eye(4).ravel().tolist()).items(): self.declare_parameter(k,v)
        self.p = lambda k: self.get_parameter(k).value
        uid = int(self.p('uav_id'))
        self.odom_frame, self.body_frame = f'uav{uid}_odom', f'uav{uid}_base_link'
        self.extrinsic = transform(np.array(self.p('t_body_imu')).reshape(4,4))
        self.last_stamp = self.last_position = None
        self.last_quat = self.last_velocity = None
        self.recovery = self.new_recovery()
        self.reason = 'WAITING_FOR_DATA'
        self.reset_count, self.latched = 0, False
        self.last_good = -math.inf
        self.image_at = [-math.inf,-math.inf]
        self.px4_positions, self.px4_attitudes = deque(maxlen=100), deque(maxlen=100)
        self.tf_alignment = VioOdomAlignment()
        self.px4_reset = self.px4_attitude_reset = self.tf_reset_count = None
        self.first_px4_valid_stamp = None
        self.pub = self.create_publisher(VehicleOdometry,f'/{self.p("px4_ns")}/fmu/in/vehicle_visual_odometry',qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.tf_pub = TransformBroadcaster(self)
        self.health = self.create_publisher(String,'vio_health',1)
        self.diagnostics = self.create_publisher(String,'vio_diagnostics',1)
        self.create_subscription(Odometry,'odomimu',self.callback,qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition,
                                 f'/{self.p("px4_ns")}/fmu/out/vehicle_local_position',
                                 self.px4_position, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude,
                                 f'/{self.p("px4_ns")}/fmu/out/vehicle_attitude',
                                 self.px4_attitude, qos_profile_sensor_data)
        for i in range(2):
            self.create_subscription(Image,f'cam{i}/image_raw',lambda msg,index=i:self.image(msg,index),qos_profile_sensor_data)
        self.create_service(Trigger,'reset_vio_bridge',self.reset)
        self.create_timer(.1,self.watchdog)

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def new_recovery(self):
        return VioRecovery(*(float(self.p(k)) for k in (
            'recovery_stable_time','recovery_timeout','recovery_max_correction',
            'recovery_max_angle_deg','recovery_sample_gap','recovery_residual',
            'recovery_max_source_gap','recovery_gap_max_correction',
            'recovery_gap_max_angle_deg')))

    def image(self,msg,index):
        self.image_at[index]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def px4_position(self, msg):
        reset = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.px4_reset is not None and reset != self.px4_reset:
            self.tf_alignment = VioOdomAlignment()
            self.first_px4_valid_stamp = None
            self.px4_positions.clear()
            self.px4_attitudes.clear()
        self.px4_reset = reset
        if msg.xy_valid and msg.z_valid and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            if self.first_px4_valid_stamp is None:
                self.first_px4_valid_stamp = msg.timestamp * 1e-6
            self.px4_positions.append((msg.timestamp * 1e-6, (msg.x, msg.y, msg.z)))

    def px4_attitude(self, msg):
        if self.px4_attitude_reset is not None and msg.quat_reset_counter != self.px4_attitude_reset:
            self.tf_alignment = VioOdomAlignment()
            self.first_px4_valid_stamp = None
            self.px4_positions.clear()
            self.px4_attitudes.clear()
        self.px4_attitude_reset = msg.quat_reset_counter
        if all(math.isfinite(v) for v in msg.q) and .9 < math.sqrt(sum(v*v for v in msg.q)) < 1.1:
            self.px4_attitudes.append((msg.timestamp * 1e-6, tuple(msg.q)))

    def publish_vio_odom(self, msg, stamp, position, orientation, velocity, omega, pv, ov, vv):
        if self.tf_reset_count != self.reset_count:
            self.tf_alignment = VioOdomAlignment()
            self.tf_reset_count = self.reset_count
        vio_pose = vio_body_pose(position, orientation)
        if self.tf_alignment.rotation is None:
            if (not self.px4_positions or not self.px4_attitudes or
                    self.first_px4_valid_stamp is None or
                    stamp - self.first_px4_valid_stamp < float(self.p('tf_alignment_wait'))):
                return
            pt, px4_position = min(self.px4_positions, key=lambda item: abs(item[0] - stamp))
            at, px4_attitude = min(self.px4_attitudes, key=lambda item: abs(item[0] - stamp))
            if max(abs(pt - stamp), abs(at - stamp)) > float(self.p('tf_pose_slop')):
                return
            self.tf_alignment.latch(vio_pose, px4_body_pose(px4_position, px4_attitude))
            self.get_logger().info('Accepted OpenVINS pose aligned to PX4 NWU odom origin')
        point, quat = self.tf_alignment.transform(vio_pose)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.body_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, point)
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = map(float, quat)
        self.tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = tf.transform.translation.x
        odom.pose.pose.position.y = tf.transform.translation.y
        odom.pose.pose.position.z = tf.transform.translation.z
        odom.pose.pose.orientation = tf.transform.rotation
        for i, variance in enumerate((*pv, *ov)):
            odom.pose.covariance[i * 6 + i] = float(variance)
        body_velocity, body_omega = FLIP @ velocity, FLIP @ omega
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, body_omega)
        for i, variance in enumerate(vv):
            odom.twist.covariance[i * 6 + i] = float(variance)
        self.odom_pub.publish(odom)

    def reset(self,request,response):
        self.last_stamp = self.last_position = None
        self.last_quat = self.last_velocity = None
        self.recovery = self.new_recovery()
        self.reset_count = (self.reset_count+1)%256
        self.latched = False
        self.reason = 'MANUAL_RESET'
        self.last_good = -math.inf
        response.success, response.message = True,'Reset acknowledged; awaiting fresh VIO. Re-arm separately.'
        return response

    def watchdog(self):
        now = self.now()
        if self.recovery.expired(now):
            self.latched, self.reason = True, 'RECOVERY_TIMEOUT'
        if self.recovery.active and not all(0 <= now-t <= self.p('max_age') for t in self.image_at):
            self.recovery.previous = self.recovery.stable_since = None
        healthy = not self.latched and not self.recovery.active and all(0 <= now-t <= self.p('max_age') for t in [self.last_good]+self.image_at)
        self.health.publish(String(data='VALID' if healthy else 'INVALID'))
        self.diagnostics.publish(String(data=json.dumps(dict(
            state='LATCHED' if self.latched else ('RECOVERING' if self.recovery.active else ('VALID' if healthy else 'STALE')),
            reason=self.reason, reset_counter=self.reset_count,
            age=None if not math.isfinite(self.last_good) else round(now-self.last_good,4)))))

    def reject(self, reason):
        self.reason = reason
        if self.recovery.active:
            self.recovery.previous = self.recovery.stable_since = None

    def callback(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.now()
        if self.latched: return
        if not 0 <= now-stamp <= self.p('max_age'):
            self.reject('ODOMETRY_STALE'); return
        if not all(0<=now-t<=self.p('max_age') for t in self.image_at):
            self.reject('IMAGES_STALE'); return
        if msg.header.frame_id != self.p('expected_world') or msg.child_frame_id != self.p('expected_imu'):
            self.reject('FRAME_MISMATCH'); return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp: self.latched, self.reason = True, 'STAMP_REGRESSION'
            return
        p,q,v,w = msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        try:
            result = convert([p.x,p.y,p.z],[q.w,q.x,q.y,q.z],np.array([v.x,v.y,v.z]),np.array([w.x,w.y,w.z]),
                             msg.pose.covariance,msg.twist.covariance,self.extrinsic)
        except ValueError:
            self.reject('INVALID_COVARIANCE_OR_POSE'); return
        if not all(np.isfinite(x).all() for x in result):
            self.reject('NONFINITE_DATA'); return
        pos,quat,vel,omega,pv,ov,vv = result
        if np.linalg.norm(vel)>self.p('max_speed'):
            self.reject('EXCESSIVE_SPEED'); return
        if max(pv)>self.p('max_position_variance') or max(ov)>self.p('max_orientation_variance') or max(vv)>self.p('max_velocity_variance'):
            self.reject('EXCESSIVE_VARIANCE'); return
        if self.last_stamp is not None and not self.recovery.active:
            dt = stamp-self.last_stamp
            angle=2*math.acos(float(np.clip(abs(np.dot(quat,self.last_quat)),0.,1.)))
            if dt > 1. or np.linalg.norm(pos-self.last_position)>self.p('max_jump')+self.p('max_speed')*dt or angle>math.radians(self.p('recovery_max_angle_deg')):
                self.recovery.begin(now,self.last_stamp,self.last_position,self.last_quat,
                                    self.last_velocity,source_gap=dt>1.)
                self.reason = 'DATA_GAP' if dt>1. else ('ORIENTATION_DISCONTINUITY' if angle>math.radians(self.p('recovery_max_angle_deg')) else 'POSITION_DISCONTINUITY')
                self.health.publish(String(data='INVALID'))
                self.get_logger().warn(f'VIO quarantine: {self.reason}; dt={dt:.4f}s jump={np.linalg.norm(pos-self.last_position):.3f}m')
        if self.recovery.active:
            if not self.recovery.accept(now,stamp,pos,quat,vel): return
            # Explicitly inform EKF2 of the accepted discontinuity. Its reset
            # deltas must be handled by the controller; never conceal a shift.
            self.reset_count = (self.reset_count+1)%256
            self.recovery = self.new_recovery()
            self.reason = 'RECOVERED_WITH_RESET'
            self.get_logger().info('VIO stable again; publishing EV reset_counter')
        out = VehicleOdometry()
        # Hardware uses XRCE clock conversion. Algorithm SITL uses /clock on
        # both sides via the PX4-side RM27_SIM_CLOCK patch; do not offset twice.
        out.timestamp, out.timestamp_sample = int(now*1e6),int(stamp*1e6)
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out.position,out.q,out.velocity,out.angular_velocity = pos.tolist(),quat.tolist(),vel.tolist(),omega.tolist()
        out.position_variance,out.orientation_variance,out.velocity_variance = pv.tolist(),ov.tolist(),vv.tolist()
        out.reset_counter,out.quality = self.reset_count,100
        self.pub.publish(out)
        self.publish_vio_odom(msg,stamp,pos,quat,vel,omega,pv,ov,vv)
        self.last_stamp,self.last_position,self.last_good = stamp,pos,stamp
        self.last_quat,self.last_velocity = quat,vel


def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
