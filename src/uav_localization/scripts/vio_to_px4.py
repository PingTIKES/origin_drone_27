#!/usr/bin/env python3
"""Guarded OpenVINS odomimu -> PX4 external vision. No ground-truth input."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleOdometry
from uav_localization.vio_geometry import convert
from uav_localization.calibration import transform


class VioBridge(Node):
    def __init__(self):
        super().__init__('vio_to_px4')
        for k,v in dict(px4_ns='px4_1',max_age=.25,max_position_variance=1.,
                        max_orientation_variance=.25,max_velocity_variance=1.,
                        max_speed=4.,max_jump=.4,expected_world='global',expected_imu='imu',
                        t_body_imu=np.eye(4).ravel().tolist()).items(): self.declare_parameter(k,v)
        self.p = lambda k: self.get_parameter(k).value
        self.extrinsic = transform(np.array(self.p('t_body_imu')).reshape(4,4))
        self.last_stamp = self.last_position = None
        self.reset_count, self.latched = 0, False
        self.last_good = -math.inf
        self.image_at = [-math.inf,-math.inf]
        self.pub = self.create_publisher(VehicleOdometry,f'/{self.p("px4_ns")}/fmu/in/vehicle_visual_odometry',qos_profile_sensor_data)
        self.health = self.create_publisher(String,'vio_health',1)
        self.create_subscription(Odometry,'odomimu',self.callback,qos_profile_sensor_data)
        for i in range(2):
            self.create_subscription(Image,f'cam{i}/image_raw',lambda msg,index=i:self.image(msg,index),qos_profile_sensor_data)
        self.create_service(Trigger,'reset_vio_bridge',self.reset)
        self.create_timer(.1,self.watchdog)

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def image(self,msg,index):
        self.image_at[index]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def reset(self,request,response):
        self.last_stamp = self.last_position = None
        self.reset_count = (self.reset_count+1)%256
        self.latched = False
        self.last_good = -math.inf
        response.success, response.message = True,'Reset acknowledged; awaiting fresh VIO. Re-arm separately.'
        return response

    def watchdog(self):
        healthy = not self.latched and all(0 <= self.now()-t <= self.p('max_age') for t in [self.last_good]+self.image_at)
        self.health.publish(String(data='VALID' if healthy else 'INVALID'))

    def callback(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.now()
        if self.latched or not 0 <= now-stamp <= self.p('max_age'): return
        if not all(0<=now-t<=self.p('max_age') for t in self.image_at):return
        if msg.header.frame_id != self.p('expected_world') or msg.child_frame_id != self.p('expected_imu'): return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp: self.latched = True
            return
        p,q,v,w = msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        try:
            result = convert([p.x,p.y,p.z],[q.w,q.x,q.y,q.z],np.array([v.x,v.y,v.z]),np.array([w.x,w.y,w.z]),
                             msg.pose.covariance,msg.twist.covariance,self.extrinsic)
        except ValueError: return
        if not all(np.isfinite(x).all() for x in result): return
        pos,quat,vel,omega,pv,ov,vv = result
        if np.linalg.norm(vel)>self.p('max_speed'):return
        if max(pv)>self.p('max_position_variance') or max(ov)>self.p('max_orientation_variance') or max(vv)>self.p('max_velocity_variance'): return
        if self.last_stamp is not None:
            dt = stamp-self.last_stamp
            if dt > 1. or np.linalg.norm(pos-self.last_position)>self.p('max_jump')+self.p('max_speed')*dt:
                self.latched = True
                self.get_logger().error('VIO discontinuity; bridge latched until reset_vio_bridge')
                return
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
        self.last_stamp,self.last_position,self.last_good = stamp,pos,stamp


def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
