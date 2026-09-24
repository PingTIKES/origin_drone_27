"""Depth cloud + timestamped PX4 pose -> rolling local NWU occupancy grid."""
from collections import deque
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from uav_mapping.rolling_grid import (RollingGrid, body_to_nwu,
                                      body_quaternion_to_nwu, ray_cells, slab_endpoint)


class RollingMapper(Node):
    def __init__(self):
        super().__init__('rolling_mapper')
        defaults = dict(uav_id=1, px4_ns='px4_1', size=12., resolution=.1,
                        memory=8., inflation=.35, half_height=.25,
                        pose_slop=.15, max_rays=1800, max_cloud_age=.4,
                        cam_xyz=[.17, .025, -.06])
        for key, val in defaults.items(): self.declare_parameter(key, val)
        p = lambda key: self.get_parameter(key).value
        self.frame = f'uav{p("uav_id")}_local_nwu'
        self.body_frame = f'uav{p("uav_id")}'
        self.grid = RollingGrid(p('size'), p('resolution'), p('memory'), p('inflation'))
        self.height, self.slop = p('half_height'), p('pose_slop')
        self.max_age, self.max_rays = p('max_cloud_age'), p('max_rays')
        self.camera = np.array(p('cam_xyz'))
        self.positions, self.attitudes = deque(maxlen=100), deque(maxlen=100)
        self.reset_id = self.att_reset = self.altitude = None
        self.last_cloud = -float('inf')
        self.create_subscription(VehicleLocalPosition,
                                 f'/{p("px4_ns")}/fmu/out/vehicle_local_position', self.position, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude,
                                 f'/{p("px4_ns")}/fmu/out/vehicle_attitude', self.attitude, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'obstacles', self.cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid, 'local_map', 1)
        self.raw_pub = self.create_publisher(OccupancyGrid, 'local_map_raw', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = TransformBroadcaster(self)

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def position(self, msg):
        reset = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        valid = msg.xy_valid and msg.z_valid and np.all(np.isfinite([msg.x, msg.y, msg.z]))
        if not valid or reset != self.reset_id:
            self.grid.clear()
            self.positions.clear()
            self.reset_id = reset
        if valid:
            self.positions.append((msg.timestamp*1e-6, np.array([msg.x, -msg.y, -msg.z])))
            if self.attitudes:
                at, rotation, quat = min(self.attitudes, key=lambda a: abs(a[0] - msg.timestamp*1e-6))
                if abs(at - msg.timestamp*1e-6) <= self.slop:
                    tf = TransformStamped()
                    tf.header.stamp = self.get_clock().now().to_msg()
                    tf.header.frame_id = self.frame
                    tf.child_frame_id = self.body_frame
                    tf.transform.translation.x = float(msg.x)
                    tf.transform.translation.y = float(-msg.y)
                    tf.transform.translation.z = float(-msg.z)
                    tf.transform.rotation.w = float(quat[0])
                    tf.transform.rotation.x = float(quat[1])
                    tf.transform.rotation.y = float(quat[2])
                    tf.transform.rotation.z = float(quat[3])
                    self.tf_pub.sendTransform(tf)
                    odom = Odometry()
                    odom.header = tf.header
                    odom.child_frame_id = self.body_frame
                    odom.pose.pose.position.x = tf.transform.translation.x
                    odom.pose.pose.position.y = tf.transform.translation.y
                    odom.pose.pose.position.z = tf.transform.translation.z
                    odom.pose.pose.orientation = tf.transform.rotation
                    if msg.v_xy_valid and msg.v_z_valid and np.all(np.isfinite([msg.vx, msg.vy, msg.vz])):
                        velocity_body = rotation.T @ np.array([msg.vx, -msg.vy, -msg.vz])
                        odom.twist.twist.linear.x = float(velocity_body[0])
                        odom.twist.twist.linear.y = float(velocity_body[1])
                        odom.twist.twist.linear.z = float(velocity_body[2])
                    self.odom_pub.publish(odom)

    def attitude(self, msg):
        try:
            rotation = body_to_nwu(msg.q)
            quat = body_quaternion_to_nwu(msg.q)
        except ValueError:
            self.attitudes.clear()
            self.grid.clear()
            return
        if msg.quat_reset_counter != self.att_reset:
            self.grid.clear()
            self.attitudes.clear()
            self.att_reset = msg.quat_reset_counter
        self.attitudes.append((msg.timestamp*1e-6, rotation, quat))

    def cloud(self, msg):
        now = self.now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id != self.body_frame or not 0 <= now - stamp <= self.max_age:
            return
        if not self.positions or not self.attitudes: return
        pt, pos = min(self.positions, key=lambda p: abs(p[0] - stamp))
        at, rotation, _ = min(self.attitudes, key=lambda p: abs(p[0] - stamp))
        if max(abs(pt - stamp), abs(at - stamp)) > self.slop: return
        if stamp <= self.last_cloud: return
        self.last_cloud = stamp
        if self.altitude is None or abs(pos[2] - self.altitude) > min(.1,self.height / 2):
            self.grid.clear()  # never reuse a map from a different flight layer
            self.altitude = pos[2]
        self.grid.recenter(pos[:2])
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.asarray(points).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points): return  # no return does not certify free space
        # Voxel/nearest endpoint reduction avoids dropping a closer hit for a farther hit.
        # Bound work by rejecting overloaded scans rather than declaring sampled-away objects free.
        world = points @ rotation.T + pos
        sensor = rotation @ self.camera + pos
        selected={}
        for point in world:
            clipped,hit=slab_endpoint(sensor,point,self.altitude,self.height)
            if clipped is None:continue
            key=(*self.grid.cell(clipped[:2]),hit)
            selected.setdefault(key,point)
        world=np.array(list(selected.values()))
        if not len(world):return
        if len(world) > self.max_rays:
            self.get_logger().warn('Depth scan exceeds max_rays; map update rejected')
            return
        self.grid.update(sensor, world, now, self.altitude, self.height)
        # Only the mounting segment inside the vehicle is assumed free, not a blind disk.
        for x, y in ray_cells(self.grid.cell(pos[:2]), self.grid.cell(sensor[:2])):
            if self.grid.inside((x, y)) and self.grid.odds[y, x] <= 0:
                self.grid.odds[y, x], self.grid.seen[y, x] = -.7, now
        out = OccupancyGrid()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.info.resolution = self.grid.res
        out.info.width = out.info.height = self.grid.n
        out.info.origin.position.x, out.info.origin.position.y = (self.grid.origin * self.grid.res).tolist()
        out.info.origin.position.z = float(self.altitude)
        out.info.origin.orientation.w = 1.
        out.data = self.grid.occupancy(now).ravel().tolist()
        self.pub.publish(out)
        raw = OccupancyGrid()
        raw.header = out.header
        raw.info = out.info
        raw.data = self.grid.occupancy(now, inflate=False).ravel().tolist()
        self.raw_pub.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = RollingMapper()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
