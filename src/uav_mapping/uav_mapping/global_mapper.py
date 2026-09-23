"""Accumulate observed rolling-map cells for RViz; never used for control."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition
from uav_mapping.global_grid import GlobalGrid


class GlobalMapper(Node):
    def __init__(self):
        super().__init__('global_mapper')
        self.declare_parameter('uav_id', 1)
        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('size', 60.)
        self.declare_parameter('resolution', .1)
        uid = int(self.get_parameter('uav_id').value)
        self.frame = f'uav{uid}_local_nwu'
        self.grid = GlobalGrid(float(self.get_parameter('size').value),
                               float(self.get_parameter('resolution').value))
        self.altitude = None
        self.reset_id = None
        self.has_data = False
        self.create_subscription(OccupancyGrid, 'local_map', self.on_map, 1)
        px4_ns = self.get_parameter('px4_ns').value
        self.create_subscription(VehicleLocalPosition,
                                 f'/{px4_ns}/fmu/out/vehicle_local_position',
                                 self.on_position, qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid, 'global_map', 1)
        self.create_timer(1., self.publish_map)

    def on_position(self, msg):
        reset = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.reset_id is not None and reset != self.reset_id:
            self.grid.clear()
            self.has_data = False
        self.reset_id = reset

    def on_map(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg.header.frame_id != self.frame or not 0 <= now-stamp <= 1.:
            return
        if msg.info.width * msg.info.height != len(msg.data):
            return
        if not math.isfinite(msg.info.origin.position.z):
            return
        altitude = msg.info.origin.position.z
        if self.altitude is not None and abs(altitude - self.altitude) > .25:
            self.grid.clear()
            self.has_data = False
        self.altitude = altitude
        cells = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        try:
            self.has_data |= self.grid.update(cells, msg.info.origin.position.x,
                                              msg.info.origin.position.y, msg.info.resolution)
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def publish_map(self):
        if not self.has_data:
            return
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.info.resolution = self.grid.res
        out.info.width = out.info.height = self.grid.n
        out.info.origin.position.x = self.grid.origin
        out.info.origin.position.y = self.grid.origin
        out.info.origin.position.z = self.altitude
        out.info.origin.orientation.w = 1.
        out.data = self.grid.data.ravel().tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
