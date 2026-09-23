"""Publish the RMUC simulation field prior for RViz, never for control."""
import base64
import json
import math
from pathlib import Path
import zlib

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from uav_mapping.rolling_grid import body_to_nwu


SPAWN_ENU = ((1.3, 9.4), (-1.3, 9.4), (1.3, 11.6), (-1.3, 11.6))


def load_prior(path):
    prior = json.loads(Path(path).read_text(encoding='utf-8'))
    data = zlib.decompress(base64.b64decode(prior['data_zlib_base64']))
    if len(data) != prior['width'] * prior['height']:
        raise ValueError('prior map dimensions do not match data')
    return prior, [v if v < 128 else v - 256 for v in data]


class PriorMapper(Node):
    def __init__(self):
        super().__init__('prior_mapper')
        self.declare_parameter('uav_id', 1)
        uid = int(self.get_parameter('uav_id').value)
        if not 1 <= uid <= len(SPAWN_ENU):
            raise ValueError('uav_id must be 1..4')
        path = Path(get_package_share_directory('uav_mapping')) / 'config/rmuc_2025_prior.json'
        prior, values = load_prior(path)
        spawn_x, spawn_y = SPAWN_ENU[uid - 1]
        self.spawn = (spawn_x, spawn_y)
        self.world_origin = tuple(prior['origin'])
        self.yaw = 0.
        self.position = (0., 0.)
        self.valid_since = None
        self.frozen = False
        self.map = OccupancyGrid()
        self.map.header.frame_id = f'uav{uid}_local_nwu'
        self.map.info.width = prior['width']
        self.map.info.height = prior['height']
        self.map.info.resolution = prior['resolution']
        self.align_map()
        self.map.data = values
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, 'global_map', qos)
        self.create_subscription(VehicleAttitude, f'/px4_{uid}/fmu/out/vehicle_attitude',
                                 self.on_attitude, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, f'/px4_{uid}/fmu/out/vehicle_local_position',
                                 self.on_position, qos_profile_sensor_data)
        self.create_subscription(String, 'vio_health', self.on_health, 1)
        self.create_timer(1., self.publish_map)
        self.get_logger().info(f'Loaded field prior: {path} ({self.map.info.width}x{self.map.info.height})')

    def align_map(self):
        # World ENU -> PX4 local NWU. The simulated x500 starts at world yaw 0,
        # so its estimated body yaw gives the local/world yaw offset. This only
        # locates a display map; the planner never consumes it.
        dx = self.world_origin[0] - self.spawn[0]
        dy = self.world_origin[1] - self.spawn[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.map.info.origin.position.x = self.position[0] + c*dx - s*dy
        self.map.info.origin.position.y = self.position[1] + s*dx + c*dy
        self.map.info.origin.orientation.z = math.sin(self.yaw/2)
        self.map.info.origin.orientation.w = math.cos(self.yaw/2)

    def on_attitude(self, msg):
        if self.frozen: return
        try:
            rotation = body_to_nwu(msg.q)
        except ValueError:
            return
        self.yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        self.align_map()

    def on_position(self, msg):
        if self.frozen: return
        if msg.xy_valid and all(math.isfinite(v) for v in (msg.x, msg.y)):
            self.position = (msg.x, -msg.y)
            self.align_map()

    def on_health(self, msg):
        if self.frozen: return
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg.data != 'VALID':
            self.valid_since = None
        elif self.valid_since is None:
            self.valid_since = now
        elif now - self.valid_since >= 2.:
            self.frozen = True
            self.get_logger().info('Field prior aligned to initial PX4 local heading')

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    node = PriorMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
