"""Publish the RMUC simulation field prior for RViz, never for control."""
import base64
import json
from pathlib import Path
import zlib

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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
        self.map = OccupancyGrid()
        self.map.header.frame_id = f'uav{uid}_local_nwu'
        self.map.info.width = prior['width']
        self.map.info.height = prior['height']
        self.map.info.resolution = prior['resolution']
        self.map.info.origin.position.x = prior['origin'][0] - spawn_y
        self.map.info.origin.position.y = prior['origin'][1] + spawn_x
        self.map.info.origin.orientation.w = 1.
        self.map.data = values
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, 'global_map', qos)
        self.create_timer(1., self.publish_map)
        self.get_logger().info(f'Loaded field prior: {path} ({self.map.info.width}x{self.map.info.height})')

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
