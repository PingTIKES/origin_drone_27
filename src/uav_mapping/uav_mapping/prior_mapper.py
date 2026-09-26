"""Publish the prebuilt simulation PGM/YAML prior map."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_mapping.map_alignment import SPAWN_ENU
import numpy as np
import yaml


def load_prior(path):
    path = Path(path)
    prior = yaml.safe_load(path.read_text(encoding='utf-8'))
    if prior.get('mode') != 'trinary' or prior.get('negate') != 0:
        raise ValueError('prior requires trinary mode and negate=0')
    image_path = path.parent / prior['image']
    pixels = np.asarray(Image.open(image_path).convert('L'))
    # Map YAML/PGM top row is the largest world y; OccupancyGrid begins at the bottom.
    grid = np.flipud(pixels)
    values = np.full(grid.shape, -1, dtype=np.int8)
    values[grid == 0] = 100
    values[grid == 254] = 0
    if not np.all(np.isin(grid, (0, 205, 254))):
        raise ValueError('unexpected PGM pixel value')
    prior['width'], prior['height'] = grid.shape[1], grid.shape[0]
    return prior, values.ravel().tolist()


class PriorMapper(Node):
    def __init__(self):
        super().__init__('prior_mapper')
        self.declare_parameter('uav_id', 1)
        uid = int(self.get_parameter('uav_id').value)
        if not 1 <= uid <= len(SPAWN_ENU):
            raise ValueError('uav_id must be 1..4')
        path = Path(get_package_share_directory('uav_mapping')) / 'config/rmuc_2025_prior.yaml'
        prior, values = load_prior(path)
        self.map = OccupancyGrid()
        self.map.header.frame_id = f'uav{uid}_map'
        self.map.info.width = prior['width']
        self.map.info.height = prior['height']
        self.map.info.resolution = prior['resolution']
        self.map.info.origin.position.x = float(prior['origin'][0])
        self.map.info.origin.position.y = float(prior['origin'][1])
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
