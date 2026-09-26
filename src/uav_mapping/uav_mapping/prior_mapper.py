"""Publish a PGM/YAML simulation prior and its map -> odom display transform."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from uav_mapping.rolling_grid import body_to_nwu
from uav_mapping.map_alignment import SpawnAlignment
import numpy as np
import yaml


SPAWN_ENU = ((1.3, 9.4), (-1.3, 9.4), (1.3, 11.6), (-1.3, 11.6))


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
        spawn_x, spawn_y = SPAWN_ENU[uid - 1]
        self.alignment = SpawnAlignment((spawn_x, spawn_y))
        self.yaw = self.position = None
        self.attitude_stamp = self.position_stamp = None
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
        self.tf_pub = TransformBroadcaster(self)
        self.odom_frame = f'uav{uid}_odom'
        self.create_subscription(VehicleAttitude, f'/px4_{uid}/fmu/out/vehicle_attitude',
                                 self.on_attitude, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, f'/px4_{uid}/fmu/out/vehicle_local_position',
                                 self.on_position, qos_profile_sensor_data)
        self.create_timer(1., self.publish_map)
        self.get_logger().info(f'Loaded field prior: {path} ({self.map.info.width}x{self.map.info.height})')

    def on_attitude(self, msg):
        if self.alignment.transform is not None: return
        try:
            rotation = body_to_nwu(msg.q)
        except ValueError:
            return
        self.yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        self.attitude_stamp = msg.timestamp
        self.maybe_anchor()

    def on_position(self, msg):
        if self.alignment.transform is not None: return
        if msg.xy_valid and all(math.isfinite(v) for v in (msg.x, msg.y)):
            self.position = (msg.x, -msg.y)
            self.position_stamp = msg.timestamp
            self.maybe_anchor()

    def maybe_anchor(self):
        if (self.position is not None and self.yaw is not None
                and self.position_stamp is not None and self.attitude_stamp is not None
                and abs(self.position_stamp - self.attitude_stamp) <= 150_000):
            self.alignment.latch(self.position, self.yaw)
            self.get_logger().info('Simulation map -> odom alignment fixed at initial PX4 pose')

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map)
        if self.alignment.transform is not None:
            x, y, yaw = self.alignment.transform
            tf = TransformStamped()
            tf.header.stamp = self.map.header.stamp
            tf.header.frame_id = self.map.header.frame_id
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = math.sin(yaw/2)
            tf.transform.rotation.w = math.cos(yaw/2)
            self.tf_pub.sendTransform(tf)


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
