"""Sole, piecewise-static map -> odom authority with manual RViz alignment."""
import math

from geometry_msgs.msg import Pose2D, TransformStamped
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

from uav_mapping.map_alignment import SPAWN_ENU, SpawnAlignment


class MapOdom(Node):
    def __init__(self):
        super().__init__('map_odom')
        self.declare_parameter('uav_id', 1)
        self.declare_parameter('sim', True)
        uid = int(self.get_parameter('uav_id').value)
        if not 1 <= uid <= len(SPAWN_ENU):
            raise ValueError('uav_id must be 1..4')
        self.sim = bool(self.get_parameter('sim').value)
        self.map_frame = f'uav{uid}_map'
        self.odom_frame = f'uav{uid}_odom'
        self.alignment = SpawnAlignment(SPAWN_ENU[uid - 1])
        self.tf_pub = StaticTransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.current_pub = self.create_publisher(Pose2D, 'map_odom/current', qos)
        self.create_subscription(Pose2D, 'map_odom/set', self.set_alignment, 10)
        if self.sim:
            self.create_subscription(VehicleLocalPosition,
                                     f'/px4_{uid}/fmu/out/vehicle_local_position',
                                     self.on_position, qos_profile_sensor_data)
        else:
            # No measured global anchor exists on hardware yet.
            self.alignment.transform = (0., 0., 0.)
            self.publish_alignment()

    def on_position(self, msg):
        if self.alignment.transform is not None:
            return
        if msg.xy_valid and all(math.isfinite(v) for v in (msg.x, msg.y)):
            self.alignment.latch((msg.x, -msg.y))
            self.publish_alignment()
            self.get_logger().info('Initial map -> odom alignment fixed at simulation spawn')

    def set_alignment(self, msg):
        values = (float(msg.x), float(msg.y), float(msg.theta))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Rejected nonfinite map -> odom alignment')
            return
        self.alignment.transform = values
        self.publish_alignment()
        self.get_logger().info(f'Manual map -> odom alignment: x={values[0]:.3f}, '
                               f'y={values[1]:.3f}, yaw={values[2]:.3f} rad')

    def publish_alignment(self):
        x, y, yaw = self.alignment.transform
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)
        self.current_pub.publish(Pose2D(x=x, y=y, theta=yaw))


def main(args=None):
    rclpy.init(args=args)
    node = MapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
