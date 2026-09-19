"""Persistent user goal in this UAV's local NWU frame -> local NED request."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped


class LocalGoal(Node):
    def __init__(self):
        super().__init__('local_goal')
        self.declare_parameter('uav_id',1)
        self.declare_parameter('cruise_alt',2.)
        self.frame=f'uav{self.get_parameter("uav_id").value}_local_nwu'
        self.alt=float(self.get_parameter('cruise_alt').value)
        self.goal=None
        self.pub=self.create_publisher(Point,'waypoint_in',1)
        self.create_subscription(PoseStamped,'goal_pose',self.callback,1)
        self.create_timer(.2,self.tick)

    def callback(self,msg):
        p=msg.pose.position
        if msg.header.frame_id!=self.frame or not all(math.isfinite(v) for v in (p.x,p.y)):return
        self.goal=Point(x=float(p.x),y=float(-p.y),z=-self.alt)

    def tick(self):
        if self.goal is not None:self.pub.publish(self.goal)


def main(args=None):
    rclpy.init(args=args)
    node=LocalGoal()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
