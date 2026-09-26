"""Accept RViz map/odom goals and publish PX4 local NED requests."""
import math
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from tf2_ros import Buffer, TransformException, TransformListener


class LocalGoal(Node):
    def __init__(self):
        super().__init__('local_goal')
        self.declare_parameter('uav_id',1)
        self.declare_parameter('cruise_alt',2.)
        uid=self.get_parameter('uav_id').value
        self.frame=f'uav{uid}_odom'
        self.map_frame=f'uav{uid}_map'
        self.alt=float(self.get_parameter('cruise_alt').value)
        self.goal=None
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.pub=self.create_publisher(Point,'waypoint_in',1)
        self.create_subscription(PoseStamped,'goal_pose',self.callback,1)
        self.create_timer(.2,self.tick)

    def callback(self,msg):
        p=msg.pose.position
        if msg.header.frame_id not in (self.frame,self.map_frame) or not all(
                math.isfinite(v) for v in (p.x,p.y)):
            return
        self.goal=msg

    def tick(self):
        if self.goal is None:return
        p=self.goal.pose.position
        x,y=float(p.x),float(p.y)
        if self.goal.header.frame_id==self.map_frame:
            try:
                tf=self.tf_buffer.lookup_transform(self.frame,self.map_frame,Time())
            except TransformException:
                return
            q=tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            c,s=math.cos(yaw),math.sin(yaw)
            x,y=tf.transform.translation.x+c*x-s*y,tf.transform.translation.y+s*x+c*y
        self.pub.publish(Point(x=x,y=-y,z=-self.alt))


def main(args=None):
    rclpy.init(args=args)
    node=LocalGoal()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
