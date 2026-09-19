"""Software depth from synchronized raw IR images; shared sim/real path."""
import cv2
import copy
import numpy as np
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from uav_perception.stereo_matcher import StereoMatcher


class SoftwareStereo(Node):
    def __init__(self):
        super().__init__('software_stereo')
        for k,v in dict(config='',t_body_imu=np.eye(4).ravel().tolist(),rate=5.,sync_slop=.003).items(): self.declare_parameter(k,v)
        p = lambda k:self.get_parameter(k).value
        self.matcher = StereoMatcher(p('config'),np.array(p('t_body_imu')).reshape(4,4))
        cv2.setNumThreads(2)
        self.period,self.last = 1/p('rate'),-float('inf')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image,'d435i/depth/image_raw',1)
        self.info = self.create_publisher(CameraInfo,'d435i/depth/camera_info',1)
        self.subs = [message_filters.Subscriber(self,Image,f'cam{i}/image_raw',qos_profile=qos_profile_sensor_data) for i in range(2)]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs,5,p('sync_slop'))
        self.sync.registerCallback(self.callback)

    def callback(self,left,right):
        stamp = left.header.stamp.sec+left.header.stamp.nanosec*1e-9
        if stamp<self.last: self.last=-float('inf')
        if stamp-self.last < self.period: return
        self.last=stamp
        a,b = [self.bridge.imgmsg_to_cv2(m,'mono8') for m in (left,right)]
        depth = self.matcher.depth(a,b)
        header = copy.deepcopy(left.header if self.matcher.order[0]=='cam0' else right.header)
        header.frame_id=self.get_namespace().strip('/')+'_stereo_depth_optical'
        out = self.bridge.cv2_to_imgmsg(depth,'32FC1')
        out.header=header
        self.pub.publish(out)
        info = CameraInfo()
        info.header=header
        info.width,info.height = self.matcher.size
        info.p=self.matcher.p.ravel().tolist()
        info.k=self.matcher.p[:3,:3].ravel().tolist()
        info.r=np.eye(3).ravel().tolist()
        info.distortion_model='plumb_bob'
        info.d=[0.]*5
        self.info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node=SoftwareStereo()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
