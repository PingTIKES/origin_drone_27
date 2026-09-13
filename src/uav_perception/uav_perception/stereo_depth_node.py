"""
D435i 深度图 → 机体坐标系障碍点云（即框架文档里的 stereo_depth_node 角色）。

订阅  d435i/depth/image_raw（→/uavN/d435i/depth/image_raw）
      sensor_msgs/Image，32FC1 浮点深度，单位米
发布  obstacles（→/uavN/obstacles）sensor_msgs/PointCloud2
      坐标系 = 机体 FLU（x 前、y 左、z 上），frame_id = uavN，
      与 pose_tf_publisher 的 map->uavN TF 配合可直接在 RViz 显示

仿真/实机同一份代码：
  仿真：start_sim_4uav.sh 的 D435i 模型深度相机（640x480@15，hfov 1.5184 rad）
  实机：realsense2_camera 的 /camera/camera/depth/image_rect_raw，
        launch 里 remap 进来即可；内参变了就改 fx/fy/cx/cy 参数
        （或后续升级为订阅 camera_info 自动读取）

处理链：跳帧（frame_decimation）→ 像素抽稀（step）→ 反投影 →
        光学系转机体 FLU（含安装外参平移）→ 量程过滤 → 发布。
默认 320x240 @ 5 Hz（step=2、15Hz 跳 2/3 帧），与 vfh_planner 注释中的
"320x240@5Hz" 输入假设一致。
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        # ---- 参数 ----
        self.declare_parameter('uav_id', 1)        # frame_id = uavN
        # 深度相机内参（默认 = worlds/models/d435i/model.sdf 的 depth 传感器：
        # 640x480, hfov 1.5184 rad → fx=fy≈337.2, 主点取中心）
        self.declare_parameter('fx', 337.2)
        self.declare_parameter('fy', 337.2)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)
        # 安装外参平移（机体 FLU 系）：D435i 挂点 base_link 系 (0.17,0,-0.06) NED
        # + 深度镜头在 d435i 内的 (0,0.025,0) NED → FLU (0.17, -0.025, 0.06)
        self.declare_parameter('cam_xyz', [0.17, -0.025, 0.06])
        self.declare_parameter('step', 2)              # 像素抽稀步长（2→320x240）
        self.declare_parameter('frame_decimation', 3)  # 每 k 帧处理 1 帧（15→5Hz）
        self.declare_parameter('min_range', 0.3)       # 盲区/噪声截断 m
        self.declare_parameter('max_range', 8.0)       # 远界 m

        uid = int(self.get_parameter('uav_id').value)
        self.frame_id = f'uav{uid}'
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.cam_xyz = [float(v) for v in self.get_parameter('cam_xyz').value]
        self.step = int(self.get_parameter('step').value)
        self.decim = int(self.get_parameter('frame_decimation').value)
        self.min_r = float(self.get_parameter('min_range').value)
        self.max_r = float(self.get_parameter('max_range').value)

        self.create_subscription(Image, 'd435i/depth/image_raw', self._cb_depth, 5)
        self.pub = self.create_publisher(PointCloud2, 'obstacles', 5)

        self._count = 0
        self._grid_cache = {}  # (h, w) -> (v, u) 抽稀后的像素网格
        self.get_logger().info(
            f'深度转点云就绪：订阅 ~d435i/depth/image_raw，发布 ~obstacles '
            f'(frame={self.frame_id}，step={self.step}，'
            f'每 {self.decim} 帧处理 1 帧)')

    def _cb_depth(self, msg: Image):
        self._count += 1
        if self._count % self.decim != 0:
            return
        if msg.encoding not in ('32FC1',):
            self.get_logger().warn(f'不支持的深度图编码 {msg.encoding}，需要 32FC1')
            return

        h, w, s = msg.height, msg.width, self.step
        if (h, w) not in self._grid_cache:
            v, u = np.mgrid[0:h:s, 0:w:s]
            self._grid_cache[(h, w)] = (v.astype(np.float32),
                                        u.astype(np.float32))
        v, u = self._grid_cache[(h, w)]

        z = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)[::s, ::s]
        valid = np.isfinite(z) & (z > self.min_r) & (z < self.max_r)
        if not np.any(valid):
            return
        z = z[valid]
        u = u[valid]
        v = v[valid]

        # 光学系反投影（z 前、x 右、y 下）
        x_o = (u - self.cx) * z / self.fx
        y_o = (v - self.cy) * z / self.fy
        # 光学系 → 机体 FLU（x 前、y 左、z 上）+ 安装平移
        x_b = z + self.cam_xyz[0]
        y_b = -x_o + self.cam_xyz[1]
        z_b = -y_o + self.cam_xyz[2]

        cloud = np.stack([x_b, y_b, z_b], axis=1)
        out = point_cloud2.create_cloud_xyz32(msg.header, cloud)
        out.header.frame_id = self.frame_id
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
