"""OpenVINS 仿真测试链路：ros_gz_bridge（Gazebo Garden -> ROS2）+ ov_msckf。

两种模式（由 run_openvins_sim.sh 自动识别，也可手动指定）：

1) 全机 D435i（stereo_all:=true，对应 ALL_STEREO=1 启动的仿真，默认）：
   四机各挂一台 D435i，本 launch 桥接全部 4 机的传感器话题；
   OpenVINS 只给 vio_uavs 列表里的机号启动（4 个实例很吃 CPU，
   默认 vio_uavs:="1"，量力而开）。

   话题映射（N=1..4）：
     gz /uavN/vio_cam0|1/image  -> /uavN/cam0|1/image_raw  (mono8 30Hz，VIO 双目)
     gz /uavN/d435i/color/image_raw -> /uavN/d435i/color/image_raw (rgb8 30Hz，检测)
     gz /uavN/d435i/depth/image_raw -> /uavN/d435i/depth/image_raw (32FC1 米, 15Hz，避障)
     gz /uavN/d435i/imu           -> /uavN/d435i/imu  (相机内置 IMU 200Hz，备用)
     gz /world/<world>/model/x500_stereo_uavN_N/link/base_link/sensor/imu_sensor/imu
       -> /uavN/imu0  (机体 IMU 250Hz，OpenVINS 订阅，与 PX4 同源)

2) 单机模式（stereo_all:=false，对应 ALL_STEREO=0 VIO_UAV=N 启动的仿真）：
     gz .../model/<vio_model>/link/base_link/sensor/imu_sensor/imu -> /<uav_ns>/imu0
     gz /vio_cam0|1/image -> /<uav_ns>/cam0|1/image_raw
     gz /d435i/* -> /<uav_ns>/d435i/*（无前缀话题，全场只允许一架带相机）

OpenVINS 每机一份独立配置（/tmp/rm27_ov_uavN/，launch 时自动生成）：
kalibr 链中的 rostopic 从模板的 /uav1/... 改写为 /uavN/...，其余参数
（内外参、噪声、估计器）四机共享同一份模板。

OpenVINS 输出（namespace uavN 下）：
  odomimu / pathimu / points_msckf / trackhist（特征跟踪可视化图）

前置条件：
  1) 仿真已启动（默认 ALL_STEREO=1；或 ALL_STEREO=0 VIO_UAV=N 单机模式）
  2) 本进程环境与仿真同一 GZ_PARTITION（run_openvins_sim.sh 已自动处理）
  3) 已安装 ros_gz_bridge（Humble+Garden 用 OSRF 源的 ros-humble-ros-gzgarden）
  4) 已编译并 source OpenVINS（ov_msckf 包）

一般由 scripts/run_openvins_sim.sh 调用，也可手动：
  source /tmp/rm27_gz_env.sh
  ros2 launch uav_localization vio_sim_test.launch.py stereo_all:=true vio_uavs:="1 2"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_uav_config(template_cfg: str, uid: int) -> str:
    """按机号生成一份独立 OpenVINS 配置目录：kalibr 链中的 rostopic
    从模板的 /uav1/... 改写成 /uavN/...，其余内容原样复制。
    返回该机的 estimator_config.yaml 路径。"""
    src_dir = os.path.dirname(template_cfg)
    out_dir = f'/tmp/rm27_ov_uav{uid}'
    os.makedirs(out_dir, exist_ok=True)
    for name in ('estimator_config.yaml', 'kalibr_imu_chain.yaml',
                 'kalibr_imucam_chain.yaml'):
        with open(os.path.join(src_dir, name), encoding='utf-8') as f:
            text = f.read()
        if uid != 1:
            text = text.replace('/uav1/', f'/uav{uid}/')
        out = os.path.join(out_dir, name)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(text)
    return os.path.join(out_dir, 'estimator_config.yaml')


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('vio_model').perform(context)
    ns = LaunchConfiguration('uav_ns').perform(context)
    config = LaunchConfiguration('config').perform(context)
    with_ov = LaunchConfiguration('with_openvins').perform(context).lower() \
        in ('true', '1', 'yes')
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))
    stereo_all = LaunchConfiguration('stereo_all').perform(context).lower() \
        in ('true', '1', 'yes')
    vio_uavs = [s for s in LaunchConfiguration('vio_uavs').perform(context)
                .replace(',', ' ').split() if s]

    bridge_args = []
    remaps = []
    if stereo_all:
        # 全机 D435i：逐机桥接（gz 话题已由启动脚本按机号独立化）
        for i in range(1, num_uavs + 1):
            # PX4 standalone 生成的世界模型名 = 模型名_实例号
            wmodel = f'x500_stereo_uav{i}_{i}'
            imu_gz = (f'/world/{world}/model/{wmodel}'
                      f'/link/base_link/sensor/imu_sensor/imu')
            bridge_args += [
                f'{imu_gz}@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/uav{i}/vio_cam0/image@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/uav{i}/vio_cam1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/uav{i}/d435i/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/uav{i}/d435i/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/uav{i}/d435i/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ]
            remaps += [
                (imu_gz, f'/uav{i}/imu0'),
                (f'/uav{i}/vio_cam0/image', f'/uav{i}/cam0/image_raw'),
                (f'/uav{i}/vio_cam1/image', f'/uav{i}/cam1/image_raw'),
            ]
    else:
        # 单机模式：无前缀话题（与旧行为一致）
        imu_gz = (f'/world/{world}/model/{model}'
                  f'/link/base_link/sensor/imu_sensor/imu')
        bridge_args = [
            f'{imu_gz}@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/vio_cam0/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/vio_cam1/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # D435i 附加传感器：RGB（检测）、深度（避障）、相机内置 IMU（备用）
            '/d435i/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/d435i/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/d435i/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ]
        remaps = [
            (imu_gz, f'/{ns}/imu0'),
            ('/vio_cam0/image', f'/{ns}/cam0/image_raw'),
            ('/vio_cam1/image', f'/{ns}/cam1/image_raw'),
            ('/d435i/color/image_raw', f'/{ns}/d435i/color/image_raw'),
            ('/d435i/depth/image_raw', f'/{ns}/d435i/depth/image_raw'),
            ('/d435i/imu', f'/{ns}/d435i/imu'),
        ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vio_gz_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=remaps,
    )
    nodes = [bridge]

    if with_ov:
        if stereo_all:
            targets = [int(s) for s in vio_uavs] or [1]
        else:
            targets = [int(ns.replace('uav', ''))]
        for i in targets:
            cfg = _make_uav_config(config, i) if stereo_all else config
            nodes.append(Node(
                package='ov_msckf',
                executable='run_subscribe_msckf',
                name='openvins',
                namespace=f'uav{i}',
                output='screen',
                # 注意：不要在这里传 topic_imu / topic_camera0 / topic_camera1！
                # open_vins 上游 ROS2Visualizer::setup_subscribers 会自行
                # declare_parameter 这三个参数，而节点又开了
                # automatically_declare_parameters_from_overrides，
                # 从 params 文件传入会触发 ParameterAlreadyDeclaredException。
                # 实际话题由 kalibr_imu_chain / kalibr_imucam_chain 的
                # rostopic 字段决定（parse_external 覆盖），与上方桥接
                # remappings 严格对应；全机模式由 _make_uav_config 按机号改写。
                parameters=[{
                    'config_path': cfg,
                    'publish_global_to_imu_tf': True,
                    'publish_calibration_tf': True,
                }],
            ))
    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('uav_localization'),
        'config', 'openvins_sim', 'estimator_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rmuc_2025_field',
                              description='Gazebo 世界名'),
        DeclareLaunchArgument('stereo_all', default_value='false',
                              description='true=全机 D435i（ALL_STEREO=1 启动的仿真）'),
        DeclareLaunchArgument('num_uavs', default_value='4',
                              description='全机模式的无人机数量'),
        DeclareLaunchArgument('vio_uavs', default_value='1',
                              description='全机模式下哪些机跑 OpenVINS，如 "1" 或 "1 3"'),
        DeclareLaunchArgument('vio_model', default_value='x500_stereo_1',
                              description='单机模式：世界中带双目的模型实例名'),
        DeclareLaunchArgument('uav_ns', default_value='uav1',
                              description='单机模式：ROS 侧话题命名空间'),
        DeclareLaunchArgument('config', default_value=default_config,
                              description='OpenVINS estimator_config.yaml 路径'),
        DeclareLaunchArgument('with_openvins', default_value='true',
                              description='false 时只起桥接（用于先验证数据通路）'),
        OpaqueFunction(function=_setup),
    ])
