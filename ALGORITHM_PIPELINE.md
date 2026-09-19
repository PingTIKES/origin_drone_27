# 无雷达视觉算法闭环：仿真与真机共用

硬件约束：整机 <249g，RK3576 调试 / RK3566 最终平台；用户实际相机为
D430i + 独立 RGB，使用相机内置 IMU，PX4 1.14.2。是否有 D4 尚未确认。
代码改动不增加传感器，不代表已经称重达标或证明板端实时性能。

## 已实现的数据流

```text
传感器来源（二选一）
  Gazebo 图像 + 相机 IMU（不使用真值定位/理想深度/真值目标）
  真机相机驱动 + 相机 IMU（同样的规范话题与采集时间戳）
                     │
       双目 + IMU → OpenVINS → vio_to_px4 → PX4 EKF2
                     │                    │
       双目 → 整流 + SGBM → 深度         估计位姿/全姿态
       或设备硬件深度 → 深度格式适配       │
                     └→ 机体系点云 → 滚动局部栅格
                                           │
 用户目标/外部任务 → waypoint_in → local_navigator → waypoint → Offboard
                         ↑         A*主规划 / 有条件VFH / 悬停
  collision_monitor → safety_waypoint（优先请求，也必须通过障碍检查）
```

新 `algorithm.launch.py` 不启动旧 sim_target_detector，不读取 field_map/npz，
也不把 Gazebo pose 或 GPS 直接接入定位。PX4 使用惯性融合控制，这是估计器的一部分。
独立 RGB 暂不参与这条定位/避障链；旧 YOLO 文件仍是桩，未提供训练模型时不伪造搜敌结果。
旧集群搜索/汇聚状态机保留作历史对照，新闭环先支持单机目标导航与显式多机防碰。

## 局部地图和规划行为

- 固定朝向 local NWU，12×12m、0.1m 分辨率，窗口随位置平移；不是每帧随机体旋转。
- 以采集时间戳匹配 PX4 估计位置和四元数（含 roll/pitch），容差 0.15s。
- 对数占据证据：射线空闲/终点占据，同帧每格只投票一次，命中优先；保存 8s，过期变未知。
- 飞行高度带 ±0.25m，障碍膨胀 0.35m。楼板/地面射线裁剪到高度带，只能发现空闲，
  不擦除已有墙体。高度层变化清图。不对整个视场或无深度像素宣称空闲。
- A* 不穿未知、不斜切障碍角、不把起点吸附到墙另一边；复用 FieldMap 的拉直接口，
  用更保守的 supercover 检查路径。目标在窗口外时只执行可达已知空间中的部分路径。
- A* 搜索预算 25ms；仅预算耗尽且地图新鲜时，72 扇区 VFH 从已验证通路中选短距离动作。
  无路、地图断流、定位失效不降级为盲飞。VFH 不另起节点争抢 waypoint。
- 每次位置目标最大前移 0.4m；失去上游目标后固定位置悬停。PX4 速度上限另设 0.5m/s；
  短航点本身不是速度限制。导航测得速度超过 0.6m/s 会进入保持。
- 不支持沿未观测高度上下绕障。前视相机遇未知方向可以原地转向观察；局部图不保证
  在视场外或超出记忆范围的所有 U 形环境最终脱困。
- 不具备动态目标预测；前视盲区、细线/透明物、深度空洞仍是限制。膨胀距离、制动能力、
  感知延迟须用真实机体尺寸和实测速度验收。

## Ubuntu：先运行真正的算法仿真

依赖：Ubuntu 22.04 / ROS2 Humble、PX4 **v1.14.2**、匹配的 Gazebo/ros_gz、
OpenVINS；px4_msgs 必须匹配 release/1.14。工作空间已有 setup_env.sh 可作安装参考，
但不要让它把已匹配的依赖升级为 main。

```bash
cd ~/rm27-uav-swarm
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 对 Ubuntu 上已有源码做有锚点检查、可备份回退的小补丁
python3 tools/prepare_algorithm_sim.py \
  --px4 ~/PX4-Autopilot --openvins ~/catkin_ws_ov/src/open_vins

cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default
cd ~/catkin_ws_ov
colcon build --packages-select ov_core ov_init ov_msckf ov_eval
source install/setup.bash
cd ~/rm27-uav-swarm
colcon build
source install/setup.bash
python3 tools/test_algorithm_stack.py
chmod +x scripts/start_algorithm_sim.sh
```

补丁原因：1.14.2 无 UXRCE_DDS_SYNCT 参数，SITL 通过仅 POSIX 编译的
RM27_SIM_CLOCK 环境开关使 DDS 保持模拟时基；真实飞控不使用该开关。
OpenVINS 补丁让成功静止初始化后可发布状态，避免“等 VIO 才起飞、等起飞才输出 VIO”
死锁。原文件旁保留 .rm27-backup；不匹配的上游源码会拒绝修改。补丁后需要重新编译，
其 Ubuntu 编译和闭环效果尚未在本 Windows 工作环境验证。

```bash
# 终端 A：只起环境（四个模型），关闭旧的 WITH_RVIZ 导航入口
./scripts/start_algorithm_sim.sh

# 终端 B：先从一架的完整算法开始
source ~/catkin_ws_ov/install/setup.bash
source ~/rm27-uav-swarm/install/setup.bash
source /tmp/rm27_gz_env.sh
ros2 launch uav_bringup algorithm.launch.py sim:=true uav_id:=1 rviz:=true
```

此时不会自动解锁。先看 `/uav1/vio_health` 为 VALID、PX4 已融合视觉且位置有效，
再在确认起飞柱上方无遮挡的仿真环境中请求起飞：

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
```

悬停到 2m 后，在 RViz 用 2D Goal 选择局部目标（目标高度固定为 altitude 参数）。
Fixed Frame 是 `uav1_local_nwu`，与旧 map 的混合坐标约定隔离。
也可以直接发目标：

```bash
ros2 topic pub --once /uav1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'uav1_local_nwu'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /uav1/navigation_state
ros2 topic pub --once /uav1/command std_msgs/msg/String "{data: land}"
```

起飞/降落由 PX4 控制，不由二维地图证明其竖直通道安全。
新仿真每轮重启时，应重启算法节点及桥，重新 source 分区文件。不要同时运行旧 launch。
第二架可起相同 launch，`uav_id:=2 bridge_clock:=false`；只保留一个 /clock 发布桥。

## 真机标定：需要实际数据，不能用示例数值冒充

1. `python3 tools/probe_camera.py`（装好 pyrealsense2）记录实际 stream profiles 和 depth scale。
   设备名称不能证明你的拼装保留了 D4；能否连续输出有效深度才决定可用分支。
2. 固定双目、相机 IMU、RGB 和机体安装。关闭会影响标定的自动变化，确认曝光、分辨率
   与运行时一致；使用什么原始/整流像素就为它标定，不能混用参数。
3. 录制实际左右目和相机 IMU；同时保存相机驱动版本、时间戳模式、帧率和硬件序列号。

```bash
ros2 bag record -o calibration_ros2 \
  /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw /camera/camera/imu
```

4. Kalibr 通常使用 ROS1 .bag：可用 rosbags-convert 将 ROS2 bag 转为 .bag（安装工具后
   按 `rosbags-convert --help` 的版本参数转换），再在独立 Kalibr 环境运行。标定板尺寸必须
   实测；双目覆盖视场边角，相机—IMU数据要激励各轴旋转/平移并避免运动模糊。

```bash
kalibr_calibrate_cameras --bag calibration.bag --target target.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw

# imu.yaml 的噪声密度/随机游走由静止 IMU 长录包的 Allan 分析得出
kalibr_calibrate_imu_camera --bag calibration.bag --target target.yaml \
  --cam calibration-camchain.yaml --imu imu.yaml
```

文件名按 Kalibr 实际输出替换。导入前检查报告中的重投影残差、时间间隔、基线和轴向，
并用独立录包复验。时间偏移默认参与 Kalibr 优化；导入工具保留其符号与数值。
左右目时间偏移相差 >2ms 时导入拒绝，因为当前 OpenVINS 双目使用一份共享偏移。

5. 实测相机 IMU 到机体 FLU 的安装变换，填 `deploy/calibration/body.example.yaml`
   的副本；null 有意不能启动。独立 RGB 另做 RGB—双目外参和时间同步，当前不做虚假深度对齐。

```bash
python3 tools/import_kalibr.py \
  --camchain camchain-imucam-calibration.yaml --imu imu.yaml \
  --body measured_body.yaml --output calibration/uav1 --uav-id 1
```

工具自动把 Kalibr `T_cam_imu` 求逆为 OpenVINS `T_imu_cam`，校验旋转、内参、基线、
噪声，输出 OpenCV 可读配置，不覆盖已有结果。它是实测结果的导入器，不会凭空完成标定。

## 真机启动与性能验收

按 [PX4 1.14.2 参数说明](deploy/px4_1_14_2_vision.md) 设置并核对视觉融合。
飞控 Agent 和相机驱动先启动；raw/rectified 话题名称由实际驱动决定，可通过 launch 参数
cam0_topic/cam1_topic/imu_topic 显式传入。相机 IMU 轴向必须与 Kalibr 使用的 IMU 一致。

```bash
# 不确定 D4 时，先用软件深度；target_system 填实际 MAV_SYS_ID
ros2 launch uav_bringup algorithm.launch.py sim:=false uav_id:=1 \
  target_system:=1 calibration_dir:=$PWD/calibration/uav1 depth_source:=software
```

若设备确实提供可靠深度：`depth_source:=hardware`，同时在 body.yaml 填真实
T_body_depth，传正确 depth_topic/depth_info_topic/depth_scale。支持 16UC1 与 32FC1，
处理行填充和大小端；CameraInfo 的整流 P 决定反投影内参，保留采集时间戳。
默认深度单位 0.001 仅是常见值，必须以 probe_camera 输出或驱动约定核对。

OpenVINS 的 SLAM features 原已启用（max_slam=25、每批10），保持此基线。
本次没有宣称增加特征就必然提升精度，也不引入 LiDAR 或它的软硬件依赖。

```bash
# 安装 psutil 后，在 RK3576 和最终 RK3566 各运行相同工况
python3 tools/record_runtime.py --uav-id 1 --seconds 120 --output rk3566_runtime.json
python3 tools/benchmark_local_core.py --output rk3566_core.json
# 仿真采集附加 --sim-time；完整 VIO 分阶段耗时还在 /tmp/uav1_openvins_timing.txt
```

记录全链同时运行的频率、P95 延迟、最大接收间隔、CPU、内存与温度。source_rate 与
wall_rate 分开报告，避免将慢速仿真的墙钟帧率误判为算法实时性。需要至少验证长时间运行、
弱纹理、曝光变化、转弯、遮挡、U 形障碍、断图像、停导航、断通信、VIO 重置。
无数据的流会报告 count=0，不能被当成性能通过。

多机防碰需先对齐各机 VIO 的航向与原点，再给 fleet_safety.launch.py 提供实测
alignment_file（shared_heading_aligned: true；spawn_offsets: [x1,y1,x2,y2,...]）。
一个布尔值不执行对齐，它是操作前置条件声明；未满足时不要启动共享距离判断。
该 launch 只起一份 collision_monitor，其请求不能绕过各机局部导航。

## 验证范围与参考

本次 Windows 上运行离线几何、实际节点回调的消息桩测试、OpenCV 配置读取和人工纹理
双目深度测试。没有运行 ROS2 DDS/Gazebo 闭环、没有真实相机标定数据、没有 RK3566 板端数据，
因此这三项仍待执行；不能把此变更直接称作飞行验收完成。

- [Kalibr 双目标定](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)
- [Kalibr 相机—IMU 标定](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Kalibr IMU 噪声模型](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [OpenVINS 配置解析](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/core/VioManagerOptions.h)
- [OpenVINS ROS2 odom 输出](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS2Visualizer.cpp)
