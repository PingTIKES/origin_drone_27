# 无雷达视觉算法闭环：仿真与真机共用

硬件约束：整机 <249g，RK3576 调试 / RK3566 最终平台；用户实际相机为
D430i + 独立 RGB，使用相机内置 IMU。仿真和真机统一使用 PX4 1.14.3：仿真从 MicoAir 公开源码构建 SITL，真机为 MicoAir743v2-AIO-35A，已刷 1.14.3 固件（具体构建来源待核对）。是否有 D4 尚未确认。
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
  swarm_agent → safety_waypoint（通信避碰请求，也必须通过障碍检查）
```

`algorithm.launch.py` 的定位、避障、规划和控制不读取 Gazebo 真值、理想深度、GPS 定位或预制场地地图。仿真另发布 STL 生成的全场先验图到 `/uavN/global_map`，仅供 RViz 观察。PX4 使用惯性融合控制，这是估计器的一部分。
独立 RGB 暂不参与定位和避障；`yolo_detector.py` 仅保留后续实现接口，未提供训练模型时不产生检测结果。四机任务只通过带有效期的 `SwarmCommand` 进入每机局部导航。

## 局部地图和规划行为

- 固定朝向 local NWU，12×12m、0.1m 分辨率，窗口随位置平移；不是每帧随机体旋转。
- 以采集时间戳匹配 PX4 估计位置和四元数（含 roll/pitch），容差 0.15s。
- 对数占据证据：射线空闲/终点占据，同帧每格只投票一次，命中优先；保存 8s，过期变未知。
- 飞行高度带 ±0.25m，障碍膨胀 0.35m。楼板/地面射线裁剪到高度带，只能发现空闲，
  不擦除已有墙体。高度层变化清图。不对整个视场或无深度像素宣称空闲。
- A* 不穿未知、不斜切障碍角、不把起点吸附到墙另一边；路径拉直也使用
  保守的 supercover 检查。目标在窗口外时只执行可达已知空间中的部分路径。
- A* 搜索预算 25ms；仅预算耗尽且地图新鲜时，72 扇区 VFH 从已验证通路中选短距离动作。
  无路、地图断流、定位失效不降级为盲飞。VFH 不另起节点争抢 waypoint。
- 每次位置目标最大前移 0.4m；失去上游目标后固定位置悬停。PX4 速度上限另设 0.5m/s；
  短航点本身不是速度限制。导航测得速度超过 0.6m/s 会进入保持。
- 不支持沿未观测高度上下绕障。前视相机遇未知方向可以原地转向观察；局部图不保证
  在视场外或超出记忆范围的所有 U 形环境最终脱困。
- 不具备动态目标预测；前视盲区、细线/透明物、深度空洞仍是限制。膨胀距离、制动能力、
  感知延迟须用真实机体尺寸和实测速度验收。

## Ubuntu：先运行真正的算法仿真

仿真依赖为 Ubuntu 22.04、ROS 2 Humble、MicoAir PX4 **1.14.3 SITL**、Gazebo Garden、OpenVINS、MicroXRCEAgent 和 `px4_msgs release/1.14`。工作空间路径必须只含英文字符；ROS 2 接口生成器在中文路径下会构建失败。

统一使用安装脚本，不再混装其他 PX4 或消息版本：

```bash
git clone https://github.com/PingTIKES/origin_drone_27.git ~/origin_drone_27
cd ~/origin_drone_27
./setup_env.sh sim
```

脚本固定 MicoAir PX4 提交 `08310a5e8ac64d02edb41523460e7dc267298deb`、`px4_msgs release/1.14` 的已测提交和 OpenVINS 的已测提交，随后应用受锚点检查的补丁并完成构建。已有依赖目录版本不一致时脚本会停止，不会静默覆盖。真机板执行 `./setup_env.sh onboard`，不下载或编译 PX4 SITL。

补丁原因：该 1.14.3 源码无 UXRCE_DDS_SYNCT 参数，SITL 通过仅 POSIX 编译的
RM27_SIM_CLOCK 环境开关使 DDS 保持模拟时基；真实飞控不使用该开关。
OpenVINS 补丁让成功静止初始化后可发布状态，避免“等 VIO 才起飞、等起飞才输出 VIO”
死锁。原文件旁保留 .rm27-backup；不匹配的上游源码会拒绝修改。补丁后需要重新编译，
本次 Ubuntu 单机起飞与短时悬停的验证结果见文末；目标导航等项目仍需逐项验收。

```bash
# 终端 A：先起单机环境；不传 1 则默认四机
PX4_DIR=~/PX4-Autopilot-1.14.3 ./scripts/start_algorithm_sim.sh 1

# 终端 B：先从一架的完整算法开始
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
source /tmp/origin_drone_27_gz_env.sh
PYTHONNOUSERSITE=1 ros2 launch uav_bringup algorithm.launch.py sim:=true uav_id:=1 rviz:=true
```

此时不会自动解锁。先看 `/uav1/vio_health` 为 VALID、PX4 已融合视觉且位置有效，
再在确认起飞柱上方无遮挡的仿真环境中请求起飞：

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
```

Gazebo 世界位姿确认悬停到约 2m，且 `/uav1/obstacles`、`/uav1/local_map`
持续发布后，在 RViz 用 2D Goal 选择局部目标（目标高度固定为 altitude 参数）。
Fixed Frame 是 `uav1_local_nwu`。
也可以直接发目标：

```bash
ros2 topic pub --once /uav1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'uav1_local_nwu'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /uav1/navigation_state
ros2 topic pub --once /uav1/command std_msgs/msg/String "{data: land}"
```

服务返回 `Start requested` 只表示接受请求；VIO 无效时节点仍保持 `INIT`，
不能把服务返回 `success=True` 当作已解锁。起飞/降落由 PX4 控制，
不由二维地图证明其竖直通道安全。
仿真每轮重启时，应重启算法节点及桥，并重新 source 分区文件。
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

真机已刷 PX4 1.14.3。按 [PX4 1.14.3 真机参数说明](deploy/px4_1_14_3_vision.md) 核对固件目标、参数和视觉融合；仿真补丁仅作用于 POSIX SITL，不刷入真机。
飞控 Agent 和相机驱动先启动；raw/rectified 话题名称由实际驱动决定，可通过 launch 参数
cam0_topic/cam1_topic/imu_topic 显式传入。相机 IMU 轴向必须与 Kalibr 使用的 IMU 一致。

```bash
# 不确定 D4 时，先用软件深度；target_system 填实际 MAV_SYS_ID
PYTHONNOUSERSITE=1 ros2 launch uav_bringup algorithm.launch.py sim:=false uav_id:=1 \
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

多机防碰需先实测并对齐各机 VIO 的航向与原点，再把平移和航向写入每机 `swarm_agent` 参数。
仿真 `algorithm_swarm_sim.launch.py` 使用已知出生点；真机没有完成对齐时不得启用共享距离判断。
`swarm_agent` 的安全请求仍进入 `local_navigator`，不能绕过在线障碍地图。

## 验证范围与参考

已实现离线几何、节点回调的消息桩测试、OpenCV 配置读取和人工纹理双目深度测试。
MicoAir PX4 1.14.3 SITL 已编译，OpenVINS 4 个包和清理后的本工作空间 9 个包完成构建，38 项离线测试通过。在 Gazebo `default` 世界，单机模型创建、MicroXRCEAgent 连接与 PX4 ROS 状态话题已验证；视觉融合、起飞和导航尚未在 1.14.3 上验证。

此前在 Ubuntu 22.04 上使用 PX4 v1.14.2 和 OpenVINS 完成单机 SITL 验证；这不是 1.14.3 的飞行验收结果。单机 Gazebo 模型、Agent、
相机与 IMU 话题当时已验证。修正出生点的启动参数后，模型处于预期
停机坪，VIO 达到 `VALID`，PX4 可解锁并进入 Offboard。修复局部导航器的地面保持
航点覆盖起飞目标后，Gazebo 模型从静止时约 0.187 m 升至约 2.106 m 并短时悬停，
单机物理起飞已复测。状态机要求估计高度连续稳定到达才进入任务阶段，且忽略
偏离巡航高度的航点。目标点测试报告 `HOLD_NO_PATH`/`HOLD_BLOCKED_START`，
后来 VIO 跳变触发 Offboard 安全退出；目标导航和正常降落未通过。切换至 1.14.3 后需重验上述项目。
没有真实相机标定数据或 RK3566 板端数据，四机协同和实机飞行仍须单独验收。

- [Kalibr 双目标定](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)
- [Kalibr 相机—IMU 标定](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Kalibr IMU 噪声模型](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [OpenVINS 配置解析](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/core/VioManagerOptions.h)
- [OpenVINS ROS2 odom 输出](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS2Visualizer.cpp)
