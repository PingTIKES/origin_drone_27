# origin_drone_27

RoboMaster 2027 无人机视觉定位、深度建图、局部规划与四机协同工作空间。仿真和真机共用 `uav_bringup/algorithm.launch.py`。飞控均要求 **PX4 1.14.3**，ROS 消息使用固定的 `px4_msgs release/1.14` 提交。仿真从 MicoAir 的 1.14.3 源码构建 SITL；真机为已刷 1.14.3 的 MicoAir743v2-AIO-35A，具体固件构建来源仍需在实机核对。SITL 时钟补丁不会刷入真机。

```text
双目 + 相机 IMU → OpenVINS → VIO 桥 → PX4 EKF2 → 本地位姿/姿态
双目软件深度或实测硬件深度 → 机体系点云 → 滚动局部地图
手动目标或四机任务 → 局部规划 → Offboard 航点 → PX4
```

Gazebo 提供传感器和动力学；定位、避障与控制不读取真值位姿或理想深度。仿真另外提供由场地 STL 预先生成的全局参考图，仅供 RViz 对照，当前规划器不使用它。独立 RGB 的目标检测尚未完成，`yolo_detector.py` 仅保留后续接口，不参与当前定位和避障。

## 功能包、输入和输出

下表中的 `N` 为机号 1–4。普通节点话题在 `/uavN`，PX4 DDS 话题在 `/px4_N`。真实相机驱动在算法入口之外启动。

| 包或组件 | 作用 | 主要输入 | 主要输出 |
|---|---|---|---|
| `uav_bringup` | 按仿真或真机参数启动算法图、传感器桥和可选 RViz | `sim`、`uav_id`、标定目录、深度来源、驱动话题 | 下列节点与话题 |
| `uav_localization` + OpenVINS | 双目惯性里程计；转换坐标与时间戳并向 PX4 提交视觉里程计 | `/uavN/cam0/image_raw`、`cam1/image_raw`、`imu0`、实测标定 | `/uavN/odomimu`、`vio_health`、`/px4_N/fmu/in/vehicle_visual_odometry` |
| `uav_perception` | 真机驱动话题中继、双目软件深度或硬件深度转机体系点云 | 左右目、IMU、可选深度图和 CameraInfo | `/uavN/d435i/depth/image_raw`（软件深度）、`/uavN/obstacles` |
| `uav_mapping` | 点云生成滚动局部栅格；仿真发布预制场地参考图，并发布估计里程计和机体 TF | `/uavN/obstacles`、PX4 本地位置和姿态、打包的场地先验图 | `/uavN/local_map`、仿真 `/uavN/global_map`、`/uavN/odom`、`/tf` 中 `uavN_local_nwu → uavN` |
| `uav_planning` | 手动目标转航点；基于新鲜局部地图规划或保持 | `/uavN/goal_pose` 或 `waypoint_in`、`local_map`、PX4 位置 | `/uavN/waypoint`、`navigation_state`、`local_path`、`desired_yaw` |
| `uav_control` | 收到请求后管理 Offboard、解锁、起飞、航点和降落 | `waypoint`、`vio_health`、PX4 状态、`/uavN/start_mission`、`command` | `/uavN/state`、PX4 `offboard_control_mode`、`trajectory_setpoint`、`vehicle_command` |
| `uav_swarm`、`uav_msgs` | 四机任务分配、状态和避碰消息；提供接口类型 | `/swarm/command`、`/swarm/uav_state` 等 | 各机 `waypoint_in`、`safety_waypoint` 及 `/swarm/state` |

`vio_health=VALID` 只说明 VIO 桥接受了新鲜数据，还须确认 PX4 EKF2 **实际融合**了视觉数据。`/uavN/obstacles` 或 `/tf` 没有频率时，应检查各自上游输入和时间戳；仅在 `ros2 topic list` 中看到名称不代表消息在发布。

## 一、单机仿真流程

### 安装与启动

主机使用 Ubuntu 22.04、ROS 2 Humble。仓库与工作空间放在纯英文路径，避免 ROS 接口生成器在中文路径下构建失败。安装脚本固定 PX4 提交 `08310a5e8ac64d02edb41523460e7dc267298deb`、`px4_msgs release/1.14` 和 OpenVINS 的已测提交；已有目录版本不符会停止。

```bash
git clone https://github.com/PingTIKES/origin_drone_27.git ~/origin_drone_27
cd ~/origin_drone_27
./setup_env.sh sim
```

终端 A 启动 Gazebo、PX4 SITL 和 MicroXRCEAgent。终端 B 须在终端 A 写出 Gazebo 环境文件后运行，启动传感器桥、OpenVINS、深度、地图、规划和控制：

```bash
# 终端 A
cd ~/origin_drone_27
PX4_DIR=~/PX4-Autopilot-1.14.3 ./scripts/start_algorithm_sim.sh 1
```

```bash
# 终端 B
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
source /tmp/origin_drone_27_gz_env.sh
PYTHONNOUSERSITE=1 ros2 launch uav_bringup algorithm.launch.py \
  sim:=true uav_id:=1 depth_source:=software rviz:=true
```

仿真固定使用双目软件深度，`/clock` 来自 Gazebo。每次重启仿真，应重启算法节点并重新 `source` Gazebo 环境文件。入口不会自动解锁或起飞。

### 起飞前检查与操作

在加载相同 ROS 环境的终端 C 检查实际消息。`ros2 topic hz` 需等待数秒才会显示统计；单次 `echo` 不能代替持续频率检查。

```bash
ros2 topic hz /uav1/cam0/image_raw
ros2 topic hz /uav1/cam1/image_raw
ros2 topic hz /uav1/imu0
ros2 topic hz /uav1/odomimu
ros2 topic echo --once /uav1/vio_health
ros2 topic hz /px4_1/fmu/out/vehicle_local_position
ros2 topic hz /uav1/d435i/depth/image_raw
ros2 topic hz /uav1/obstacles
ros2 topic hz /uav1/local_map
ros2 topic hz /uav1/global_map
ros2 topic hz /uav1/odom
ros2 topic hz /tf
ros2 topic echo --once /uav1/navigation_state
```

这些检查命令应分别运行，不要在同一终端等待某个持续 `hz` 命令时继续粘贴后续命令。确认 `vio_health` 持续为 `VALID`、PX4 本地位置有效且已融合视觉、`obstacles`/`local_map`/`tf` 持续更新。RViz Fixed Frame 设为 `uav1_local_nwu`。若显示 `Frame [uav1_local_nwu] does not exist`，先查 PX4 位置和姿态、点云采集时间与 `/clock`，再查 `rolling_mapper` 日志；地图和 TF 由该节点产生。

若 `/tf` 有频率而 `obstacles`、`local_map` 均无频率，先查 `/uav1/d435i/depth/image_raw`。深度也没有频率时，检查双目左右图像及 `software_stereo` 日志；深度有频率但点云没有时，检查 `stereo_depth_node` 日志。双目看到无纹理或极暗场景时可能没有可用视差，节点会拒绝生成虚假的障碍点云。`/uav1/navigation_state=HOLD_MAP_STALE` 表示地图链路不满足导航要求。RViz 的 Fixed Frame 位于左侧 `Displays → Global Options`，而 `TF` 可视化项可以通过 `Add → TF` 增加；Fixed Frame 已设置并不保证地图话题有数据。

`stereo_depth_node` 每 5 秒对没有深度输入或有深度输入却无点云的情况给出警告。查看其日志可用 `ls -t ~/.ros/log/python3_*.log | head` 找到当前进程文件，并结合 `ros2 node info /uav1/stereo_depth_node` 核对订阅名。更新代码后必须重建 `uav_perception` 并重启算法 launch；仅在旧进程运行时修改源码不会改变该进程的行为。

仿真 x500 的 `/uavN/obstacles` 在深度反投影到机体 FLU 后，会滤掉落入已知机体碰撞盒和四个桨盘扫掠范围的点。`stereo_depth_node` 日志每 5 秒报告过滤点数；若始终为 0，原地打转就不能归因于点云中的机体点，应继续检查双目深度伪点、`local_map` 的已知/未知区域、起点膨胀、VIO 与导航状态。该掩膜仅针对仿真 x500；真机默认关闭，须测量完整机架、相机安装位姿和桨盘尺寸后才能配置对应模型。掩膜只删除位于机体物理空间内的点，不会清除相机前方的整片区域。

### 在 RViz 对照地图、轨迹与坐标系

仿真启动的 `prior_mapper` 立即发布 `/uav1/global_map`，无需等待相机看到场地。它由当前 RMUC2025 STL 的 1.5–2.5 m 高度层预先生成，0.1 m 栅格覆盖整片场地；场地内空白为该高度层的参考空闲区域，外部为未知。预制栅格采用 Gazebo 世界 ENU 坐标，发布节点利用仿真出生点和 PX4 初始姿态将其对齐到各机的 `uavN_local_nwu`；视觉定位稳定约 2 秒后固定这个显示变换。相机模型安装姿态为零，Gazebo 相机 +X 视线与 x500 机头 +X 一致。RViz 的 TF 中还可查看 `uavN_camera_mount` 和 `uavN_camera_optical`；光学坐标系按 ROS 约定以 +Z 为视线。先验图是**仿真可视化参考**，并不保证飞行安全：STL 变更、实际高度不同、动态障碍、出生点设置变化或定位漂移都会造成偏差。局部规划仍只使用实时 `/uav1/local_map`。场地 STL 更新后，先运行 `RM27_FIELD_MESH=/path/to/rmuc_2025.stl PYTHONNOUSERSITE=1 python3 tools/generate_field_prior.py` 重新生成打包地图，再重建 `uav_mapping`。

RViz 默认打开 `PriorFieldMap`、`Px4EstimatedOdom`、`TF` 和 `LocalPath`；可勾选 `ObservedLocalMap` 查看导航当前使用的局部窗口。排查黑块时，先暂时关闭 `PriorFieldMap` 和 `ObservedLocalMap` 的叠加，分别打开 `RawLocalHits` 和 `ObstaclePoints`：原始点云是相机报告的位置，`RawLocalHits` 中的 100 是高度带内的原始击中，`ObservedLocalMap` 中额外的 100 是 0.35 m 安全膨胀。两张局部图中 0=已观测空闲，-1=未观测，100=占用；规划器对后两者均不放行。低飞时若点云在机体下方约 0.25 m 的高度带内出现成片水平面，地面也会成为占用格，不应简单清空或放行；先核对飞行高度、相机外参与深度质量。Fixed Frame 为 `uav1_local_nwu`，TF 树应出现 `uav1_local_nwu → uav1`。`/uav1/odom` 是 PX4 的估计里程计，**不是**独立真值；若 VIO 发散，里程计相对先验图也会偏移。检查 TF 可单独运行：

```bash
ros2 run tf2_ros tf2_echo uav1_local_nwu uav1
```

在确认 VIO 稳定、PX4 已融合视觉、点云与局部地图持续更新后，再在仿真中测试目标导航；若出现 `HOLD_MAP_STALE`、里程计跳变或视觉失效，停止任务并保存录包，不把积累图当作继续飞行的依据。

局部避障仅在当前高度的已观测空闲栅格内运行：0.1 m 局部图对障碍膨胀 0.35 m，A* 寻路、路径平滑并每次只下发最多 0.4 m 的安全航点；规划超时才尝试有完整空闲走廊的 VFH。黑色占据格及未知格都不可通行。由于只有一台前视相机，下一段路径偏离机头超过 15° 时，导航状态变为 `ALIGNING_PATH`：先锁定当前位置并原地转向，实际航向进入 8° 且水平速度低于 0.15 m/s 后才下发前进航点；Offboard 还会拒绝相对机头超过 20° 的平移目标，避免旧航点造成侧飞或倒飞。`HOLD_NO_PATH`、`HOLD_BLOCKED_START` 或 `HOLD_CORRIDOR_BLOCKED` 表示当前视野没有可证明安全的路线，飞机应原地悬停，不会靠持续旋转来宣称找到路线；应检查 `/uav1/navigation_state`、`/uav1/local_map` 和双目深度是否把地面或墙误判为障碍。RViz 的里程计箭头只保留当前一帧，以免旋转历史显示成扇形。

确认仿真起飞区无遮挡后请求起飞。服务返回 `Start requested` 只表示请求已接受，实际解锁和起飞仍须观察 `/uav1/state`、PX4 状态和 Gazebo。

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
ros2 topic echo /uav1/state
```

悬停稳定后，在 RViz 使用 2D Goal，或向本地 NWU 坐标系发布目标。`altitude` 默认 2 m，`local_goal` 会把二维目标转为对应高度。

```bash
ros2 topic pub --once /uav1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'uav1_local_nwu'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /uav1/navigation_state
ros2 topic pub --once /uav1/command std_msgs/msg/String "{data: land}"
```

确认降落和 PX4 上锁后停止算法及仿真终端。四机启动、时钟桥和任务指令见 [四机仿真说明](SWARM_SIMULATION.md)。

## 二、真机流程：每架分别执行

真机示例为 `uav1`；其他飞机要同时替换机号、`MAV_SYS_ID`、`px4_N` 命名空间和各自标定文件。先在拆桨台架完成配置与检查，再进行受控低速飞行验收。机载板执行：

```bash
cd ~/origin_drone_27
./setup_env.sh onboard
```

机载模式安装依赖和本工作空间，不编译 PX4 SITL，也不改写已刷飞控固件。

### 1. 核对飞控、接线和时间

1. 在 QGroundControl 或 PX4 控制台确认板型、**实际刷入的 PX4 1.14.3 构建来源**、`MAV_SYS_ID` 和 DDS 消息定义。项目消息包是 `px4_msgs release/1.14`；定制固件必须核对 `dds_topics.yaml`。飞控 `uxrce_dds_client` 与板端 Agent 的串口和波特率应一致；每机使用唯一 `UXRCE_DDS_KEY` 和命名空间 `px4_N`。`TARGET_SYSTEM` 必须填实际 `MAV_SYS_ID`。
2. 按 [PX4 1.14.3 真机配置](deploy/px4_1_14_3_vision.md) 核对视觉融合、失联动作、速度限制、无板载磁罗盘的航向方案和遥控接管。MicoAir743v2-AIO-35A 的 Bluejay 电调需核对 DShot 协议、电机映射和转向；拆桨检查后再装桨。参数表是核对基线，不是自动写参脚本。
3. 真机 ROS 使用系统时钟；相机图像和 IMU 消息须带正确的**采集时间戳**，与 PX4/Agent 同步后的时基一致。仿真专用 `RM27_SIM_CLOCK` 不用于真机。

### 2. 确认相机能力并采集标定数据

实际相机为 D430i 加独立 RGB 的组装；是否能提供硬件深度取决于模块与驱动，不能由名称推断。安装 `pyrealsense2` 后运行：

```bash
python3 tools/probe_camera.py
```

记录设备序列号、左右目和 IMU 的分辨率、帧率、格式及真实深度流；若有深度传感器，记录 `depth_scale_m` 并验证深度图连续输出。没有可靠硬件深度时选 `DEPTH_SOURCE=software`。独立 RGB 不作为当前链路的深度参考。

固定相机、IMU 与机体安装；标定期间与飞行时保持相同焦距、分辨率、曝光策略、驱动设置和时间戳模式。先启动实际相机 ROS 驱动，用 `ros2 topic list -t` 和 `ros2 topic hz <话题>` 确认左右目、IMU 持续发布。记录的图像须与运行时选择的图像类型一致：**原始图像标定就传原始图像话题；整流图像须有与之匹配的标定**。入口默认话题名是 `/camera/camera/infra1/image_rect_raw`、`infra2/image_rect_raw` 和 `/camera/camera/imu`。如果 Kalibr 用 `image_raw` 标定，启动时必须用 `CAM0_TOPIC` 和 `CAM1_TOPIC` 改成对应原始流。

可按实际话题录 ROS 2 bag，再转换为 Kalibr 所需格式；标定板尺寸必须实测。双目内参、畸变、外参使用覆盖视场边缘的数据；相机—IMU 外参与时间偏移使用充分的各轴激励；IMU 噪声密度和随机游走通过静止长录包的 Allan 分析获得。例如：

```bash
ros2 bag record -o calibration_ros2 \
  /camera/camera/infra1/image_raw /camera/camera/infra2/image_raw /camera/camera/imu
# 在独立 Kalibr 环境完成相机和 IMU 标定；用实际输出替换文件名
kalibr_calibrate_cameras --bag calibration.bag --target target.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /camera/camera/infra1/image_raw /camera/camera/infra2/image_raw
kalibr_calibrate_imu_camera --bag calibration.bag --target target.yaml \
  --cam calibration-camchain.yaml --imu imu.yaml
```

检查重投影残差、双目基线方向、时间偏移、IMU 坐标轴，再用独立录包复验。`tools/import_kalibr.py` 要求左右目时间偏移之差不超过 2 ms；超过时先解决同步问题。

### 3. 测量机体外参并生成每机标定目录

机体采用 `FLU`（x 前、y 左、z 上）。测量**相机 IMU → 机体**的 `T_body_imu`，将 [body.example.yaml](deploy/calibration/body.example.yaml) 复制为机号专用文件并填入真实 4×4 齐次矩阵；示例中的 `null` 不能启动。选择硬件深度时，还须测量**深度光学参考系 → 机体**的 `T_body_depth`，不能用独立 RGB 外参替代。软件深度通过双目标定推导深度参考系，无须填写 `T_body_depth`。

```bash
cp deploy/calibration/body.example.yaml ~/measured_body_uav1.yaml
# 手工填入实测矩阵后导入；输出目录必须尚不存在
python3 tools/import_kalibr.py \
  --camchain camchain-imucam-calibration.yaml \
  --imu imu.yaml --body ~/measured_body_uav1.yaml \
  --output ~/calibration/uav1 --uav-id 1
```

导入后 `~/calibration/uav1/` 应有 `estimator_config.yaml`、`kalibr_imucam_chain.yaml`、`kalibr_imu_chain.yaml`、`body.yaml`。工具把 Kalibr 的 `T_cam_imu` 求逆成 OpenVINS 的 `T_imu_cam` 并校验配置；不会凭空测得外参，也不会覆盖已有目录。每架机各自测量和生成。更多限制见 [算法链路说明](ALGORITHM_PIPELINE.md)。

### 4. 启动驱动、算法和地面检查

先启动相机 ROS 驱动并验证左右目和 IMU；机载脚本**不会**启动驱动。根据实际串口、标定图像流和 `MAV_SYS_ID` 启动脚本，它会启动串口 MicroXRCEAgent 和同一 `algorithm.launch.py`。以下示例用软件深度：

```bash
cd ~/origin_drone_27
CALIBRATION_DIR=~/calibration/uav1 TARGET_SYSTEM=1 \
SERIAL_DEV=/dev/ttyS1 SERIAL_BAUD=921600 \
CAM0_TOPIC=/camera/camera/infra1/image_raw \
CAM1_TOPIC=/camera/camera/infra2/image_raw \
IMU_TOPIC=/camera/camera/imu DEPTH_SOURCE=software \
./deploy/start_onboard.sh 1
```

`CAM0_TOPIC` 等应改为**实际驱动且与标定一致**的话题。脚本默认 `ROS_DOMAIN_ID=42`、CycloneDDS；机载板、地面站和飞控桥的网络/DDS 域须一致。若硬件深度已实测可用、`body.yaml` 含真实 `T_body_depth`，改为 `DEPTH_SOURCE=hardware`，并设置 `DEPTH_TOPIC`、`DEPTH_INFO_TOPIC` 和按驱动约定核对的 `DEPTH_SCALE`。`DEPTH_SCALE=0.001` 仅是默认值。软件深度没有硬件深度话题依赖。

在同一 ROS 域的新终端加载 `/opt/ros/humble/setup.bash`、OpenVINS 和本仓库 `install/setup.bash`，依次检查：

```bash
ros2 topic hz /camera/camera/infra1/image_raw
ros2 topic hz /camera/camera/infra2/image_raw
ros2 topic hz /camera/camera/imu
ros2 topic hz /uav1/odomimu
ros2 topic echo --once /uav1/vio_health
ros2 topic hz /px4_1/fmu/out/vehicle_local_position
ros2 topic hz /uav1/obstacles
ros2 topic hz /uav1/local_map
ros2 topic hz /tf
ros2 topic echo --once /uav1/state
```

每条 `hz` 命令应分别运行，前三个驱动话题按实际配置替换。硬件深度还要检查深度图和 CameraInfo 的类型、帧率与时间戳。`/uav1/obstacles` 没有数据时，沿“深度图 → CameraInfo → `stereo_depth_node`”排查；`/tf` 缺失时，检查 PX4 本地位置和姿态、点云时间戳及 `rolling_mapper` 日志。RViz Fixed Frame 为 `uav1_local_nwu`。飞控控制台查 `listener vehicle_visual_odometry`、`listener estimator_status`，再用 ULog 核对 EV 融合和创新；**收到视觉消息不等于 EKF2 已融合**。起飞前还要确认本地位置有效、`vio_health` 稳定为 `VALID`、地图持续更新、遥控接管可用。

### 5. 受控飞行与停止

拆桨台架先验证串口断开、相机断流、VIO 失效、通信断开和遥控接管行为；再按场地与机体安全流程进行低速、低高度首飞。入口不会自动解锁。具备前述条件后在地面站监看 PX4 状态并请求任务：

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
ros2 topic echo /uav1/state
# 需结束时
ros2 topic pub --once /uav1/command std_msgs/msg/String "{data: land}"
```

服务返回成功仅表示接受请求；PX4 是否解锁、起飞、降落以飞控状态和实机观察为准。确认着陆、上锁后停止算法和相机驱动。当前真实 RGB 识别、多机实测坐标对齐、整机重量与板端实时性能仍需单独验收。

## 离线测试

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
cd ~/origin_drone_27
PYTHONNOUSERSITE=1 python3 tools/test_algorithm_stack.py
```

离线测试不能替代 Gazebo、DDS、真机台架和飞行验收。
