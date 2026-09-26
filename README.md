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
| `uav_mapping` | 点云生成滚动局部栅格；仿真发布预制场地参考图，并发布估计里程计和机体 TF | `/uavN/obstacles`、PX4 本地位置和姿态、打包的场地先验图 | `/uavN/local_map`、仿真 `/uavN/global_map`、`/uavN/odom`、`/tf` 中仿真 `uavN_map → uavN_odom → uavN_base_link`（真机的 `map → odom` 暂为无全局锚点的单位变换） |
| `uav_planning` | 手动目标转航点；基于新鲜局部地图规划或保持 | `/uavN/goal_pose` 或 `waypoint_in`、`local_map`、PX4 位置 | `/uavN/waypoint`、`navigation_state`、`local_path`、`desired_yaw` |
| `uav_control` | 收到请求后管理 Offboard、解锁、起飞、航点和降落 | `waypoint`、`vio_health`、PX4 状态、`/uavN/start_mission`、`command` | `/uavN/state`、PX4 `offboard_control_mode`、`trajectory_setpoint`、`vehicle_command` |
| `uav_swarm`、`uav_msgs` | 四机任务分配、状态和避碰消息；提供接口类型 | `/swarm/command`、`/swarm/uav_state` 等 | 各机 `waypoint_in`、`safety_waypoint` 及 `/swarm/state` |

`vio_health=VALID` 只说明 VIO 桥接受了新鲜数据，还须确认 PX4 EKF2 **实际融合**了视觉数据。`/uavN/obstacles` 或 `/tf` 没有频率时，应检查各自上游输入和时间戳；仅在 `ros2 topic list` 中看到名称不代表消息在发布。

## 算法参数文件

每个有可配置 ROS 节点的功能包都有一个 `config/params.yaml`。启动 `algorithm.launch.py`（或四机 `algorithm_swarm_sim.launch.py`）时，会按**节点名**读取该包的文件；同一包中不同节点各有一个 `ros__parameters` 段。文件中的具体值优先于启动文件的默认值，修改后**重启算法 launch**即可生效，不需要改 Python 源码。第一次新增这些文件后需按安装流程构建一次；使用 `colcon build --symlink-install` 时，源目录里的参数文件与安装目录相连。若使用普通安装，修改后重新构建相应包。

| 功能包 | 文件 | 节点与主要可调项 |
|---|---|---|
| `uav_localization` | `src/uav_localization/config/params.yaml` | `openvins` 的 TF 发布开关，`vio_to_px4` 的数据新鲜度、方差和跳变阈值，`compare_vio_gt` 的诊断周期 |
| `uav_perception` | `src/uav_perception/config/params.yaml` | `software_stereo` 的处理率、纹理阈值和同步容差；`stereo_depth_node` 的抽样、量程与机体掩膜；相机中继与预留的 `yolo_detector` |
| `uav_mapping` | `src/uav_mapping/config/params.yaml` | `rolling_mapper` 的地图大小、分辨率、记忆时间、膨胀半径、高度带及点云限制；`prior_mapper` 的机号 |
| `uav_planning` | `src/uav_planning/config/params.yaml` | `local_navigator` 的超时、速度、规划步长、制动与转向阈值；`local_goal` 的巡航高度 |
| `uav_control` | `src/uav_control/config/params.yaml` | `offboard_control` 的起飞、航点、朝向与数据超时限制 |
| `uav_swarm` | `src/uav_swarm/config/params.yaml` | `swarm_agent` 的状态、避碰和轨迹参数；`swarm_coordinator` 的搜索区与任务周期 |

`null` 表示沿用启动时计算的值，例如 `uav_id`、`px4_ns`、实际标定外参、传感器话题、`target_system`、仿真/真机深度模式。需要明确覆盖时，把 `null` 改成正确类型的数值、布尔值、字符串或数组。**四机共用同一份文件**，因此不要把 `uav_id`、`px4_ns` 等改成单机固定值，否则各机话题会串接。真机外参应优先修正实测标定目录，不能用参数覆盖来掩盖错误标定。`uav_bringup` 是启动包，没有自身声明的 ROS 节点参数；其 `sim`、`uav_id`、`calibration_dir` 等仍由 launch 参数控制。`uav_msgs` 只定义接口，没有节点参数。OpenVINS 的滤波器参数仍在对应的 `estimator_config.yaml`，不会被本表替代。

例如降低局部规划速度，改 `src/uav_planning/config/params.yaml` 中 `local_navigator.ros__parameters.max_speed`，保存并重启算法 launch。查看实际值可运行：

```bash
ros2 param get /uav1/local_navigator max_speed
ros2 param get /uav1/rolling_mapper inflation
```

## 一、单机仿真流程

### 安装与启动

主机使用 Ubuntu 22.04、ROS 2 Humble。仓库与工作空间放在纯英文路径，避免 ROS 接口生成器在中文路径下构建失败。安装脚本固定 PX4 提交 `08310a5e8ac64d02edb41523460e7dc267298deb`、`px4_msgs release/1.14` 和 OpenVINS 的已测提交；已有目录版本不符会停止。

```bash
git clone https://github.com/PingTIKES/origin_drone_27.git ~/origin_drone_27
cd ~/origin_drone_27
./setup_env.sh sim
```

终端 A 默认启动 `rmuc_2025_3m_vio_columns`：用户提供的约 3 m 围墙 STL 与 6 根高对比 VIO 识别柱。启动脚本核对并解压仓库内的 3 m STL，再按同一 SDF 重新生成先验 PGM/YAML，随后启动 Gazebo、PX4 SITL 和 MicroXRCEAgent。终端 B 须在终端 A 写出 Gazebo 环境文件后运行，启动传感器桥、OpenVINS、深度、地图、规划和控制：

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

这些检查命令应分别运行，不要在同一终端等待某个持续 `hz` 命令时继续粘贴后续命令。确认 `vio_health` 持续为 `VALID`、PX4 本地位置有效且已融合视觉、`obstacles`/`local_map`/`tf` 持续更新。仿真 RViz Fixed Frame 为 `uav1_map`；若显示 frame 不存在，先查 `prior_mapper` 的 PX4 初始位置，再查 `rolling_mapper` 的里程计 TF。机体 TF 需要 PX4 有效位置与姿态，局部地图还需要新鲜点云。

若 `/tf` 有频率而 `obstacles`、`local_map` 均无频率，先查 `/uav1/d435i/depth/image_raw`。深度也没有频率时，检查双目左右图像及 `software_stereo` 日志；深度有频率但点云没有时，检查 `stereo_depth_node` 日志。双目看到无纹理或极暗场景时可能没有可用视差，节点会拒绝生成虚假的障碍点云。`/uav1/navigation_state=HOLD_MAP_STALE` 表示地图链路不满足导航要求。RViz 的 Fixed Frame 位于左侧 `Displays → Global Options`，而 `TF` 可视化项可以通过 `Add → TF` 增加；Fixed Frame 已设置并不保证地图话题有数据。

`stereo_depth_node` 每 5 秒对没有深度输入或有深度输入却无点云的情况给出警告。查看其日志可用 `ls -t ~/.ros/log/python3_*.log | head` 找到当前进程文件，并结合 `ros2 node info /uav1/stereo_depth_node` 核对订阅名。更新代码后必须重建 `uav_perception` 并重启算法 launch；仅在旧进程运行时修改源码不会改变该进程的行为。

仿真 x500 的 `/uavN/obstacles` 在深度反投影到机体 FLU 后，会滤掉落入已知机体碰撞盒和四个桨盘扫掠范围的点。`stereo_depth_node` 日志每 5 秒报告过滤点数；若始终为 0，原地打转就不能归因于点云中的机体点，应继续检查双目深度伪点、`local_map` 的占用格、起点膨胀、VIO 与导航状态。该掩膜仅针对仿真 x500；真机默认关闭，须测量完整机架、相机安装位姿和桨盘尺寸后才能配置对应模型。掩膜只删除位于机体物理空间内的点，不会清除相机前方的整片区域。

双目软件深度会拒绝水平 7 像素窗口灰度标准差低于 1 的点。无纹理区域的水平视差不可可靠测量；仿真曾在静止时把同一条深灰色图像带交替估成 0.36–0.94 m 的障碍，导致 `RawLocalHits` 产生横排假击中。参数 `texture_std_min` 可在 `uav_perception/config/params.yaml` 中按实际相机噪声调整；拒绝的像素不会生成障碍点。调整后重启算法 launch，观察 `ObstaclePoints`、`RawLocalHits` 和 `ObservedLocalMap`，确认真实边缘仍有深度且近距假带消失。

### 在 RViz 对照地图、轨迹与坐标系

仿真先验地图是标准的 [`rmuc_2025_prior.yaml`](src/uav_mapping/config/rmuc_2025_prior.yaml) 和 [`rmuc_2025_prior.pgm`](src/uav_mapping/config/rmuc_2025_prior.pgm)。它由 3 m 围墙 STL 与 6 根识别柱碰撞轮廓的 1.5–2.5 m 高度层预先栅格化而成，分辨率 0.1 m；黑色是该高度层的占用，白色是场地内部参考空闲区，灰色是未知。`prior_mapper` 读取这两个文件并发布 `/uav1/global_map`（OccupancyGrid）；不需等待相机观测。地图坐标系 `uavN_map` 固定在 Gazebo 世界 ENU，PGM 左下角对应 YAML 的 `origin`。仿真出生点与首次有效 PX4 水平位置确定显示用 `uavN_map → uavN_odom`，旋转固定为 NWU 到 ENU 的 +90°，随后保持固定，不再用飞行中的位置或启动时尚未对齐的航向重算；`rolling_mapper` 发布 `uavN_odom → uavN_base_link` 和 `/uavN/odom`。`uavN_odom` 是 PX4 本地 NWU，`uavN_base_link` 是机体 FLU。相机光学帧仍为 `uavN_camera_optical`，+Z 指向视线。这个先验地图只供仿真对照，**不参与局部避障或控制**；模型变更、飞行高度差异、动态障碍及 VIO 漂移均可能使其与实际障碍不符。默认仿真每次启动都会从当前 3 m STL 与柱子 SDF 刷新 PGM；如手动修改模型，运行 `PYTHONNOUSERSITE=1 python3 tools/generate_field_prior.py` 并重建 `uav_mapping`（使用 `--symlink-install` 时安装地图随源文件更新）。先验图元数据记录 STL 与 SDF 的 SHA-256，便于核对 Gazebo 与 RViz 使用的是同一场地。真机没有该场地的全局定位锚点：`uavN_map → uavN_odom` 暂为单位变换，仅用于统一 TF 树；不发布仿真先验图，也不代表无人机在场地 PGM 中的位置。接入真实全局定位时须移除这个静态发布者，再由定位模块发布校正后的 `map → odom`。真机 RViz 以 `uavN_map` 为 Fixed Frame。RViz 的 `2D Goal Pose` 会以 `map` 帧发布目标，`local_goal` 查询当前 TF 后转换到 `odom`，再发布 PX4 本地 NED 航点；TF 不可用时不会下发目标。

仿真 RViz 默认打开 `PriorFieldMap`、`TF`（只显示 `uavN_base_link`）和 `LocalPath`；`ObservedLocalMap`、`Px4EstimatedOdom`、`RawLocalHits` 和 `ObstaclePoints` 默认关闭，可在排查时单独打开。排查黑块时，分别打开 `RawLocalHits` 和 `ObstaclePoints`：原始点云是相机报告的位置，`RawLocalHits` 中的 100 是高度带内的原始击中，`ObservedLocalMap` 中额外的 100 是 0.35 m 安全膨胀。两张局部图只输出 0 和 100：100=点云占用及其膨胀，0=没有保留的障碍击中；0 不代表相机证明该区域安全。先验 `/uav1/global_map` 仍是独立参考图，保留它自己的未知区，不参与导航。低飞时若点云在机体下方约 0.25 m 的高度带内出现成片水平面，地面也会成为占用格；先核对飞行高度、相机外参与深度质量。仿真 Fixed Frame 为 `uav1_map`，TF 树应出现 `uav1_map → uav1_odom → uav1_base_link`。`/uav1/odom` 是 PX4 的估计里程计，**不是**独立真值；若 VIO 发散，里程计相对先验图也会偏移。检查 TF 可单独运行：

```bash
ros2 run tf2_ros tf2_echo uav1_map uav1_odom
ros2 run tf2_ros tf2_echo uav1_odom uav1_base_link
ros2 run tf2_ros tf2_echo uav1_map uav1_base_link
```

`map → odom` 在启动时固定后，上述第一条变换应保持不变，机体移动时后两条应变化。若 Gazebo 中机体移动而后两条不变，检查 PX4 本地位置和 VIO；此 TF 来自 PX4 估计，不是 Gazebo 真值。

在确认 VIO 稳定、PX4 已融合视觉、点云与局部地图持续更新后，再在仿真中测试目标导航；若出现 `HOLD_MAP_STALE`、里程计跳变或视觉失效，停止任务并保存录包，不把积累图当作继续飞行的依据。

局部避障以当前高度观察到的障碍点为依据：0.1 m 局部图对占用格膨胀 0.35 m，其余格子输出 0；若到目标的直线无占用格，优先直接给出前方路径，否则 A* 寻路、路径平滑，每次只下发最多 0.4 m 的航点；规划超时才尝试 VFH。由于只有一台前视相机，未观察区域内可能存在墙，真机使用前必须先在仿真确认该策略。规划先于转向：路径方向偏离机头不超过 30° 时，边前进边修正偏航；超过 30° 时进入 `ALIGNING_PATH`，先锁定当前位置并转向，误差进入 15° 且水平速度低于 0.15 m/s 后再前进；Offboard 拒绝相对机头超过 35° 的平移目标。偏航设定点按经过时间以 45°/s 推进，PX4 自身的转向限制仍会生效。仿真 PX4 的 `MPC_XY_VEL_MAX` 和 `MPC_XY_CRUISE` 当前均为 0.5 m/s。`HOLD_MAP_STALE`、`HOLD_ALTITUDE_CHANGE_UNSUPPORTED`、`HOLD_OVERSPEED`、`POSE_INVALID` 分别表示地图未更新、高度不匹配、速度超限和位置失效；二值地图和转向门槛不会取消这些状态。RViz 的里程计箭头只保留当前一帧，以免旋转历史显示成扇形。

确认仿真起飞区无遮挡后请求起飞。服务返回 `Start requested` 只表示请求已接受，实际解锁和起飞仍须观察 `/uav1/state`、PX4 状态和 Gazebo。

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
ros2 topic echo /uav1/state
```

悬停稳定后，在 RViz 使用 2D Goal，或向本地 NWU 坐标系发布目标。`altitude` 默认 2 m，`local_goal` 会把二维目标转为对应高度。

```bash
ros2 topic pub --once /uav1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'uav1_odom'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /uav1/navigation_state
ros2 topic pub --once /uav1/command std_msgs/msg/String "{data: land}"
```

确认降落和 PX4 上锁后停止算法及仿真终端。四机启动、时钟桥和任务指令见 [四机仿真说明](SWARM_SIMULATION.md)。

### VIO 跳变恢复与任务暂停

VIO 桥检测到位置突跳后进入有界隔离，`vio_health` 为 `INVALID`，控制器进入
`VIO_HOLD`。仅在 PX4 本地位置仍有效、新鲜时，控制器持续发送 Offboard 心跳并
保持进入隔离时的位置与朝向；规划器清除旧目标，不继续沿旧路径飞行。
桥接器不发布伪造位置，也不把 PX4 已融合的位置反馈作为新的独立视觉观测。

默认要求候选 VIO 连续稳定 0.5 s，相邻候选与速度预测的残差不超过 0.03 m，
候选间隔不超过 0.1 s。单帧突跳允许的匀速预测偏差不超过 0.75 m、姿态偏差不超过
20°；超过 1 s 的 OpenVINS 输出缺口采用单独的时间和位移限制：首个新样本距离上一
正常样本最多 3 s，预测偏差最多 2.5 m、姿态偏差最多 90°。从首个异常样本开始的
连续性检查等待上限为 2 s；该计时不会从断流前的最后一帧开始。这些检查只能约束不连续性，不能
证明绝对位置准确。恢复时递增 PX4 EV `reset_counter`，控制器用 PX4 的
`delta_xy`、`delta_z`、`delta_heading` 同步调整悬停目标，避免追赶坐标重置前的目标。
缺失的重置增量、失效的 PX4 位置、持续超时仍进入 `FAULT`。

VIO 连续健康 1 s 后，控制器进入 `RECOVERY_HOLD`，继续悬停。确认定位与地图后，
使用原服务恢复，再重新在 RViz 打点；暂停期间输入的目标会丢弃：

```bash
ros2 topic echo /uav1/vio_diagnostics
ros2 topic echo /uav1/state
# 上面两条分别在独立终端运行；恢复命令：
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
```

桥接恢复参数在 `src/uav_localization/config/params.yaml`，控制器的
`vio_hold_timeout`（默认 5 s）和 `vio_resume_stable_time`（默认 1 s）在
`src/uav_control/config/params.yaml`。调整时须给桥接隔离与控制器稳定等待留出时间。
真机目前没有独立定位备份，持续 VIO 丢失后不能保证长时间定点留空；保留 PX4
定位失效处理，独立传感器接入之前不宣称具有冗余定位。

仿真相机仍为 30 Hz，OpenVINS `track_frequency` 改为 40 Hz 的接收上限：
Gazebo 的 32/36 ms 帧间隔会被原先严格的 30 Hz 限流丢弃一部分。对本次 3226 帧
录包按 OpenVINS 源码规则计算，30 上限接收 1871 帧（17.57 Hz），40 上限接收
全部帧（30.30 Hz）；这不等于已经证明 OpenVINS 所有跳变都已消除。

可离线回放桥接回调，命令不会发布 ROS 数据或控制飞行器：

```bash
PYTHONNOUSERSITE=1 python3 tools/replay_vio_recovery.py \
  ~/origin_drone_27/flight_bags/vio_fault_20260926_193536
```

复现录包建议额外包含 `/uav1/vio_diagnostics`、
`/px4_1/fmu/in/offboard_control_mode` 和 `/px4_1/fmu/in/trajectory_setpoint`，
以核对隔离期间的心跳及悬停目标。离线回放无法替代 PX4/Gazebo 闭环飞行验证。

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

每条 `hz` 命令应分别运行，前三个驱动话题按实际配置替换。硬件深度还要检查深度图和 CameraInfo 的类型、帧率与时间戳。`/uav1/obstacles` 没有数据时，沿“深度图 → CameraInfo → `stereo_depth_node`”排查；`/tf` 缺失时，检查 PX4 本地位置和姿态、点云时间戳及 `rolling_mapper` 日志。真机 RViz Fixed Frame 为 `uav1_map`，TF 应有 `uav1_map → uav1_odom → uav1_base_link`；第一段只是无全局锚点的单位变换，真机不加载仿真先验地图。飞控控制台查 `listener vehicle_visual_odometry`、`listener estimator_status`，再用 ULog 核对 EV 融合和创新；**收到视觉消息不等于 EKF2 已融合**。起飞前还要确认本地位置有效、`vio_health` 稳定为 `VALID`、地图持续更新、遥控接管可用。

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
