# origin_drone_27

RoboMaster 2027 无人机视觉定位、局部建图与规划的 ROS 2 Humble 工作空间。仿真与真机统一按 **PX4 1.14.3** 配置：仿真使用 MicoAir 公开的 1.14.3 源码构建 SITL，真机使用 **MicoAir743v2-AIO-35A、PX4 1.14.3 固件**。真机所刷固件的具体构建来源仍需核对，不能仅凭版本号认定二进制或 DDS 消息完全一致。Gazebo 提供相机、IMU 和动力学，算法链不读取仿真真值定位、理想深度或预置场地栅格。单机入口为 `algorithm.launch.py`，四机入口为 `algorithm_swarm_sim.launch.py`。

> 状态：PX4 1.14.3 已完成 SITL 编译与单机启动检查；视觉定位、解锁、起飞与短时悬停的实测记录来自旧版 1.14.2，需在 1.14.3 上重验。目标点导航、四机协同、正常降落、RK3566 板端实时性及真机标定仍未验收。

## 文档入口

| 需求 | 文档与入口 |
| --- | --- |
| 单机视觉算法与真机标定 | [ALGORITHM_PIPELINE.md](ALGORITHM_PIPELINE.md) · `scripts/start_algorithm_sim.sh` + `algorithm.launch.py` |
| 四机混合集群 | [SWARM_SIMULATION.md](SWARM_SIMULATION.md) · `algorithm_swarm_sim.launch.py` |
| 相机几何与上机前检查 | [CAMERA_FIX_AND_BRINGUP.md](CAMERA_FIX_AND_BRINGUP.md) |
| 历史 GPS/真值检测仿真 | [LEGACY_SIMULATION.md](LEGACY_SIMULATION.md) · `start_sim_4uav.sh` + `run_swarm.sh` |

历史入口用于对照，含先验地图和真值目标检测，不能用于证明当前视觉算法闭环。`setup_env.sh` 服务于历史 **PX4 1.15.4 / px4_msgs release/1.15** 环境；运行新算法时不要执行它来安装或升级依赖。

## Ubuntu 算法仿真速览

环境：Ubuntu 22.04 x86_64、ROS 2 Humble、Gazebo 与匹配的 `ros_gz_bridge`、MicoAir PX4 **1.14.3**、`px4_msgs` **release/1.14**、OpenVINS、MicroXRCEAgent。建议至少 16 GB 内存；四机渲染与编译的实际资源需求取决于机器。**ROS 接口生成器在本机的中文目录路径构建失败**，因此工作空间请放在纯英文路径。命令从仓库根目录执行，以下用 `~/origin_drone_27` 举例；如果克隆到别处，使用实际路径。

```bash
git clone https://github.com/PingTIKES/origin_drone_27.git ~/origin_drone_27
cd ~/origin_drone_27
source /opt/ros/humble/setup.bash

# 准备与 PX4 1.14 系列对应的 px4_msgs；不要混用 release/1.15。
mkdir -p third_party
git clone --depth 1 --branch release/1.14 https://github.com/PX4/px4_msgs.git third_party/px4_msgs
ln -s ../third_party/px4_msgs src/px4_msgs
rosdep install --from-paths src --ignore-src -r -y

# MicoAir PX4 1.14.3 和 OpenVINS 源码准备好后，按 ALGORITHM_PIPELINE.md 执行受版本检查的补丁。
python3 tools/prepare_algorithm_sim.py \
  --px4 ~/PX4-Autopilot-1.14.3 --openvins ~/catkin_ws_ov/src/open_vins
# 重新编译 PX4 SITL、OpenVINS；之后在本工作空间：
colcon build --symlink-install
source install/setup.bash
PYTHONNOUSERSITE=1 python3 tools/test_algorithm_stack.py
```

`prepare_algorithm_sim.py` 只接受文档固定的 MicoAir PX4 **1.14.3 源码提交**并为修改文件保留 `.rm27-backup`；它不会下载依赖或替你编译。PX4、OpenVINS、Agent 的准备与构建顺序见 [ALGORITHM_PIPELINE.md 的 Ubuntu 步骤](ALGORITHM_PIPELINE.md#ubuntu先运行真正的算法仿真)。先确认这些依赖和仿真网格已安装，再启动：

```bash
# 终端 A：先用单机验证；去掉末尾的 1 即启动默认四机
cd ~/origin_drone_27
PX4_DIR=~/PX4-Autopilot-1.14.3 ./scripts/start_algorithm_sim.sh 1

# 终端 B：单机视觉链；仿真每次重启后都要重新启动本终端的节点
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
source /tmp/rm27_gz_env.sh
ros2 launch uav_bringup algorithm.launch.py sim:=true uav_id:=1 rviz:=true
```

确认 `/uav1/vio_health` 为 `VALID`、PX4 视觉融合和本地位置有效后，按 [单机操作与验收步骤](ALGORITHM_PIPELINE.md#ubuntu先运行真正的算法仿真)请求起飞。四机需在终端 B 改用 `algorithm_swarm_sim.launch.py`，并按 [四机启动说明](SWARM_SIMULATION.md#启动)逐机确认视觉状态。不要与 `run_swarm.sh`、`goal_nav.launch.py` 或 `WITH_RVIZ=1` 同时运行，它们使用历史控制入口。

## 当前边界

- 独立 RGB 目标识别仍是桩代码；四机默认只做分区搜索与返航，不会凭空产生目标检测结果。
- 真机必须提供实际相机内外参、机体安装变换和多机坐标对齐；示例配置不能直接用于飞行。
- 真机飞控使用 `micoair_h743-v2` 固件目标；烧录版本、DDS 消息与电机协议应按 [真机视觉配置说明](deploy/px4_1_14_3_vision.md) 核对，不要把仿真补丁用于真机。
- 离线测试使用 ROS 消息桩，不能替代 DDS/Gazebo 集成测试或板端性能测试。

MicoAir PX4 **1.14.3 SITL 已编译成功**，OpenVINS 4 个包与本工作空间 10 个包构建通过，36 项离线测试通过。在 Gazebo `default` 世界完成单机启动检查：模型创建成功，MicroXRCEAgent 建立连接，ROS 可见 `/px4_1/fmu/out/vehicle_status` 与 `/px4_1/fmu/out/vehicle_local_position`。这是版本与启动链路检查，尚未验证 1.14.3 的视觉融合、解锁、起飞、导航或降落。此前使用 **PX4 1.14.2** 的单机 SITL 曾从约 0.187 m 起飞至约 2.106 m 短时悬停，但目标导航报告过 `HOLD_NO_PATH`/`HOLD_BLOCKED_START`，随后 VIO 跳变触发安全退出 Offboard；该记录不能作为 **1.14.3** 的飞行验收结果。真机、四机协同和板端实时性也尚未验收。

若 `import cv2` 报 NumPy 2 与 OpenCV ABI 不兼容，先检查用户级 Python 包是否覆盖了 Ubuntu 的 `python3-numpy`；本机使用 `PYTHONNOUSERSITE=1` 后恢复兼容。不要在同一 ROS 2 系统解释器中混装不兼容的 NumPy/OpenCV 版本。

若相机有图像而 VIO 始终 `INVALID`，先用 `source /tmp/rm27_gz_env.sh; gz model -m x500_stereo_uav1_1 -p` 确认模型出生在预期停机坪，再检查 `/uav1/cam0/image_raw` 与 `/uav1/odomimu`。PX4 1.14 系列只在 `PX4_GZ_MODEL` 分支应用 `PX4_GZ_MODEL_POSE`；旧脚本曾把飞机生在世界原点，画面低纹理并导致 VIO 无效。当前启动脚本已修正，但需在 1.14.3 上重验。

若深度图存在而导航一直显示 `HOLD_MAP_STALE`，依次检查 `/uav1/d435i/depth/camera_info`、`/uav1/obstacles`、`/uav1/local_map` 是否持续发布；仿真重启后须重启算法 launch。出现 `HOLD_NO_PATH` 或 `HOLD_BLOCKED_START` 时不要把服务响应或目标消息当作移动成功，需先检查局部地图与 Gazebo 实际位姿。

详细的数据流、故障处理和实机限制分别见上表文档。
