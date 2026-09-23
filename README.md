# origin_drone_27

RoboMaster 2027 无人机视觉定位、在线深度建图、局部规划和四机协同工作空间。仿真与真机共用 `algorithm.launch.py`，统一按 **PX4 1.14.3** 和 `px4_msgs release/1.14` 构建。

## 唯一运行链路

```text
双目图像 + 相机 IMU -> OpenVINS -> PX4 EKF2
双目软件深度 / 真机硬件深度 -> 机体系点云 -> 滚动局部地图
任务目标 -> local_navigator -> Offboard
四机任务 -> swarm_coordinator -> swarm_agent -> 每机 local_navigator
```

Gazebo 只提供传感器和动力学。运行链路不读取真值位姿、理想深度或预制占据地图。尚未完成的 RGB 目标检测接口保留在 `yolo_detector.py`，当前不会生成虚假检测结果。

## 安装

仓库请放在纯英文路径。Ubuntu 22.04、ROS 2 Humble：

```bash
git clone https://github.com/PingTIKES/origin_drone_27.git ~/origin_drone_27
cd ~/origin_drone_27
./setup_env.sh sim
```

`setup_env.sh sim` 只接受 MicoAir PX4 1.14.3 固定提交 `08310a5e8ac64d02edb41523460e7dc267298deb`，安装 `px4_msgs release/1.14`、OpenVINS 和本工作空间。机载板使用：

```bash
./setup_env.sh onboard
```

机载模式不下载或编译 PX4 SITL。相机实测标定、飞控参数和串口配置见 [算法与真机流程](ALGORITHM_PIPELINE.md) 与 [PX4 1.14.3 真机配置](deploy/px4_1_14_3_vision.md)。

## 单机仿真

终端 A：

```bash
cd ~/origin_drone_27
PX4_DIR=~/PX4-Autopilot-1.14.3 ./scripts/start_algorithm_sim.sh 1
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
source /tmp/origin_drone_27_gz_env.sh
PYTHONNOUSERSITE=1 ros2 launch uav_bringup algorithm.launch.py \
  sim:=true uav_id:=1 rviz:=true
```

确认 `/uav1/vio_health` 为 `VALID`，`/uav1/obstacles`、`/uav1/local_map` 和 `/tf` 持续发布，且 PX4 已融合视觉位置后，再请求起飞：

```bash
ros2 service call /uav1/start_mission std_srvs/srv/Trigger '{}'
```

## 四机仿真与真机

四机启动见 [SWARM_SIMULATION.md](SWARM_SIMULATION.md)。真机在完成每架相机标定和 PX4 参数核对后运行：

```bash
CALIBRATION_DIR=~/calibration/uav1 TARGET_SYSTEM=1 \
  ./deploy/start_onboard.sh 1
```

该脚本不会自动解锁。

## 验证

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 python3 tools/test_algorithm_stack.py
```

离线测试不能替代 Gazebo、DDS、真机台架和飞行验收。当前真实 RGB 目标识别、真实相机标定、多机坐标对齐和板端实时性能仍需完成。
