# 混合集群算法仿真

该入口运行四套与真机一致的定位、软件双目深度、滚动地图、局部规划和
Offboard 控制。Gazebo 只提供环境、相机、IMU 和动力学，不向集群算法提供
真值位置、理想深度或真值目标。

## 数据流

```text
swarm_coordinator（任务级，5 Hz）
  -> /swarm/command（公共 NED，带 mission_id / command_id / TTL）
  -> 每机 swarm_agent
       -> /uavN/waypoint_in -> local_navigator -> /uavN/waypoint -> PX4
       -> /swarm/uav_state（公共位置/速度/健康度，10 Hz）
       -> /swarm/trajectory_intent（未来 2 s 意图，5 Hz）
       -> 本机预测避碰 -> /uavN/safety_waypoint
       -> /swarm/ack
```

静态障碍由每机 D435i 双目、滚动局部栅格和 A* 处理。机间动态避碰不依赖
前视相机看到队友，而是使用通信的位置、速度和短时轨迹意图。状态过期时，
已经见过该成员的飞机发布当前位置作为安全航点并暂停前进。

## 编译与离线测试

```bash
source /opt/ros/humble/setup.bash
cd ~/origin_drone_27
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 tools/test_algorithm_stack.py
```

## 启动

终端 A 只启动 Gazebo、四个 PX4 SITL 和 MicroXRCEAgent：

```bash
cd ~/origin_drone_27
PX4_DIR=~/PX4-Autopilot ./scripts/start_algorithm_sim.sh
```

终端 B 启动四套算法和一个任务协调器：

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/origin_drone_27/install/setup.bash
source /tmp/rm27_gz_env.sh
ros2 launch uav_bringup algorithm_swarm_sim.launch.py rviz:=true
```

确认四架 VIO 都为 `VALID` 后，分别允许起飞：

```bash
for i in 1 2 3 4; do
  ros2 service call /uav${i}/start_mission std_srvs/srv/Trigger '{}'
done
```

## 检查话题

```bash
ros2 topic hz /swarm/uav_state
ros2 topic hz /swarm/trajectory_intent
ros2 topic echo /swarm/ack
ros2 topic echo /swarm/state
ros2 topic echo /uav1/navigation_state
```

预期每架状态 10 Hz，总 `/swarm/uav_state` 约 40 Hz；每架意图 5 Hz，总计约
20 Hz。协调器进入 `SEARCH` 后发布带有效期的公共目标，每机 agent 转成本机
坐标并持续刷新 `waypoint_in`。

## 故障注入

1. 停止一架 `swarm_agent`，其他已经收到过该机状态的 agent 应停止前进。
2. 给 `/swarm/command` 发布过期命令，应收到 `expired` ACK，且航点不改变。
3. 制造两机迎面目标，观察 `/uavN/navigation_state` 出现 `SAFETY_` 前缀。
4. 重启一架 PX4 或触发本地坐标 reset，agent 增加 `frame_epoch` 并等待新命令。

## 当前边界

- 仿真公共坐标使用已知出生点和零航向偏置；真机必须测量每机平移和航向。
- 当前避碰是适合 RK3566 的采样式互惠速度选择，不是完整三维轨迹优化器。
- 任务协调器仍是单实例；失去协调器后，本机避障和状态广播继续运行，但不会产生新任务。
- RGB 目标检测仍未实现，因此默认任务只执行分区搜索并在完成后返航。
