# 相机 90° 偏差诊断与复测

> 本文保留首次相机诊断记录。后续已实现 VIO 桥、深度适配、标定导入和局部规划等代码，
> 当前启动和剩余验收项以 [ALGORITHM_PIPELINE.md](ALGORITHM_PIPELINE.md) 为准；
> 下文“尚未实现”的迁移项描述的是首次检查时的状态。

检查基线：main，2b9e4df2f7274e0be4b18a23dd4d2279361cd885。
本次完成代码检查和离线几何测试，未运行 Ubuntu ROS2/Gazebo 或连接真机。

## 原因与修改

`worlds/models/d435i/model.sdf` 四个成像传感器的 rpy 原为
`-1.5707963 0 -1.5707963`。Gazebo 相机安装系为 X 前/Y 左/Z 上；
这组旋转使前视轴转向机体 -Y，图像上方向转向机体 +X，所以画面侧倒且取景偏向侧面。
ROS 光学系的 X 右/Y 下/Z 前是图像反投影约定，不能直接作为 Gazebo sensor 安装旋转。

修改：

- 四个 sensor 保留原位置，将 rpy 全改为 `0 0 0`。
- OpenVINS 仿真 `T_imu_cam` 的旋转改为光学系到机体 FLU：
  `[[0,0,1],[-1,0,0],[0,-1,0]]`，保留安装平移。
- 深度节点的平移改为 `[0.17, 0.025, -0.06]`。Gazebo base_link 本来就是 FLU，
  原代码误当 NED，再翻转 Y/Z，导致平移符号错误。

依据：Gazebo 官方坐标说明
<https://gazebosim.org/api/sdformat/16/classsdf_1_1SDF__VERSION__NAMESPACE_1_1Imu.html>
（CustomRpy 中明确区分默认相机朝向与 ROS optical frame）。

无需修改 RViz Image 显示、旋转像素或旋转整个赛场。图像显示不依赖 map TF 来旋转像素。
修改模型模板后必须重新生成仿真模型；只重开 RViz 无效。

## Ubuntu 应用与验证

关闭本轮仿真、goal_nav、OpenVINS 等终端里的运行进程。在项目根目录应用补丁后：

```bash
source /opt/ros/humble/setup.bash
python3 tools/check_camera_geometry.py
colcon build --packages-select uav_perception uav_localization
source install/setup.bash
WITH_RVIZ=1 ./scripts/start_sim_4uav.sh
```

测试工具需 numpy；补丁仅修改三个运行文件时，可另行复制 tools/check_camera_geometry.py。
脚本默认全机相机模式，启动时自动从模板重建 worlds/models/.gen；不要手改生成物。
WITH_RVIZ 会在仿真中启动自动起飞和导航。默认只有 uav1 桥接深度并接入避障。

验收：

1. 飞机水平悬停，RGB 地平线应水平，前方物体应出现在前视画面。
2. Gazebo 原始图与 RViz 的同一路图像方向一致。
3. 查看 `/uav1/obstacles` 的机体系点：正前方物体 X>0，左侧 Y>0，下方 Z<0
   （考虑相机安装偏移）；RViz 可临时将 Fixed Frame 设为 uav1 检查局部几何。
4. 再检查 map 下的点云与场景关系；当前 pose_tf_publisher 存在独立问题：
   位置使用 `(N,E,-D)`，却把它作为普通右手坐标系并用 `yaw=-heading`，且仅使用航向，
   忽略 roll/pitch。这不是二维图像侧倒的原因，但会影响全局点云对齐。
   后续应统一 map 为 ENU 或 NWU，同步修改地图、goal、轨迹、出生点和全姿态 TF，
   不能只在 TF 中孤立地翻转一个符号。本次未扩展修改整个导航坐标接口。
5. VIO：另开终端运行 `./scripts/run_openvins_sim.sh`，先静置初始化再飞行；
   `ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=1` 对比轨迹。
   当前比较对象为 PX4 EKF 本地位置，是参考轨迹而非严格 Gazebo 真值。
   每次重启仿真都重启 OpenVINS 桥，以继承新的 GZ_PARTITION。

若手动另起 goal_nav，先 source /tmp/rm27_gz_env.sh。
运行 run_swarm 与 goal_nav 应二选一，避免两套任务同时发布航点。

## 项目链路与真机迁移边界

仿真：Gazebo 传感器/动力学 → PX4 SITL → MicroXRCEAgent → offboard_control。
打点导航：RViz goal → goal_planner → VFH（仅桥接相机的无人机）→ offboard。
感知：深度图 → stereo_depth_node → 机体系障碍点云 → VFH。
集群任务：swarm_coordinator 负责搜索/汇聚/返航，collision_monitor 监控机间距离。
OpenVINS 仿真链独立做轨迹评估，目前并未形成 VIO 反馈 PX4 的无 GPS 闭环。

目标硬件按用户实际方案：RK3576 用于算法调试，RK3566 为最终平台，相机为自拼 D435i 方案。
迁移顺序：

1. 确认相机组成、实际基线、左右目顺序、是否保留 D4 深度处理板，以及 VIO 使用相机
   IMU 还是飞控 IMU。若没有 D4，当前 stereo_depth_node 不会从双目计算深度，需补上
   整流/立体匹配和深度输出。
2. 固定机械安装，标定双目内参/畸变/相对外参，再标定所选 IMU 的相机外参、时间偏移和
   IMU 噪声。原始图与整流图各使用对应的参数；不复制仿真 50 mm 基线或理想内参。
3. 单板录包验证时间戳、双目同步、帧率和 USB 带宽。先离线 VIO 轨迹验证，再检查深度
   尺度和光学系到机体的完整旋转/平移。深度节点当前只支持 32FC1 米制数据；实际驱动若
   输出 16UC1，必须按设备真实 depth scale 转换，不能只 remap 话题。
4. 实机 OpenVINS launch/config 需按实际 OpenVINS 版本接通：当前内参为零、外参占位；
   仿真 launch 使用 config_path + Kalibr 文件，实机 launch 仍使用另一套参数结构。
   不能把 README 的启动命令等同于已完成实机支持。
5. 实现 VIO → vehicle_visual_odometry 桥，验证世界系、机体系、时间戳、协方差和重置
   语义，然后按飞控固件版本配置视觉融合。目前该桥在 openvins.launch.py 中仅有注释。
6. 将检测桩替换为真实 RKNN 推理。分别针对 RK3576/RK3566 导出并验证模型和运行时；
   在最终 RK3566 上测 VIO+深度+检测同时运行的帧率、时延、丢帧和温度。
7. 单机台架验证遥控接管、定位失效和通信失效处理后，再进行单机飞行、最后扩展四机。
   实机启动文件还需整理：uav_bringup 未将 auto_takeoff launch 参数接入节点，且每板都
   启动集群调度和 collision_monitor，会造成多实例竞争；应让全局节点只运行一份。

这些是代码中已识别的迁移工作项，不代表本次已完成真机适配或飞行验证。
