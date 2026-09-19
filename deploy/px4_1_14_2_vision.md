# PX4 1.14.2 视觉定位配置

面向本项目 `vio_to_px4.py`：输出为机体原点、世界固定 FRD（航向任意）、
机体 FRD 速度。不要重复补偿相机杆臂，也不要假定 VIO 的初始航向就是北。

以下在地面台架核对后通过 QGC 参数设置，重启再验证；不是自动刷写脚本：

| 参数 | 基线 | 意义 |
|---|---:|---|
| EKF2_GPS_CTRL | 0 | 不以 GPS 支撑算法定位 |
| EKF2_EV_CTRL | 15 | 视觉位置、速度、航向融合 |
| EKF2_HGT_REF | 3 | 视觉高度参考；保留可用气压计融合提供独立高度来源 |
| EKF2_EV_POS_X/Y/Z | 0 | 本桥已经转换到机体原点 |
| EKF2_EV_NOISE_MD | 0 | 使用消息协方差，PX4 参数作为下限 |
| COM_OF_LOSS_T | 0.5 | Offboard 失联时限起点，需故障注入验证 |
| COM_OBL_RC_ACT | 4 | Offboard 丢失进入 Land |
| COM_POSCTL_NAVL | 1 | 定位失效走 Land/Descend，而非承诺一定能定点降落 |
| MPC_XY_VEL_MAX | 0.5 | 第一阶段低速测试上限 m/s |
| MPC_XY_CRUISE | 0.5 | 巡航速度基线 m/s |

EKF2_EV_DELAY、创新门限、噪声下限需根据实际日志调整；不以放宽拒绝门限掩盖标定错误。
真机磁罗盘配置按机体干扰和 yaw 源验证，不直接照搬 SITL 的 EKF2_MAG_TYPE=5。
真机必须保留遥控接管；此配置不对跌落或定位完全失效时的着陆结果作保证。

DDS：每板设置唯一 UXRCE_DDS_KEY，并给 uxrce_dds_client 的命名空间指定 px4_N；
algorithm.launch 的 target_system 必须等于这台飞控的 MAV_SYS_ID。
px4_msgs 使用 release/1.14 对应消息定义；不要拿 main 分支的新消息与 1.14.2 混用。
板端 MicroXRCEAgent serial 接口路径和波特率按接线设置。

时间：真机保持 1.14.2 默认 XRCE 时间同步，ROS 使用系统时钟。相机采集时间必须转换到
同一系统时基。只有 SITL 使用本项目的 RM27_SIM_CLOCK 补丁和 ROS use_sim_time。
不要向 1.14.2 写入它不存在的 UXRCE_DDS_SYNCT 参数。

起飞前在 PX4 控制台核对 `listener vehicle_visual_odometry`、`listener estimator_status`，
再用 ULog 核对 EV 融合状态/创新，确认并非只收到消息而没有融合。
对每机验证：相机断流、停 VIO、停导航、拔除机载通信、遥控接管、VIO 重置。
桥对图像断流、旧数据、协方差异常、跳变进行拒绝；飞控 Offboard 节点故障后锁定，
不自动重新解锁。恢复需处理原因并在地面重启节点。

依据：
- [PX4 v1.14 参数表](https://docs.px4.io/v1.14/en/advanced_config/parameter_reference)
- [1.14.2 VehicleOdometry](https://github.com/PX4/PX4-Autopilot/blob/v1.14.2/msg/VehicleOdometry.msg)
- [1.14.2 DDS 话题](https://github.com/PX4/PX4-Autopilot/blob/v1.14.2/src/modules/uxrce_dds_client/dds_topics.yaml)
