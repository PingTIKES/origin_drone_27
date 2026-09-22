#!/usr/bin/env bash
# =============================================================
# OpenVINS 仿真测试一键脚本（桥接 + OpenVINS + 精度评估）
#
# 前置（终端 1，仿真默认 ALL_STEREO=1 即四机全挂 D435i）：
#   ./scripts/start_sim_4uav.sh
# 然后（终端 2）：
#   ./scripts/run_openvins_sim.sh                  # 桥接 4 机传感器，OpenVINS 默认只跑 uav1
#   VIO_UAVS="1 3" ./scripts/run_openvins_sim.sh   # 多机同跑 VIO（每实例约 1 核 CPU，量力而开）
# 单机模式（仿真用 ALL_STEREO=0 VIO_UAV=N 启动时，旧行为）：
#   ./scripts/run_openvins_sim.sh N
#
# 依赖（一次性安装，详见 README「OpenVINS 仿真测试」一节）：
#   - ros_gz_bridge：Humble+Garden 需用 OSRF 源装 ros-humble-ros-gzgarden
#   - OpenVINS：独立 colcon 工作空间编译 rpng/open_vins（master 支持 ROS2），
#     编译后 source 其 install/setup.bash（OV_WS 环境变量指向该工作空间）
#
# 流程：读分区环境 -> 依赖检查 -> 等相机话题 -> 桥接+OpenVINS（本脚本前台运行）
# 评估：另开终端 3 执行
#   ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=1
# =============================================================
set -e
UAV_ID="${1:-1}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OV_WS="${OV_WS:-$HOME/catkin_ws_ov}"

# ---- 1. Gazebo 分区环境（与仿真同一分区，否则桥接收不到话题）----
if [ -z "${GZ_PARTITION:-}" ]; then
    if [ -f /tmp/rm27_gz_env.sh ]; then
        # shellcheck disable=SC1091
        source /tmp/rm27_gz_env.sh
        echo "[vio] 已载入仿真分区：$GZ_PARTITION"
    else
        echo "[vio] 错误：未设置 GZ_PARTITION 且找不到 /tmp/rm27_gz_env.sh"
        echo "[vio]       请先在终端 1 启动仿真（./scripts/start_sim_4uav.sh）"
        exit 1
    fi
fi

# ---- 2. 依赖检查 ----
if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "[vio] 错误：未找到 ros_gz_bridge。"
    echo "[vio] Humble + Gazebo Garden 的组合需添加 OSRF 源后安装："
    echo "[vio]   sudo apt install ros-humble-ros-gzgarden"
    echo "[vio] （或按 README 说明源码编译 ros_gz，export GZ_VERSION=garden）"
    exit 1
fi
if ! ros2 pkg prefix ov_msckf >/dev/null 2>&1; then
    if [ -f "$OV_WS/install/setup.bash" ]; then
        echo "[vio] 载入 OpenVINS 工作空间：$OV_WS"
        # shellcheck disable=SC1091
        source "$OV_WS/install/setup.bash"
    fi
fi
if ! ros2 pkg prefix ov_msckf >/dev/null 2>&1; then
    echo "[vio] 错误：未找到 ov_msckf（OpenVINS）。编译并 source 后重试："
    echo "[vio]   mkdir -p $OV_WS/src && cd $OV_WS/src"
    echo "[vio]   git clone https://github.com/rpng/open_vins/"
    echo "[vio]   cd $OV_WS && colcon build"
    echo "[vio]   source $OV_WS/install/setup.bash   # 或 export OV_WS 指向你的工作空间"
    exit 1
fi
# 本工作空间（提供 uav_localization 的 launch/config）
if [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
else
    echo "[vio] 错误：工作空间未编译，请先 colcon build --packages-select uav_localization"
    exit 1
fi

# ---- 3. 等待仿真侧相机话题出现，并自动识别全机/单机模式 ----
echo "[vio] 等待相机话题（gz 侧）..."
if ! timeout 90 bash -c 'until gz topic -l 2>/dev/null | grep -qE "^(/uav1)?/vio_cam0/image$"; do sleep 2; done'; then
    echo "[vio] 错误：90 秒内未出现相机话题。"
    echo "[vio]       请确认仿真已启动且带 D435i（默认 ALL_STEREO=1；或单机模式 ALL_STEREO=0 VIO_UAV=$UAV_ID）。"
    exit 1
fi
if gz topic -l 2>/dev/null | grep -q "^/uav1/vio_cam0/image$"; then
    MODE="all"
else
    MODE="single"
fi
echo "[vio] 相机话题已就绪（模式：$MODE）"

# ---- 4. 启动桥接 + OpenVINS（前台运行，Ctrl+C 结束）----
if [ "$MODE" = "all" ]; then
    VIO_UAVS="${VIO_UAVS:-1}"
    echo "[vio] 全机 D435i：桥接 4 机全部传感器话题，OpenVINS 跑在 uav: $VIO_UAVS"
    echo "[vio] （想多机同跑：VIO_UAVS=\"1 2\" $0 —— 每实例约 1 核 CPU，量力而开）"
    echo "[vio] 起飞前请保持飞机静置 ~3 秒等待终端出现 initialized，再正常起飞。"
    echo "[vio] 评估另开终端：ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=<机号>"
    exec ros2 launch uav_localization vio_sim_test.launch.py \
        stereo_all:=true vio_uavs:="$VIO_UAVS"
else
    echo "[vio] 单机模式：启动 ros_gz_bridge + OpenVINS（uav$UAV_ID）..."
    echo "[vio] 起飞前请保持飞机静置 ~3 秒等待终端出现 initialized，再正常起飞。"
    echo "[vio] 评估另开终端：ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=$UAV_ID"
    exec ros2 launch uav_localization vio_sim_test.launch.py \
        uav_ns:="uav$UAV_ID" vio_model:="x500_stereo_$UAV_ID"
fi
