#!/usr/bin/env bash
# =============================================================================
# RM2027 四机集群 · 机载板一键启动脚本（LubanCat-3 实机）
#
# 用法：  ./deploy/start_onboard.sh <机号 1-4>
#   机号必须与该板静态 IP 对应：192.168.10.11→1，.12→2，.13→3，.14→4
#
# 环境变量（可选）：
#   AUTO_TAKEOFF=0   起 offboard 但不自动起飞（首飞默认，手动解锁试链路）
#   AUTO_TAKEOFF=1   链路验证通过后允许自动起飞
#   SERIAL_DEV=/dev/ttyS1   飞控串口（默认 ttyS1，接线改了再调）
#   SERIAL_BAUD=921600      飞控串口波特率
#   CYCLONEDDS_XML=路径     cyclonedds.xml 位置（默认本脚本同目录）
#
# 启动内容（全部后台托管，Ctrl+C 一键清场）：
#   1) MicroXRCEAgent serial —— 板子 ↔ PX4 飞控的 uXRCE-DDS 桥
#   2) uav_bringup.launch.py uav_id:=N —— offboard 控制（+按 uav_id 的机上节点）
#
# 前置条件（详见 deploy/实机网络配置.md）：
#   - 板上 setup_env.sh 只跑第 1、4、6 步（不装 Gazebo/PX4 SITL）；
#   - 静态 IP / hosts / chrony 已配好，能 ping 通 192.168.10.1；
#   - sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-realsense2-camera；
#   - 飞控 PX4 参数 EKF2_EV_CTRL=15（外部视觉融合）；
#   - 本工作空间已在板上 colcon build 出 install/。
#
# 注意：OpenVINS / 感知 / 避障节点负载重，建议另开终端单独起，
#       确认图像 hz 正常后再纳入本脚本。
# =============================================================================
set -euo pipefail

# ---------- 参数 ----------
UAV_ID="${1:-}"
if [[ -z "$UAV_ID" || ! "$UAV_ID" =~ ^[1-4]$ ]]; then
    echo "用法: $0 <机号 1-4>   （机号 = 板子 IP 末位 - 10）"
    exit 1
fi

AUTO_TAKEOFF="${AUTO_TAKEOFF:-0}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyS1}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
CYCLONEDDS_XML="${CYCLONEDDS_XML:-$SCRIPT_DIR/cyclonedds.xml}"

# ---------- DDS 环境（五台机器统一，见 deploy/实机网络配置.md 第 4 节） ----------
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML"

if [[ ! -f "$CYCLONEDDS_XML" ]]; then
    echo "[start_onboard] 找不到 cyclonedds.xml：$CYCLONEDDS_XML"
    echo "  从 deploy/ 拷贝一份，或用 CYCLONEDDS_XML=路径 指定。"
    exit 1
fi

# ---------- ROS 环境 ----------
source /opt/ros/humble/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[start_onboard] 未找到 $WS_DIR/install/setup.bash"
    echo "  请先在板上 colcon build（setup_env.sh 第 4 步）。"
    exit 1
fi

# ---------- 清场 ----------
PIDS=()
cleanup() {
    echo
    echo "[start_onboard] 正在停止 uav$UAV_ID 机上节点..."
    for pid in "${PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[start_onboard] 已清场。飞机若仍在 Offboard 悬停，请用遥控器接管降落。"
}
trap cleanup INT TERM

echo "=============================================================="
echo "  RM2027 机载板启动 · uav$UAV_ID"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "  飞控串口 $SERIAL_DEV @ $SERIAL_BAUD"
echo "  AUTO_TAKEOFF=$AUTO_TAKEOFF（0=不自动起飞，首飞保持 0）"
echo "=============================================================="

# ---------- 1) MicroXRCEAgent：板子 ↔ 飞控 ----------
if [[ ! -e "$SERIAL_DEV" ]]; then
    echo "[start_onboard] 警告：串口 $SERIAL_DEV 不存在，检查飞控接线或 SERIAL_DEV。"
fi
MicroXRCEAgent serial --dev "$SERIAL_DEV" -b "$SERIAL_BAUD" &
PIDS+=($!)
echo "[start_onboard] MicroXRCEAgent 已起 (pid $!)。等飞控上线 5 秒..."
sleep 5

# ---------- 2) offboard 控制 ----------
# uav_bringup.launch.py 内部按 uav_id 起该机 offboard 实例；
# AUTO_TAKEOFF=0 时只待命，手动解锁后也不自动给起飞航点。
ros2 launch uav_bringup uav_bringup.launch.py \
    uav_id:="$UAV_ID" auto_takeoff:="$([ "$AUTO_TAKEOFF" = "1" ] && echo True || echo False)" &
PIDS+=($!)
echo "[start_onboard] uav_bringup (uav_id=$UAV_ID) 已起 (pid $!)。"

echo
echo "  机上链路自检（另开终端）："
echo "    ros2 topic echo /uav$UAV_ID/state      # 飞控心跳"
echo "    ros2 topic hz  /uav$UAV_ID/odometry    # 位姿流"
echo "  地面站应能 echo 到以上话题；收不到先查 DDS 环境变量与 Peers。"
echo "  Ctrl+C 停止全部机上节点。"
echo

wait
