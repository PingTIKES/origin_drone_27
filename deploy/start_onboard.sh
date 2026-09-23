#!/usr/bin/env bash
# Start the same algorithm.launch.py used in simulation against a PX4 1.14.3 aircraft.
set -euo pipefail

UAV_ID="${1:-}"
[[ "$UAV_ID" =~ ^[1-4]$ ]] || { echo 'Usage: deploy/start_onboard.sh <uav-id 1-4>' >&2; exit 2; }
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OV_WS="${OV_WS:-$HOME/catkin_ws_ov}"
CALIBRATION_DIR="${CALIBRATION_DIR:-$ROOT_DIR/deploy/calibration/uav$UAV_ID}"
TARGET_SYSTEM="${TARGET_SYSTEM:-$UAV_ID}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyS1}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"
CYCLONEDDS_XML="${CYCLONEDDS_XML:-$ROOT_DIR/deploy/cyclonedds.xml}"

[[ -d "$CALIBRATION_DIR" ]] || {
    echo "Measured calibration directory missing: $CALIBRATION_DIR" >&2
    echo 'Set CALIBRATION_DIR to a directory containing estimator_config.yaml and body.yaml.' >&2
    exit 1
}
[[ -f "$OV_WS/install/setup.bash" ]] || { echo "OpenVINS workspace is not built: $OV_WS" >&2; exit 1; }
[[ -f "$ROOT_DIR/install/setup.bash" ]] || { echo 'Build this workspace first.' >&2; exit 1; }
[[ -e "$SERIAL_DEV" ]] || { echo "Serial device missing: $SERIAL_DEV" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$OV_WS/install/setup.bash"
source "$ROOT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML"

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

MicroXRCEAgent serial --dev "$SERIAL_DEV" -b "$SERIAL_BAUD" & PIDS+=("$!")
sleep 3
ros2 launch uav_bringup algorithm.launch.py \
    sim:=false uav_id:="$UAV_ID" target_system:="$TARGET_SYSTEM" \
    calibration_dir:="$CALIBRATION_DIR" depth_source:=hardware rviz:=false &
PIDS+=("$!")

echo "[onboard] uav$UAV_ID algorithm started. It will not arm automatically."
echo "[onboard] Verify VIO, depth, local map and PX4 fusion before calling /uav$UAV_ID/start_mission."
wait
