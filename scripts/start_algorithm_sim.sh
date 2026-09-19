#!/usr/bin/env bash
# Gazebo/PX4 environment ONLY. Start algorithm.launch.py in another terminal.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SOURCE="$PX4_DIR/src/modules/uxrce_dds_client/uxrce_dds_client.cpp"
PX4_EXEC="$PX4_DIR/build/px4_sitl_default/bin/px4"
if ! grep -q RM27_SIM_CLOCK "$SOURCE" || [[ ! -f "$PX4_EXEC" || "$SOURCE" -nt "$PX4_EXEC" ]]; then
    echo 'Run tools/prepare_algorithm_sim.py and rebuild PX4 v1.14.2 SITL first.' >&2
    exit 1
fi
export RM27_SIM_CLOCK=1 ALL_STEREO=1 WITH_RVIZ=0
exec "$ROOT_DIR/scripts/start_sim_4uav.sh"
