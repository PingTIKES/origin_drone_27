#!/usr/bin/env bash
# Download the pinned RMUC2025 visual/collision mesh used by Gazebo.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MESH="$ROOT_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl"
URL='https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/04e01a3678568667b480662700a88ed920dfcc6b/rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl'
if [[ -s "$MESH" ]]; then
    echo "[field] Mesh already exists: $MESH"
else
    mkdir -p "$(dirname "$MESH")"
    PARTIAL="$(mktemp "${MESH}.download.XXXXXX")"
    trap 'rm -f "$PARTIAL"' EXIT INT TERM
    curl -fL --retry 3 -o "$PARTIAL" "$URL"
    mv "$PARTIAL" "$MESH"
    trap - EXIT INT TERM
    echo "[field] Downloaded: $MESH"
fi
