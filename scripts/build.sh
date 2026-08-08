#!/usr/bin/env bash
# Build ROS2 workspace when ROS2 is available.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. On Windows PC use mock stack; on Ubuntu/Pi install ROS2 Jazzy."
  exit 0
fi
cd "$ROOT/ros2_ws"
colcon build --symlink-install
echo "Build complete. source install/setup.bash"
