#!/usr/bin/env bash
# Build every ROS 2 package of the monorepo (team packages + Hyundai submodules).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/external/hyundai_robotics/hdr_description/package.xml" ]] || \
  { echo "Hyundai submodules missing: git submodule update --init --recursive" >&2; exit 1; }
cd "${ROOT}/ros2_ws"
colcon build --symlink-install "$@"
