#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "${ROOT}/ros2_ws"
colcon build --symlink-install --packages-select \
  pac_common pac_perception pac_planning pac_robot pac_eoat pac_simulation pac_bringup
