#!/usr/bin/env bash
set -eo pipefail
echo '===== PAC 2026 HDP160-31 preflight ====='
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; echo '[OK] ROS 2 Humble setup'; else echo '[FAIL] ROS setup missing'; fi
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
python3 --version
git --version
for pkg in ros_gz_sim ros_gz_bridge; do if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then echo "[OK] ROS package: $pkg"; else echo "[WARN] missing: $pkg"; fi; done
echo 'Rules:'
echo '- do NOT invent HDP160-31 kinematics or inertias'
echo '- do NOT use 160 kg nominal payload as a direct box limit'
echo '- verify robot description before MoveIt/Gazebo robot integration'
