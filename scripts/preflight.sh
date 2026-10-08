#!/usr/bin/env bash
set -eo pipefail
echo '===== PAC 2026 preflight (target robot HDR50-22) ====='
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; echo '[OK] ROS 2 Humble setup'; else echo '[FAIL] ROS setup missing'; fi
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
python3 --version
git --version
for pkg in ros_gz_sim ros_gz_bridge; do if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then echo "[OK] ROS package: $pkg"; else echo "[WARN] missing: $pkg"; fi; done
echo 'Rules:'
echo '- do NOT use the 50 kg nominal payload as a direct box limit (gripper + TCP offset count)'
echo '- do NOT invent kinematics or inertias for robots without a verified description'
echo '- verify robot description before MoveIt/Gazebo robot integration'
