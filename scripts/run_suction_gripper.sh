#!/usr/bin/env bash
# Start the V4.4 suction-cup gripper: ROS-Gazebo bridge + vacuum logic node.
# Requires Gazebo started with hdr50_workcell_v4_4_pick.launch.py. Ctrl+C stops both.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

echo ">>> [흡착 준비] V4.4 흡착 그리퍼 확인 중..."
if ! timeout 10 ign topic -l | grep -Fx "/pac/gripper/state" >/dev/null; then
  echo "ERROR: Start Gazebo with hdr50_workcell_v4_4_pick.launch.py (gripper not found)." >&2
  exit 1
fi

BRIDGE_PID=""
NODE_PID=""
stop_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || return 0
  for _ in $(seq 50); do kill -0 -- "-$pid" 2>/dev/null || return 0; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || true
}
cleanup() {
  trap - EXIT INT TERM
  stop_group "$NODE_PID"
  stop_group "$BRIDGE_PID"
}
trap cleanup EXIT
trap 'echo "[suction] stopping..."; exit 130' INT TERM

mkdir -p "$ROOT/logs"
echo ">>> [흡착 준비] 흡착 그리퍼 bridge·제어 노드 시작 중..."
setsid "$BRIDGE_BIN" \
  '/pac/suction/touched@std_msgs/msg/Bool[ignition.msgs.Boolean' \
  '/pac/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  '/pac/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  > "$ROOT/logs/v44_suction_bridge_$(date +%Y%m%d_%H%M%S).log" 2>&1 &
BRIDGE_PID=$!
setsid python3 -u "$ROOT/scripts/suction_gripper_node.py" &
NODE_PID=$!
wait "$NODE_PID"
