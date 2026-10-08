#!/usr/bin/env bash
# V4.4 harness: V4.3 automatic weighing, then HDR50-22 pick & place onto the pallet.
# Requires Gazebo started with: ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

BOX="v43_scale_box_5kg"
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

echo ">>> [준비] V4.4 작업셀(그리퍼 포함) 실행 여부 확인 중..."
if ! timeout 10 ign topic -l | grep -Fx "/pac/gripper/state" >/dev/null; then
  echo "ERROR: Start Gazebo with hdr50_workcell_v4_4_pick.launch.py (gripper not found)." >&2
  exit 1
fi

echo ">>> ===== 1단계: 자동 계량 (V4.3) ====="
bash "$ROOT/scripts/run_auto_scale_v43.sh"

echo ">>> ===== 2단계: 로봇 Pick & Place (V4.4) ====="
BRIDGE_PID=""
NODE_PID=""
stop_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || return 0
  for _ in $(seq 50); do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}
cleanup() {
  trap - EXIT INT TERM
  stop_group "$NODE_PID"
  stop_group "$BRIDGE_PID"
}
trap cleanup EXIT
trap 'echo "[V4.4] Interrupted; cancelling robot motion and stopping child processes..."; exit 130' INT TERM

mkdir -p "$ROOT/logs"
BRIDGE_LOG="$ROOT/logs/v44_pick_bridge_$(date +%Y%m%d_%H%M%S).log"
echo ">>> [준비] 로봇용 ROS-Gazebo bridge 시작 중..."
setsid "$BRIDGE_BIN" \
  "/model/$BOX/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" \
  '/pac/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  '/pac/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
sleep 2
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "ERROR: Bridge exited. See $BRIDGE_LOG" >&2
  exit 1
fi

setsid python3 -u "$ROOT/scripts/run_pick_place_v44.py" &
NODE_PID=$!
set +e
wait "$NODE_PID"
RC=$?
set -e
exit "$RC"
