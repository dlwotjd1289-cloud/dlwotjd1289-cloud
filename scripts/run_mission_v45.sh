#!/usr/bin/env bash
# V4.5 harness: V4.3 weighing -> team planner (pac-mission1-shared) -> HDR50-22 places
# the box at the planner's ranked candidate -> ExecutionResult JSON for the State Manager.
# Requires Gazebo started with: ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

BOX="v43_scale_box_5kg"
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$ROOT/logs/v45_mission_$STAMP"
mkdir -p "$RUN_DIR"

echo ">>> [준비] V4.4 작업셀(그리퍼 포함) 실행 여부 확인 중..."
if ! timeout 10 ign topic -l | grep -Fx "/pac/gripper/state" >/dev/null; then
  echo "ERROR: Start Gazebo with hdr50_workcell_v4_4_pick.launch.py (gripper not found)." >&2
  exit 1
fi

echo ">>> ===== 1단계: 자동 계량 (V4.3) ====="
bash "$ROOT/scripts/run_auto_scale_v43.sh" 2>&1 | tee "$RUN_DIR/v43_scale.log"
MEASURED_KG="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import mission_bridge_v45 as MB; print(MB.parse_scale_pass(open(sys.argv[2]).read()))' \
  "$ROOT/scripts" "$RUN_DIR/v43_scale.log")"
echo ">>> 계량값 ${MEASURED_KG} kg 을 BoxState.weight_kg 로 사용"

echo ">>> ===== 2단계: 적재 위치 계획 (태현 후보 + 동한 순위) ====="
python3 -u "$ROOT/scripts/plan_placement_v45.py" \
  --measured-kg "$MEASURED_KG" --out "$RUN_DIR/placement.json"

echo ">>> ===== 3단계: 로봇 Pick & Place (계획 위치) ====="
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
trap 'echo "[V4.5] Interrupted; cancelling robot motion and stopping child processes..."; exit 130' INT TERM

echo ">>> [준비] 로봇용 ROS-Gazebo bridge 시작 중..."
setsid "$BRIDGE_BIN" \
  "/model/$BOX/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" \
  '/pac/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  '/pac/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty' \
  > "$RUN_DIR/v45_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "ERROR: Bridge exited. See $RUN_DIR/v45_bridge.log" >&2
  exit 1
fi

setsid python3 -u "$ROOT/scripts/run_mission_v45.py" \
  --placement "$RUN_DIR/placement.json" --result-out "$RUN_DIR/execution_result.json" &
NODE_PID=$!
set +e
wait "$NODE_PID"
RC=$?
set -e
echo ">>> 결과 파일: $RUN_DIR/{placement.json,execution_result.json}"
exit "$RC"
