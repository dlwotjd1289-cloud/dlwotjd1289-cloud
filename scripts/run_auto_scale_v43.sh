#!/usr/bin/env bash
# Standalone V4.3 test harness: uses unchanged Gazebo V4.2 world.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

WORLD="ahead_workcell_v4_2_physical_scale"
# Box model name: V43_BOX=box_03 bash scripts/run_auto_scale_v43.sh (stacking runs).
BOX="${V43_BOX:-v43_scale_box_5kg}"
# Generator boxes: V43_BOX_SDF (model file), V43_BOX_LENGTH [m], V43_EXPECTED_MASS [kg], V43_SPAWN_Z [m].
BOX_SDF="${V43_BOX_SDF:-$ROOT/test_data/scale_auto_box_5kg_v43.sdf}"
SPAWN_Z="${V43_SPAWN_Z:-1.05}"
# Arrival pose at the inlet (generator cycle ARRIVAL_JITTER=1: lateral offset / yaw of the arriving box).
SPAWN_Y="${V43_SPAWN_Y:-1.20}"
# inlet x (V4.6 world: scale moved upstream by PAC_SCALE_SHIFT_M)
INLET_X=$(python3 -c "print(-4.45 + float('${PAC_SCALE_SHIFT_M:-0}'))")
SPAWN_YAW="${V43_SPAWN_YAW:-0.0}"
SPAWN_QZ=$(python3 -c "import math; print(math.sin($SPAWN_YAW / 2))")
SPAWN_QW=$(python3 -c "import math; print(math.cos($SPAWN_YAW / 2))")
NODE_ARGS=("$BOX")
# V43_TARE_KG: empty scale platform incl. rollers (7 in the V4.2 world, 10 in the V4.4 world,
# which has 3 extra intermediate rollers on the platform). Auto-detect V4.4 by its gripper topic.
if [[ -z "${V43_TARE_KG:-}" ]] && timeout 10 ign topic -l | grep -Fx /pac/gripper/state >/dev/null; then
  V43_TARE_KG=10.0
  : "${V43_BOX_LENGTH:=0.40}" "${V43_EXPECTED_MASS:=5.0}"
fi
if [[ -n "${V43_BOX_LENGTH:-}" && -n "${V43_EXPECTED_MASS:-}" ]]; then
  NODE_ARGS+=("$V43_BOX_LENGTH" "$V43_EXPECTED_MASS" "${V43_TARE_KG:-7.0}")
fi
if [[ "$BOX" == "v43_scale_box_5kg" ]]; then GRIP_NS="/pac/gripper"; else GRIP_NS="/pac/gripper/$BOX"; fi
# Call executables directly (no `ros2 run` wrapper): a killed wrapper leaves
# its child running, which left an orphan bridge after the 17:30 run.
CREATE_BIN="$(ros2 pkg prefix ros_gz_sim)/lib/ros_gz_sim/create"
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
echo ">>> [준비] Gazebo V4.2 실행 여부 확인 중..."
gz_up() { timeout 10 ign topic -l 2>/dev/null | grep -Fx "/world/$WORLD/clock" >/dev/null; }
# retried for 30 s: the first topic listing of a heavy world (V4.6) once came back empty
for _ in 1 2 3; do gz_up && break; sleep 2; done
if ! gz_up; then
  echo "ERROR: Start Gazebo V4.2 in Terminal 1 and press ▶ before running." >&2
  exit 1
fi

box_exists() {
  # Capture first: `| grep -q` under pipefail can report false on SIGPIPE. Retry: with many
  # models in the world, `ign model` occasionally times out (box_23 run, 2026-10-09).
  local out
  for _ in 1 2 3; do
    out="$(timeout 10 ign model -m "$BOX" 2>/dev/null || true)"
    [[ "$out" == *"Name: $BOX"* ]] && return 0
    [[ "$out" == *"No model named"* ]] && return 1
    sleep 1
  done
  return 1
}

release_box() {
  # V4.4 gripper (DetachableJoint) attaches when the box first appears; release it.
  # Harmless in the plain V4.2 world (no subscriber).
  timeout 5 ign topic -t "$GRIP_NS/detach" -m ignition.msgs.Empty -p ' ' >/dev/null 2>&1 || true
}

reset_box_to_inlet() {
  timeout 10 ign service -s "/world/$WORLD/set_pose" \
    --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req "name: \"$BOX\" position {x: $INLET_X y: $SPAWN_Y z: $SPAWN_Z} orientation {z: $SPAWN_QZ w: $SPAWN_QW}" | grep -q "data: true"
}

BRIDGE_PID=""
NODE_PID=""
# Bridge and node run in their own sessions (setsid), so terminal Ctrl+C only
# reaches this script and cleanup stops them in a safe order. SIGTERM is used
# because background jobs of a non-interactive shell ignore SIGINT.
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
  # Stop the controller first so it cannot re-send a non-zero roller command,
  # then command zero directly in Gazebo, then stop the bridge.
  stop_group "$NODE_PID"
  for _ in 1 2 3; do
    timeout 3 ign topic -t /pac/conveyor/roller_cmd_vel -m ignition.msgs.Double -p 'data: 0.0' >/dev/null 2>&1 || true
  done
  stop_group "$BRIDGE_PID"
}
trap cleanup EXIT
trap 'echo "[V4.3] Interrupted; stopping rollers and child processes..."; exit 130' INT TERM

# Repeated runs in one Gazebo session reuse our own test box: it is moved back
# to the inlet instead of being deleted (entity removal crashed the Gazebo GUI
# once, and the V4.4 gripper binds to the first box entity it sees).
echo ">>> [준비] 이전 테스트 박스 확인 중..."
REUSE_BOX=0
if [[ "${V43_BOX_AT_INLET:-0}" == 1 ]] && box_exists; then
  # Created at the inlet (and released) by the V4.4 cycle runner while the arm stood still: use it
  # as is. A teleport (set_pose) after the spawn-time attach/detach left the later suction joint
  # without effect in Fortress (box did not follow the cup; 2026-10-09).
  REUSE_BOX=1
  echo ">>> [준비] 컨베이어 입구에 미리 생성된 박스 사용 ($BOX)"
elif box_exists; then
  REUSE_BOX=1
  # Move it before the empty-scale tare (it may have stopped on the scale).
  echo ">>> [준비] 기존 5kg 테스트 박스를 컨베이어 입구로 되돌리는 중..."
  release_box
  sleep 0.5
  if ! reset_box_to_inlet; then
    echo "ERROR: Could not move $BOX back to the inlet; relaunch clean Gazebo world." >&2
    exit 1
  fi
  sleep 1
fi

mkdir -p "$ROOT/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
BRIDGE_LOG="$ROOT/logs/v43_scale_bridge_$STAMP.log"

echo ">>> [준비] ROS-Gazebo bridge 시작 중..."
# '[' bridges Gazebo -> ROS; ']' bridges ROS -> Gazebo.
setsid "$BRIDGE_BIN" \
  '/pac/scale/wrench@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench' \
  "/model/$BOX/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" \
  '/pac/conveyor/roller_cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double' \
  > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
sleep 3
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "ERROR: Bridge exited. See $BRIDGE_LOG" >&2
  exit 1
fi

echo ">>> [준비] 자동 계량 제어 노드 시작 중..."
setsid python3 -u "$ROOT/scripts/run_auto_scale_v43.py" "${NODE_ARGS[@]}" &
NODE_PID=$!
sleep 2
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "ERROR: Controller failed to start. See ROS error above." >&2
  exit 1
fi

if [[ "$REUSE_BOX" == 0 ]]; then
  echo ">>> [준비] 5kg 테스트 박스를 컨베이어 입구에 투입 중..."
  if ! timeout -k 3 30 "$CREATE_BIN" \
    -name "$BOX" \
    -file "$BOX_SDF" \
    -x "$INLET_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y "$SPAWN_YAW"; then
      echo "ERROR: Spawn failed; relaunch clean Gazebo V4.2 world." >&2
      exit 1
  fi
  # `create` reports OK once the request is sent; confirm the entity exists.
  for _ in $(seq 25); do box_exists && break; sleep 0.2; done
  if ! box_exists; then
    echo "ERROR: $BOX was not created in Gazebo (see Gazebo terminal)." >&2
    exit 1
  fi
  release_box
fi

set +e
wait "$NODE_PID"
RC=$?
set -e
exit "$RC"
