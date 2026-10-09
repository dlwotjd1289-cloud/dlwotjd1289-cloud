#!/usr/bin/env bash
# V4.4 closed loop, per box:
#   V4.3 weighing (scale mass) -> top-view CCTV perception (pose + footprint)
#   -> AHEAD placement planner (Donghan PlacementPlanner, SKU by footprint, mass from scale)
#   -> MoveIt2 suction pick (camera pose) & place into the planned slot/yaw
#   -> ACTUAL placed pose committed to the planner state -> AHEAD live viewer sync.
#   bash scripts/run_ahead_cycle_v44.sh [COUNT]      (default 6; max 12 = gripper plugins)
# Requires: hdr50_workcell_v4_4_pick.launch.py (V4.4 world with CCTVs), hdr50_moveit_v44.launch.py.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
COUNT="${1:-6}"
WORLD="ahead_workcell_v4_2_physical_scale"
RUN_DIR="$ROOT/logs/v44_ahead_cycle/$(date +%Y%m%d_%H%M%S)"
STATE="$RUN_DIR/planner_state.json"
mkdir -p "$RUN_DIR"
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'echo "[ahead cycle] interrupted"; exit 130' INT TERM

box_exists() { [[ "$(timeout 10 ign model -m "$1" 2>/dev/null || true)" == *"Name: $1"* ]]; }

echo ">>> [준비] PICK·팔레트 CCTV 영상 bridge와 박스 인식 노드 시작 중..."
if ! timeout 10 ign topic -l | grep -Fx /pac/cctv_pick/image >/dev/null; then
  echo "ERROR: PICK CCTV not found; start hdr50_workcell_v4_4_pick.launch.py (V4.4 world)." >&2
  exit 1
fi
setsid "$BRIDGE_BIN" '/pac/cctv_pick/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
  '/pac/cctv_pallet/image@sensor_msgs/msg/Image[ignition.msgs.Image' > "$RUN_DIR/cctv_bridge.log" 2>&1 &
PIDS+=($!)
setsid python3 -u "$ROOT/scripts/box_perception_v44.py" > "$RUN_DIR/perception.log" 2>&1 &
PIDS+=($!)

echo ">>> [준비] 단일 시험 박스가 팔레트를 막지 않도록 정리 중..."
if box_exists v43_scale_box_5kg; then
  timeout 5 ign topic -t /pac/gripper/detach -m ignition.msgs.Empty -p ' ' >/dev/null 2>&1 || true
  timeout 10 ign service -s "/world/$WORLD/set_pose" --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "v43_scale_box_5kg" position {x: -2.5 y: -2.0 z: 0.13} orientation {w: 1}' >/dev/null
fi

PLACED=()
for ((k = 1; k <= COUNT; k++)); do
  BOX=$(printf "box_%02d" "$k")
  if box_exists "$BOX"; then
    # Resume: a box already resting on the pallet (e.g. placed before an interrupted run) is
    # committed to the planner state from its actual Gazebo pose; anything else is an error.
    read -r PX PY PZ PYAW <<< "$(python3 - "$ROOT/scripts" "$BOX" <<'PYEOF'
import math, sys
sys.path.insert(0, sys.argv[1])
from moveit_pick_place_v44 import gz_model_pose
(x, y, z), (qx, qy, qz, qw) = gz_model_pose(sys.argv[2])
print(x, y, z, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
PYEOF
)"
    if python3 -c "import sys; x,y,z=map(float,sys.argv[1:4]); sys.exit(0 if abs(x)<0.6 and 0.6<y<1.8 and z<1.3 else 1)" "$PX" "$PY" "$PZ"; then
      MASS=$(grep -ohE 'V4.3 PASS: [0-9.]+' "$ROOT"/logs/v44_ahead_cycle/*/"${BOX}_v43.log" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$' || true)
      echo ">>> [$k/$COUNT] $BOX already on the pallet: committing its actual pose (resume, mass ${MASS:-5.0} kg)"
      python3 -I "$ROOT/scripts/ahead_planner_bridge_v44.py" commit --state "$STATE" --box-id "$BOX" \
        --mass "${MASS:-5.0}" --world "$PX" "$PY" "$PZ" "$PYAW"
      # Viewer may already have it (duplicate id is rejected there); a box placed by an
      # interrupted run never got synced, so try again.
      python3 "$ROOT/scripts/sync_pallet_viewer_v44.py" --box "$BOX" --mass "${MASS:-5.0}" >> "$RUN_DIR/viewer_sync.log" 2>&1 || true
      PLACED+=("$BOX")
      continue
    fi
    PICK_WAIT=$(python3 -c "import sys; x,y,z=map(float,sys.argv[1:4]); print(1 if x>-1.15 and x<-0.85 and abs(y-1.2)<0.1 and abs(z-1.02)<0.02 else 0)" "$PX" "$PY" "$PZ")
    MASS=$(grep -ohE 'V4.3 PASS: [0-9.]+' "$ROOT"/logs/v44_ahead_cycle/*/"${BOX}_v43.log" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$' || true)
    if [[ "$PICK_WAIT" != 1 || -z "$MASS" ]]; then
      echo "ERROR: $BOX exists but is neither on the pallet nor weighed and waiting at PICK; start from a fresh V4.4 world." >&2
      exit 1
    fi
    echo ">>> [$k/$COUNT] $BOX already weighed ($MASS kg) and waiting at PICK: resume from perception"
    SKIP_WEIGH=1
  else
    SKIP_WEIGH=0
  fi
  if [[ "$SKIP_WEIGH" == 1 ]]; then
    :
  else
  echo ">>> ===== [$k/$COUNT] $BOX: ① 자동 계량 ====="
  V43_BOX="$BOX" bash "$ROOT/scripts/run_auto_scale_v43.sh" > "$RUN_DIR/${BOX}_v43.log" 2>&1 || true
  grep -E 'V4.3 (PASS|FAIL)' "$RUN_DIR/${BOX}_v43.log" || true
  MASS=$(grep -oE 'V4.3 PASS: [0-9.]+' "$RUN_DIR/${BOX}_v43.log" | grep -oE '[0-9.]+$' || true)
  [[ -n "$MASS" ]] || { echo "CYCLE FAIL: weighing failed for $BOX"; exit 1; }
  fi

  echo ">>> ===== [$k/$COUNT] $BOX: ② CCTV 박스 인식 ====="
  python3 "$ROOT/scripts/perceive_once_v44.py" --out "$RUN_DIR/${BOX}_perception.json" \
    || { echo "CYCLE FAIL: perception failed for $BOX"; exit 1; }

  echo ">>> ===== [$k/$COUNT] $BOX: ③ 적재 알고리즘(AHEAD planner) 계획 ====="
  SLOT=$(python3 -I "$ROOT/scripts/ahead_planner_bridge_v44.py" plan --state "$STATE" --box-id "$BOX" \
         --mass "$MASS" --perception "$RUN_DIR/${BOX}_perception.json" --remaining $((COUNT - k)) \
         --out "$RUN_DIR/${BOX}_plan.json" 2> "$RUN_DIR/${BOX}_plan.err" | tail -1) \
    || { cat "$RUN_DIR/${BOX}_plan.err"; echo "CYCLE FAIL: planner failed for $BOX"; exit 1; }
  read -r SX SY SZ SYAW <<< "$SLOT"
  python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"    planner: candidate {d['candidate_id']} score {d['score']:.3f}, corner {[round(v,3) for v in d['target_corner_pallet']]} yaw {d['yaw_rad']:.2f} rad, {d['n_ranked']} ranked / {d['n_rejected']} rejected, {d['planning_time_sec']:.2f} s -> world ($SX, $SY, $SZ)\")" "$RUN_DIR/${BOX}_plan.json"

  echo ">>> ===== [$k/$COUNT] $BOX: ④ MoveIt 흡착 Pick & Place (카메라 위치 사용) ====="
  PERCEPTION_EXTERNAL=1 bash "$ROOT/scripts/run_moveit_pick_place_v44.sh" --box "$BOX" \
    --slot "$SX" "$SY" "$SZ" --slot-yaw "$SYAW" --pose-source camera --placed "${PLACED[@]}" \
    > "$RUN_DIR/${BOX}_moveit.log" 2>&1 || true
  grep -E '^>>> \[MoveIt|\[camera\]|MOVEIT PICK&PLACE' "$RUN_DIR/${BOX}_moveit.log" | sed -E 's/; planners:.*//' || true
  grep -q 'MOVEIT PICK&PLACE PASS' "$RUN_DIR/${BOX}_moveit.log" \
    || { echo "CYCLE FAIL at $BOX (log $RUN_DIR/${BOX}_moveit.log; then python3 scripts/moveit_reset_v44.py)"; exit 1; }
  PLACED+=("$BOX")

  echo ">>> ===== [$k/$COUNT] $BOX: ⑤ 실제 적재 결과를 상태에 반영·3D 뷰어 동기화 ====="
  read -r PX PY PZ PYAW <<< "$(python3 - "$ROOT/scripts" "$BOX" <<'PYEOF'
import math, sys
sys.path.insert(0, sys.argv[1])
from moveit_pick_place_v44 import gz_model_pose
(x, y, z), (qx, qy, qz, qw) = gz_model_pose(sys.argv[2])
print(x, y, z, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
PYEOF
)"
  python3 -I "$ROOT/scripts/ahead_planner_bridge_v44.py" commit --state "$STATE" --box-id "$BOX" \
    --mass "$MASS" --world "$PX" "$PY" "$PZ" "$PYAW"
  python3 "$ROOT/scripts/sync_pallet_viewer_v44.py" --box "$BOX" --mass "$MASS" | tee -a "$RUN_DIR/viewer_sync.log" || true
  rm -rf "$ROOT/scripts/__pycache__"
done
echo "AHEAD CYCLE PASS: ${#PLACED[@]} boxes (${PLACED[*]}); planner state $STATE; logs $RUN_DIR"
