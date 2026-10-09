#!/usr/bin/env bash
# V4.4 multi-box stacking: for each box, V4.3 automatic weighing -> MoveIt2 suction pick &
# place into the next slot (already placed boxes are planning-scene obstacles and must not
# move) -> optional sync to the AHEAD live pallet viewer.
#   bash scripts/run_stack_v44.sh [COUNT]     (default 8; max 12 = gripper plugins)
#   POSE_SOURCE=camera bash scripts/run_stack_v44.sh 8   (pick pose from the top-view CCTV)
#   PLAN_FILE=plan.json bash scripts/run_stack_v44.sh 6  (slots from an AHEAD plan, viewer format)
# Requires Gazebo V4.4 + hdr50_moveit_v44.launch.py. Viewer sync only if the AHEAD
# simulator (scripts/run_ahead_simulator.py, http://127.0.0.1:4173) is running.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
COUNT="${1:-8}"
WORLD="ahead_workcell_v4_2_physical_scale"
LOG_DIR="$ROOT/logs/v44_stack/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# Slot plan: scripts/stack_plan_v44.py (placeholder 2 x 3 pattern, far rows first) or an
# AHEAD plan in the viewer /api/place JSON format via PLAN_FILE=plan.json.
mapfile -t SLOTS < <(python3 "$ROOT/scripts/stack_plan_v44.py" "$COUNT" ${PLAN_FILE:+--plan "$PLAN_FILE"})
if (( ${#SLOTS[@]} < COUNT )); then echo "STACK FAIL: slot plan invalid"; exit 1; fi

box_exists() { [[ "$(timeout 10 ign model -m "$1" 2>/dev/null || true)" == *"Name: $1"* ]]; }

echo ">>> [적재 준비] 단일 시험 박스가 팔레트를 막지 않도록 정리 중..."
if box_exists v43_scale_box_5kg; then
  timeout 5 ign topic -t /pac/gripper/detach -m ignition.msgs.Empty -p ' ' >/dev/null 2>&1 || true
  timeout 10 ign service -s "/world/$WORLD/set_pose" --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "v43_scale_box_5kg" position {x: -2.5 y: -2.0 z: 0.13} orientation {w: 1}' >/dev/null
  echo "    v43_scale_box_5kg moved to the floor at (-2.5, -2.0)"
fi

PLACED=()
for ((k = 1; k <= COUNT; k++)); do
  BOX=$(printf "box_%02d" "$k")
  read -r SX SY SZ SYAW <<< "${SLOTS[$((k - 1))]}"
  if box_exists "$BOX"; then
    echo ">>> [$k/$COUNT] $BOX already in the world: treated as placed (rerun / resume)"
    PLACED+=("$BOX")
    continue
  fi
  echo ">>> ===== [$k/$COUNT] $BOX: 자동 계량 ====="
  V43_BOX="$BOX" bash "$ROOT/scripts/run_auto_scale_v43.sh" 2>&1 | tee "$LOG_DIR/${BOX}_v43.log" | grep -E '^>>>|V4.3 (PASS|FAIL)'
  MASS=$(grep -oE 'V4.3 PASS: [0-9.]+' "$LOG_DIR/${BOX}_v43.log" | grep -oE '[0-9.]+$' || true)
  if [[ -z "$MASS" ]]; then echo "STACK FAIL: weighing failed for $BOX (see $LOG_DIR)"; exit 1; fi
  echo ">>> ===== [$k/$COUNT] $BOX: MoveIt 적재 → 슬롯 ($SX, $SY, $SZ), yaw $SYAW rad ====="
  if ! bash "$ROOT/scripts/run_moveit_pick_place_v44.sh" --box "$BOX" --slot "$SX" "$SY" "$SZ" --slot-yaw "$SYAW" \
       --pose-source "${POSE_SOURCE:-gazebo}" --placed "${PLACED[@]}" 2>&1 | tee "$LOG_DIR/${BOX}_moveit.log" | grep -E '^>>>|^    |MOVEIT PICK&PLACE'; then
    true
  fi
  if ! grep -q 'MOVEIT PICK&PLACE PASS' "$LOG_DIR/${BOX}_moveit.log"; then
    echo "STACK FAIL at $BOX (see $LOG_DIR/${BOX}_moveit.log; after fixing, python3 scripts/moveit_reset_v44.py)"
    exit 1
  fi
  PLACED+=("$BOX")
  python3 "$ROOT/scripts/sync_pallet_viewer_v44.py" --box "$BOX" --mass "$MASS" | tee -a "$LOG_DIR/viewer_sync.log" || true
  rm -rf "$ROOT/scripts/__pycache__"
done
echo "STACK PASS: ${#PLACED[@]} boxes on the pallet (${PLACED[*]}); logs in $LOG_DIR"
