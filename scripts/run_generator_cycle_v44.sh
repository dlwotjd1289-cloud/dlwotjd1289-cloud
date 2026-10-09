#!/usr/bin/env bash
# V4.4 closed loop on an AHEAD dataset-generator scenario (arrival order from the generator,
# placement decided by the AHEAD planner). Per arriving box:
#   spawn generator SKU box (size, mass) -> V4.3 weighing -> fixed pole CCTV coarse pose
#   -> AHEAD planner (SKU from arrival identity, scale mass, remaining stock by SKU only)
#   -> MoveIt2: gripper-camera refinement, suction pick, place at planned slot/yaw
#   -> actual pose committed to the planner state -> AHEAD live viewer sync.
#   bash scripts/run_generator_cycle_v44.sh DATASET_DIR SCENARIO_ID [COUNT]
#   e.g. bash scripts/run_generator_cycle_v44.sh ~/AHEAD/v44_generated/sample_seed20261009 S0001 24
#   Resume after a stop in the same Gazebo world: RESUME_DIR=<previous run dir> bash ... (boxes in
#   its planner state are kept; a weighed box waiting at PICK continues from perception).
# Requires a fresh V4.4 world (hdr50_workcell_v4_4_pick.launch.py) + hdr50_moveit_v44.launch.py.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
DATASET="$1"; SCEN="$2"; COUNT="${3:-24}"
CATALOG="${GENERATOR_CONFIG:-$ROOT/../ahead-dataset-generator/config/default.yaml}"
ARRIVALS="$DATASET/simulation_observations/$SCEN.jsonl"
[[ -f "$ARRIVALS" && -f "$CATALOG" ]] || { echo "ERROR: $ARRIVALS or $CATALOG not found" >&2; exit 1; }
RUN_DIR="$ROOT/logs/v44_generator_cycle/${SCEN}_$(date +%Y%m%d_%H%M%S)"
STATE="$RUN_DIR/planner_state.json"
mkdir -p "$RUN_DIR/boxes"
if [[ -n "${RESUME_DIR:-}" ]]; then
  cp "$RESUME_DIR/planner_state.json" "$STATE" 2>/dev/null || true
  cp "$RESUME_DIR"/box_*_v43.log "$RUN_DIR/" 2>/dev/null || true
  cp "$RESUME_DIR/buffer.json" "$RUN_DIR/buffer.json" 2>/dev/null || true
fi
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
WORLD="ahead_workcell_v4_2_physical_scale"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'echo "[generator cycle] interrupted"; exit 130' INT TERM
box_exists() {   # retried: `ign model` occasionally times out with many models in the world
  local out
  for _ in 1 2 3; do
    out="$(timeout 10 ign model -m "$1" 2>/dev/null || true)"
    [[ "$out" == *"Name: $1"* ]] && return 0
    [[ "$out" == *"No model named"* ]] && return 1
    sleep 1
  done
  return 1
}

# Arrival table: k sku x y z mass generator_box_id (order revealed one box at a time below).
python3 - "$ARRIVALS" "$COUNT" > "$RUN_DIR/arrivals.tsv" <<'PYEOF'
import json, sys
rows = [json.loads(l)["observation"] for l in open(sys.argv[1])][: int(sys.argv[2])]
for k, r in enumerate(rows, 1):
    s = r["size"]
    print(k, r["sku_id"], s["x"], s["y"], s["z"], r["weight_kg"], r["box_id"], sep="\t")
PYEOF
N=$(wc -l < "$RUN_DIR/arrivals.tsv")
echo ">>> 시나리오 $SCEN: 박스 $N개 (generator 도착 순서), 로그 $RUN_DIR"

echo ">>> [준비] 고정 CCTV·그리퍼 카메라 영상 bridge와 박스 인식 노드 시작 중..."
setsid "$BRIDGE_BIN" '/pac/top_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
  '/pac/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
  '/pac/scale_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' > "$RUN_DIR/camera_bridge.log" 2>&1 &
PIDS+=($!)
setsid python3 -u "$ROOT/scripts/box_perception_v44.py" > "$RUN_DIR/perception.log" 2>&1 &
PIDS+=($!)
if box_exists v43_scale_box_5kg; then
  timeout 10 ign service -s "/world/$WORLD/set_pose" --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
    --timeout 3000 --req 'name: "v43_scale_box_5kg" position {x: -2.5 y: -2.0 z: 0.13} orientation {w: 1}' >/dev/null
fi

HELD=()
CREATE_BIN="$(ros2 pkg prefix ros_gz_sim)/lib/ros_gz_sim/create"
# ARRIVAL_JITTER=1: boxes arrive off-centre / turned like on a real infeed (lateral offset up to
# +-ARRIVAL_DY_M, yaw up to +-ARRIVAL_YAW_DEG, reproducible per ARRIVAL_SEED and arrival index);
# perception + gripper camera must correct the pick. Default: centred, yaw 0 (generator pose).
arrival_pose() {   # K -> "y yaw_rad" at the inlet
  python3 - "$1" <<PYEOF
import math, random, sys
k = int(sys.argv[1])
if "${ARRIVAL_JITTER:-0}" != "1":
    print(1.20, 0.0)
else:
    r = random.Random(int("${ARRIVAL_SEED:-20261009}") * 1000 + k)
    dy = r.uniform(-1, 1) * float("${ARRIVAL_DY_M:-0.08}")
    yaw = math.radians(r.uniform(-1, 1) * float("${ARRIVAL_YAW_DEG:-10}"))
    print(f"{1.20 + dy:.4f} {yaw:.4f}")
PYEOF
}
spawn_at_inlet() {   # K: create arrival K at the conveyor inlet and release the gripper joint
  local K="$1" SKU SX SY SZ MASS GEN_ID BOX SDF SPAWN_Z
  IFS=$'\t' read -r _ SKU SX SY SZ MASS GEN_ID <<< "$(awk -F'\t' -v k="$K" '$1 == k' "$RUN_DIR/arrivals.tsv")"
  BOX=$(printf "box_%02d" "$K")
  SDF="$RUN_DIR/boxes/${BOX}.sdf"
  SPAWN_Z=$(python3 "$ROOT/scripts/make_box_sdf_v44.py" --size "$SX" "$SY" "$SZ" --mass "$MASS" --out "$SDF")
  local AY AYAW
  read -r AY AYAW <<< "$(arrival_pose "$K")"
  # one retry: a create request under load was lost once (box_11, 2026-10-09)
  timeout -k 3 20 "$CREATE_BIN" -name "$BOX" -file "$SDF" -x "$(python3 -c "print(-4.45 + float('${PAC_SCALE_SHIFT_M:-0}'))")" -y "$AY" -z "$SPAWN_Z" -Y "$AYAW" >/dev/null 2>&1 \
    || { sleep 1; box_exists "$BOX" \
         || timeout -k 3 20 "$CREATE_BIN" -name "$BOX" -file "$SDF" -x "$(python3 -c "print(-4.45 + float('${PAC_SCALE_SHIFT_M:-0}'))")" -y "$AY" -z "$SPAWN_Z" -Y "$AYAW" >/dev/null 2>&1; } \
    || return 1
  for _ in $(seq 25); do box_exists "$BOX" && break; sleep 0.2; done
  box_exists "$BOX" || return 1
  for _ in 1 2 3; do
    timeout 5 ign topic -t "/pac/gripper/$BOX/detach" -m ignition.msgs.Empty -p ' ' >/dev/null 2>&1 || true
  done
  sleep 0.5
}

# Pipelining (as on a real line): once the robot has lifted the current box off PICK, the next
# box is fed and weighed while the robot transfers / places / returns. PIPELINE=0 disables it.
PREFETCH_BOX=""; PREFETCH_PID=""
is_placed() {
  [[ -f "$STATE" ]] && python3 -c "import json,sys; sys.exit(0 if any(b['box_id']==sys.argv[2] for b in json.load(open(sys.argv[1]))['placed']) else 1)" "$STATE" "$1"
}
start_weighing() {   # K: spawn + V4.3 for arrival K in the background (setsid group, in PIDS)
  local K="$1" SKU SX SY SZ MASS GEN_ID BOX SDF SPAWN_Z
  IFS=$'\t' read -r _ SKU SX SY SZ MASS GEN_ID <<< "$(awk -F'\t' -v k="$K" '$1 == k' "$RUN_DIR/arrivals.tsv")"
  BOX=$(printf "box_%02d" "$K")
  SDF="$RUN_DIR/boxes/${BOX}.sdf"
  SPAWN_Z=$(python3 "$ROOT/scripts/make_box_sdf_v44.py" --size "$SX" "$SY" "$SZ" --mass "$MASS" --out "$SDF")
  local AY AYAW
  read -r AY AYAW <<< "$(arrival_pose "$K")"
  if [[ "${ARRIVAL_JITTER:-0}" == 1 ]]; then
    echo "$BOX $AY $AYAW" >> "$RUN_DIR/arrival_jitter.tsv"
    python3 -c "import math; print(f'    도착 자세 $BOX: 중심에서 {($AY - 1.20) * 1000:+.0f} mm, 회전 {math.degrees($AYAW):+.1f} deg')"
  fi
  V43_BOX="$BOX" V43_BOX_SDF="$SDF" V43_SPAWN_Z="$SPAWN_Z" V43_BOX_LENGTH="$SX" V43_EXPECTED_MASS="$MASS" \
    V43_SPAWN_Y="$AY" V43_SPAWN_YAW="$AYAW" \
    V43_TARE_KG="${V43_TARE_KG:-10.0}" V43_BOX_AT_INLET="${2:-0}" \
    setsid bash "$ROOT/scripts/run_auto_scale_v43.sh" > "$RUN_DIR/${BOX}_v43.log" 2>&1 < /dev/null &
  WEIGH_PID=$!
  PIDS+=("$WEIGH_PID")
  if [[ "${PAC_LAYOUT:-}" == v46 ]]; then
    # V4.6 camera 1 (top view of the scale): stable once the box rests on the scale.
    rm -f "$RUN_DIR/${BOX}_perception.json"
    setsid python3 "$ROOT/scripts/perceive_once_v44.py" --camera scale --box "$BOX" --size "$SX" "$SY" "$SZ" \
      --timeout 120 --out "$RUN_DIR/${BOX}_perception.json" > "$RUN_DIR/${BOX}_scale_perception.log" 2>&1 < /dev/null &
    PIDS+=($!)
  fi
}
# ---- buffer table / pallet change / gripper payload (2026-10-09) ---------------------------------
# Policy: a box without a slot on the current pallet goes to the buffer table (2 bays) while the
# following boxes fill the remaining space; buffered boxes are re-planned after every placement.
# Only when the buffer is full (or the box does not fit a bay) is the pallet changed; buffered
# boxes are then placed first on the new pallet. Stack height: nominal 1.5 m, up to 1.6 m.
BUFFER="$RUN_DIR/buffer.json"
[[ -f "$BUFFER" ]] || echo '{}' > "$BUFFER"
PALLET_NO=1
GRIPPER="${PAC_GRIPPER:-pad}"
GRIPPER_CAP=$(python3 -c "import re,sys; s=open(sys.argv[1]).read(); d=eval(re.search(r'GRIPPERS = (\{[^}]*\})', s).group(1)); print(d[sys.argv[2]])" "$ROOT/scripts/moveit_pick_place_v44.py" "$GRIPPER")
echo ">>> [준비] 그리퍼 $GRIPPER (정격 ${GRIPPER_CAP} kg), 적층 높이 기준 ${PAC_STACK_NOMINAL_M:-1.5} m / 허용 ${PAC_STACK_MAX_M:-1.6} m, 버퍼 2칸"
SWAPS=0; BUFFERED_TOTAL=0

arrive_pick() {   # V4.6: the plan was made while the box travelled; wait until it is at PICK
  [[ -n "${TRAVEL_PID:-}" ]] || return 0
  wait "$TRAVEL_PID" || true
  TRAVEL_PID=""
  grep -E 'V4.3 (PASS|FAIL)' "$RUN_DIR/${BOX}_v43.log" | tail -1
  grep -q 'V4.3 PASS' "$RUN_DIR/${BOX}_v43.log" || { echo "CYCLE FAIL: $BOX did not reach PICK"; exit 1; }
}

hold_box() {   # BOX SZ REASON: simulated operator / reject lane (not a robot move)
  local HX HY
  HX=$(python3 -c "print(-2.6 + 0.6 * ((${#HELD[@]}) % 4))"); HY=$(python3 -c "print(-1.6 - 0.6 * ((${#HELD[@]}) // 4))")
  timeout 10 ign service -s "/world/$WORLD/set_pose" --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
    --timeout 3000 --req "name: \"$1\" position {x: $HX y: $HY z: $(python3 -c "print($2/2+0.01)")} orientation {w: 1}" >/dev/null
  HELD+=("$1")
  echo ">>> $1: $3 -> 보류 구역으로 이동 (HELD ${#HELD[@]})"
}

remaining_json() {   # K: SKUs still to come after arrival K + boxes waiting on the buffer
  python3 - "$RUN_DIR/arrivals.tsv" "$1" "$BUFFER" <<'PYEOF'
import json, sys
c = {}
for line in open(sys.argv[1]):
    k, sku = line.split("\t")[:2]
    if int(k) > int(sys.argv[2]):
        c[sku] = c.get(sku, 0) + 1
for v in json.load(open(sys.argv[3])).values():
    c[v["sku"]] = c.get(v["sku"], 0) + 1
print(json.dumps(c))
PYEOF
}

plan_slot() {   # BOX SKU MASS PERCEPTION REMAINING [PICK_X PICK_Y] -> "x y z yaw" or empty
  local extra=()
  if [[ -n "${6:-}" ]]; then
    extra=(--pick-xy "$6" "$7")
  elif [[ "${PAC_LAYOUT:-}" == v46 ]]; then
    extra=(--pick-xy -1.03 1.20)   # perception came from the scale camera; the robot picks at PICK
  fi
  local plan_rc=0 out="$RUN_DIR/${1}_plan.json" err="$RUN_DIR/${1}_plan.err"
  rm -f "$out"
  python3 -I "$ROOT/scripts/ahead_planner_bridge_v44.py" plan --state "$STATE" --box-id "$1" --sku "$2" \
    --catalog "$CATALOG" --mass "$3" --perception "$4" --remaining-json "$5" "${extra[@]}" \
    --out "$out" > "$RUN_DIR/${1}_plan.stdout" 2> "$err" || plan_rc=$?
  if [[ "$plan_rc" == 20 ]] && grep -q '^PLAN FAIL: NO_SLOT (' "$err"; then
    return 0  # Empty slot means a valid no-slot decision, never a Python failure.
  fi
  if [[ "$plan_rc" != 0 ]]; then
    echo "PLANNER ERROR: $1 (exit $plan_rc); cell stops, no buffer/pallet fallback" >&2
    tail -20 "$err" >&2
    return "$plan_rc"
  fi
  # Parse the plan artifact, not arbitrary stdout; stale/malformed output is an error.
  python3 - "$out" "$1" <<'PYPLAN'
import json, math, sys
try:
    plan = json.load(open(sys.argv[1]))
    assert plan['box_id'] == sys.argv[2]
    values = [*plan['slot_world'], plan['yaw_rad']]
    assert len(values) == 4 and all(math.isfinite(float(v)) for v in values)
    print(' '.join(f'{float(v):.4f}' for v in values))
except (OSError, ValueError, KeyError, TypeError, AssertionError) as exc:
    print(f'PLANNER ERROR: invalid plan artifact: {exc}', file=sys.stderr)
    sys.exit(21)
PYPLAN
}

extra_placed() {   # boxes on the buffer table as MoveIt obstacles
  python3 -c "import json,sys; print(json.dumps({v['box']: v['size'] for v in json.load(open(sys.argv[1])).values()}))" "$BUFFER" \
    > "$RUN_DIR/buffer_obstacles.json"
}

robot_move() {   # BOX SX SY SZ MASS TAG -- extra moveit args. Pipelining only for conveyor picks.
  local BOX="$1" SX="$2" SY="$3" SZ="$4" MASS="$5" TAG="$6"; shift 7
  local LOG="$RUN_DIR/${BOX}_moveit_${TAG}.log" K NEXT_K NEXT_BOX GATE="" SPAWNED=0
  extra_placed
  if [[ " $* " != *" --pick-from buffer "* ]]; then
    K=$((10#${BOX#box_})); NEXT_K=$((K + 1)); NEXT_BOX=$(printf "box_%02d" "$NEXT_K")
    if [[ "${PIPELINE:-1}" == 1 && "$NEXT_K" -le "$N" ]] && ! is_placed "$NEXT_BOX" && ! box_exists "$NEXT_BOX"; then
      GATE="$RUN_DIR/.spawned_$NEXT_BOX"; rm -f "$GATE"
    fi
  fi
  SPAWN_GATE="$GATE" PERCEPTION_EXTERNAL=1 setsid bash "$ROOT/scripts/run_moveit_pick_place_v44.sh" --box "$BOX" \
    --size "$SX" "$SY" "$SZ" --mass "$MASS" --gripper "$GRIPPER" --pose-source camera --placed-state "$STATE" \
    --extra-placed-json "$RUN_DIR/buffer_obstacles.json" --result-json "$RUN_DIR/${BOX}_placed_${TAG}.json" "$@" \
    > "$LOG" 2>&1 < /dev/null &
  local MOVEIT_PID=$!
  PIDS+=("$MOVEIT_PID")
  while kill -0 "$MOVEIT_PID" 2>/dev/null; do
    if [[ -n "$GATE" && "$SPAWNED" == 0 ]] && grep -q '다음 박스를 컨베이어 입구에 생성 대기' "$LOG" 2>/dev/null; then
      if spawn_at_inlet "$NEXT_K"; then
        echo "    >>> 로봇 정지 중 다음 박스 $NEXT_BOX 컨베이어 입구에 생성"; SPAWNED=1
      else
        echo "    ! $NEXT_BOX 생성 실패: 로봇 작업 후 순차 진행"; SPAWNED=2
      fi
      touch "$GATE"
    fi
    if [[ "$SPAWNED" == 1 && -z "$PREFETCH_BOX" ]] && grep -q 'PICK 비움' "$LOG" 2>/dev/null; then
      echo "    >>> PICK 비움 -> 다음 박스 $NEXT_BOX 계량·이송을 로봇 작업과 동시에 시작"
      start_weighing "$NEXT_K" 1
      PREFETCH_BOX="$NEXT_BOX"; PREFETCH_PID="$WEIGH_PID"
    fi
    sleep 0.3
  done
  wait "$MOVEIT_PID" || true
  cp "$LOG" "$RUN_DIR/${BOX}_moveit.log"
  grep -E '\[camera\]|복구|MOVEIT PICK&PLACE' "$LOG" | sed -E 's/; planners:.*//' || true
  grep -q 'MOVEIT PICK&PLACE PASS' "$LOG" \
    || { echo "CYCLE FAIL at $BOX (log $LOG; then python3 scripts/moveit_reset_v44.py)"; exit 1; }
}

commit_box() {   # BOX SKU MASS SX SY SZ TAG
  local PX PY PZ PW
  read -r PX PY PZ PW <<< "$(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(d['x'], d['y'], d['z'], d['yaw'])" "$RUN_DIR/${1}_placed_${7}.json")"
  python3 -I "$ROOT/scripts/ahead_planner_bridge_v44.py" commit --state "$STATE" --box-id "$1" --sku "$2" \
    --catalog "$CATALOG" --mass "$3" --world "$PX" "$PY" "$PZ" "$PW" --planned "$RUN_DIR/${1}_plan.json"
  python3 "$ROOT/scripts/sync_pallet_viewer_v44.py" --box "$1" --mass "$3" --size "$4" "$5" "$6" \
    ${VIEWER_ROBOT:+--robot} >> "$RUN_DIR/viewer_sync.log" 2>&1 || true
  rm -rf "$ROOT/scripts/__pycache__"
}

show_plan() {   # BOX SLOT
  python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"    planner: {d['candidate_id']} score {d['score']:.3f}, corner {[round(v,3) for v in d['target_corner_pallet']]} yaw {d['yaw_rad']:.2f} rad, 높이 기준 {d['height_mode']} ({d['stack_limit_m']} m), 이송 TCP z {d['transfer_tcp_z']} m -> world ({sys.argv[2]})\")" "$RUN_DIR/${1}_plan.json" "$2"
}

buffer_bay_slot() {   # SX SY SZ -> "BAY x y z yaw" for a free bay the box fits, or empty
  python3 - "$BUFFER" "$1" "$2" "$3" <<'PYEOF'
import json, math, sys
buf = json.load(open(sys.argv[1]))
sx, sy, sz = map(float, sys.argv[2:5])
across, along = min(sx, sy), max(sx, sy)
import os
depth = float(os.environ.get("PAC_BUFFER_DEPTH", "0.58"))
if across > 0.37 or along > depth - 0.02:   # bay: 0.38 m between posts (x), shelf depth (y; V4.6 0.66 m)
    sys.exit(0)
yaw = math.pi / 2 if sx > sy else 0.0    # long side along the shelf depth
import os
cx, cy = float(os.environ.get("PAC_BUFFER_X", "1.45")), float(os.environ.get("PAC_BUFFER_Y", "-0.55"))
for bay, bx in (("0", cx - 0.215), ("1", cx + 0.215)):
    if bay not in buf:
        print(bay, round(bx, 4), cy, round(0.9425 + sz / 2, 4), round(yaw, 4))
        break
PYEOF
}

buffer_put() {   # BOX SKU MASS SX SY SZ BAY BX BY BZ BYAW
  echo ">>> $1: 버퍼 테이블 칸 $7 에 임시 보관 (현재 팔레트 자리 없음, 다음 박스들로 빈 공간 채우기)"
  robot_move "$1" "$4" "$5" "$6" "$3" buffer -- --slot "$8" "$9" "${10}" --slot-yaw "${11}" --place-on buffer
  python3 - "$BUFFER" "$7" "$1" "$2" "$3" "$4" "$5" "$6" "$RUN_DIR/${1}_placed_buffer.json" "$RUN_DIR/${1}_perception.json" <<'PYEOF'
import json, sys
p, bay, box, sku, mass, sx, sy, sz, placed, perc = sys.argv[1:]
buf = json.load(open(p))
m = json.load(open(placed))
buf[bay] = {"box": box, "sku": sku, "mass": float(mass), "size": [float(sx), float(sy), float(sz)],
            "pose": [m["x"], m["y"], m["z"], m["yaw"]], "perception": perc}
json.dump(buf, open(p, "w"), indent=1)
PYEOF
  BUFFERED_TOTAL=$((BUFFERED_TOTAL + 1))
}

try_buffered() {   # K: re-plan every buffered box on the current pallet, place those that fit
  local bay rec BOX SKU MASS SX SY SZ PX PY PZ PW PERC SLOT
  for bay in 0 1; do
    rec=$(python3 -c "import json,sys; b=json.load(open(sys.argv[1])).get(sys.argv[2]); print('' if b is None else ' '.join(map(str, [b['box'], b['sku'], b['mass'], *b['size'], *b['pose'], b['perception']])))" "$BUFFER" "$bay")
    [[ -n "$rec" ]] || continue
    read -r BOX SKU MASS SX SY SZ PX PY PZ PW PERC <<< "$rec"
    SLOT=$(plan_slot "$BOX" "$SKU" "$MASS" "$PERC" "$(remaining_json "$1")" "$PX" "$PY") || { echo "CYCLE FAIL: planner error for buffered $BOX"; exit 1; }
    [[ -n "$SLOT" ]] || continue
    echo ">>> 버퍼 칸 $bay 의 $BOX: 이제 자리 있음 -> 버퍼에서 집어 팔레트에 적재"
    show_plan "$BOX" "$SLOT"
    read -r PSX PSY PSZ PYAW <<< "$SLOT"
    robot_move "$BOX" "$SX" "$SY" "$SZ" "$MASS" frombuffer -- --slot "$PSX" "$PSY" "$PSZ" --slot-yaw "$PYAW" \
      --pick-from buffer --pick-pose "$PX" "$PY" "$PZ" "$PW"
    python3 -c "import json,sys; b=json.load(open(sys.argv[1])); b.pop(sys.argv[2]); json.dump(b, open(sys.argv[1], 'w'), indent=1)" "$BUFFER" "$bay"
    commit_box "$BOX" "$SKU" "$MASS" "$SX" "$SY" "$SZ" frombuffer
  done
}

swap_pallet() {
  local NP
  NP=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['placed']))" "$STATE" 2>/dev/null || echo 0)
  echo ">>> ===== 팔레트 교체: 팔레트 $PALLET_NO (박스 $NP 개) 출고, 빈 팔레트 투입 (AGV/작업자 역할, 시뮬레이션 처리) ====="
  python3 "$ROOT/scripts/pallet_swap_v44.py" --state "$STATE" --index "$PALLET_NO" | tail -1
  cp "$STATE" "$RUN_DIR/pallet_${PALLET_NO}_final_state.json"
  PALLET_NO=$((PALLET_NO + 1)); SWAPS=$((SWAPS + 1))
  STATE="$RUN_DIR/pallet_${PALLET_NO}_state.json"
  curl -sf -X POST -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:4173/api/reset >/dev/null 2>&1 || true
}

# fd 3: commands inside the loop must not consume the arrival list from stdin.
while IFS=$'\t' read -r -u 3 K SKU SX SY SZ MASS GEN_ID; do
  BOX=$(printf "box_%02d" "$K")
  SKIP_WEIGH=0
  if [[ -f "$STATE" ]] && python3 -c "import json,sys; sys.exit(0 if any(b['box_id']==sys.argv[2] for b in json.load(open(sys.argv[1]))['placed']) else 1)" "$STATE" "$BOX"; then
    echo ">>> [$K/$N] $BOX already placed (resume)"
    continue
  fi
  if [[ -f "$RUN_DIR/buffer.json" ]] && python3 -c "import json,sys; sys.exit(0 if any(v['box']==sys.argv[2] for v in json.load(open(sys.argv[1])).values()) else 1)" "$RUN_DIR/buffer.json" "$BOX"; then
    echo ">>> [$K/$N] $BOX is on the buffer table (resume)"
    continue
  fi
  if [[ "$PREFETCH_BOX" == "$BOX" ]]; then
    :   # fed and weighed during the previous robot cycle (waited for below)
  elif box_exists "$BOX"; then
    BY=$(python3 - "$ROOT/scripts" "$BOX" <<'PYEOF'
import sys
sys.path.insert(0, sys.argv[1])
from moveit_pick_place_v44 import gz_model_pose
p = gz_model_pose(sys.argv[2])
print(p[0][1] if p else 0.0)
PYEOF
)
    if python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < -1.0 else 1)" "$BY"; then
      echo ">>> [$K/$N] $BOX is in the hold area (no placement from planner earlier): keep held"
      HELD+=("$BOX")
      continue
    fi
    if [[ -n "${RESUME_DIR:-}" ]] && grep -q 'V4.3 PASS' "$RUN_DIR/${BOX}_v43.log" 2>/dev/null; then
      echo ">>> [$K/$N] $BOX already weighed and waiting at PICK (resume from perception)"
      SKIP_WEIGH=1
    elif [[ -n "${RESUME_DIR:-}" ]]; then
      echo ">>> [$K/$N] $BOX exists but was not weighed: weigh again (runner reuses the box)"
    else
      echo "ERROR: $BOX already exists; start from a fresh V4.4 world (or RESUME_DIR=...)." >&2; exit 1
    fi
  fi
  # Remaining stock by SKU = boxes still to arrive after this one (counts only, no order).
  REMAINING=$(awk -F'\t' -v k="$K" '$1 > k {c[$2]++} END {printf "{"; n=0; for (s in c) {printf "%s\"%s\": %d", (n++ ? ", " : ""), s, c[s]} printf "}"}' "$RUN_DIR/arrivals.tsv")
  echo ">>> ===== [$K/$N] $BOX = $GEN_ID ($SKU ${SX}x${SY}x${SZ} m, ${MASS} kg): ① 투입·자동 계량 ====="
  TRAVEL_PID=""
  if [[ "$PREFETCH_BOX" == "$BOX" ]]; then
    echo "    (로봇 작업 중에 미리 투입·계량 시작됨)"
    TRAVEL_PID="$PREFETCH_PID"
    PREFETCH_BOX=""
  elif [[ "$SKIP_WEIGH" != 1 ]]; then
    start_weighing "$K"
    TRAVEL_PID="$WEIGH_PID"
  fi
  if [[ "${PAC_LAYOUT:-}" == v46 && -n "$TRAVEL_PID" ]]; then
    # V4.6: mass right after weighing (box still travelling to PICK) -> camera 1 -> plan now.
    for _ in $(seq 1200); do
      grep -q 'WEIGHED [0-9.]* kg' "$RUN_DIR/${BOX}_v43.log" 2>/dev/null && break
      kill -0 "$TRAVEL_PID" 2>/dev/null || break
      sleep 0.2
    done
    WMASS=$(grep -oE 'WEIGHED [0-9.]+ kg' "$RUN_DIR/${BOX}_v43.log" | grep -oE '[0-9.]+' | head -1 || true)
    [[ -n "$WMASS" ]] || { wait "$TRAVEL_PID"; grep -E 'V4.3 (PASS|FAIL)' "$RUN_DIR/${BOX}_v43.log"; echo "CYCLE FAIL: weighing failed for $BOX"; exit 1; }
    echo "    계량 완료 ${WMASS} kg (박스는 PICK으로 이송 중)"
    echo ">>> ===== [$K/$N] $BOX: ② 카메라 1 (계량 구간 탑뷰) 박스 인식 ====="
    for _ in $(seq 150); do [[ -s "$RUN_DIR/${BOX}_perception.json" ]] && break; sleep 0.2; done
    [[ -s "$RUN_DIR/${BOX}_perception.json" ]] || { echo "CYCLE FAIL: scale camera perception failed for $BOX"; exit 1; }
    tail -1 "$RUN_DIR/${BOX}_scale_perception.log"
  else
    [[ -n "$TRAVEL_PID" ]] && { wait "$TRAVEL_PID" || true; }
    TRAVEL_PID=""
    grep -E 'V4.3 (PASS|FAIL)' "$RUN_DIR/${BOX}_v43.log" || true
    WMASS=$(grep -oE 'V4.3 PASS: [0-9.]+' "$RUN_DIR/${BOX}_v43.log" | grep -oE '[0-9.]+$' || true)
    [[ -n "$WMASS" ]] || { echo "CYCLE FAIL: weighing failed for $BOX"; exit 1; }
    if [[ "${PAC_LAYOUT:-}" != v46 || ! -s "$RUN_DIR/${BOX}_perception.json" ]]; then
      echo ">>> ===== [$K/$N] $BOX: ② 고정 CCTV 박스 인식 ====="
      python3 "$ROOT/scripts/perceive_once_v44.py" --box "$BOX" --size "$SX" "$SY" "$SZ" \
        --out "$RUN_DIR/${BOX}_perception.json" || { echo "CYCLE FAIL: perception failed for $BOX"; exit 1; }
    fi
  fi

  if python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)" "$WMASS" "$GRIPPER_CAP"; then
    arrive_pick
    hold_box "$BOX" "$SZ" "그리퍼 정격 ${GRIPPER_CAP} kg 초과 (${WMASS} kg): 흡착판 확장(pad_xl 45 kg) 또는 지지대(pad_fork 60 kg) 필요 [PAC_GRIPPER]"
    continue
  fi

  echo ">>> ===== [$K/$N] $BOX: ③ 적재 알고리즘 계획 (팔레트 $PALLET_NO) ====="
  SLOT=$(plan_slot "$BOX" "$SKU" "$WMASS" "$RUN_DIR/${BOX}_perception.json" "$(remaining_json "$K")") || { echo "CYCLE FAIL: planner error for $BOX"; exit 1; }
  if [[ -z "$SLOT" ]]; then
    tail -1 "$RUN_DIR/${BOX}_plan.err" | cut -c1-240
    BAY=$(buffer_bay_slot "$SX" "$SY" "$SZ")
    arrive_pick
    if [[ -n "$BAY" ]]; then
      echo ">>> ===== [$K/$N] $BOX: ④ 버퍼로 이동 ====="
      buffer_put "$BOX" "$SKU" "$WMASS" "$SX" "$SY" "$SZ" $BAY
      continue
    fi
    echo ">>> $BOX: 현재 팔레트에 자리 없음, 버퍼 $(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))))" "$BUFFER")/2 칸 사용 중(또는 칸보다 큼) -> 팔레트 교체"
    swap_pallet
    try_buffered "$K"
    echo ">>> ===== [$K/$N] $BOX: ③ 적재 알고리즘 계획 (새 팔레트 $PALLET_NO) ====="
    SLOT=$(plan_slot "$BOX" "$SKU" "$WMASS" "$RUN_DIR/${BOX}_perception.json" "$(remaining_json "$K")") || { echo "CYCLE FAIL: planner error for $BOX"; exit 1; }
    if [[ -z "$SLOT" ]]; then
      tail -1 "$RUN_DIR/${BOX}_plan.err" | cut -c1-240
      hold_box "$BOX" "$SZ" "빈 팔레트에도 자리 없음(planner)"
      continue
    fi
  fi
  read -r PSX PSY PSZ PYAW <<< "$SLOT"
  show_plan "$BOX" "$SLOT"
  if [[ -n "${TRAVEL_PID:-}" ]]; then echo "    (계획 완료, 박스 PICK 도착 대기)"; fi
  arrive_pick
  echo ">>> ===== [$K/$N] $BOX: ④ MoveIt 흡착 Pick & Place (CCTV + 그리퍼 카메라) ====="
  robot_move "$BOX" "$SX" "$SY" "$SZ" "$WMASS" pallet -- --slot "$PSX" "$PSY" "$PSZ" --slot-yaw "$PYAW"
  echo ">>> ===== [$K/$N] $BOX: ⑤ 실제 적재 결과 반영(그리퍼 카메라 측정)·3D 뷰어 동기화 ====="
  commit_box "$BOX" "$SKU" "$WMASS" "$SX" "$SY" "$SZ" pallet
  try_buffered "$K"
done 3< "$RUN_DIR/arrivals.tsv"
# Arrivals done: buffered boxes go onto the pallet (a last pallet change if they do not fit).
try_buffered "$N"
if [[ "$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))))" "$BUFFER")" != 0 ]]; then
  echo ">>> 도착 종료, 버퍼에 남은 박스가 현재 팔레트에 안 들어감 -> 팔레트 교체 후 적재"
  swap_pallet
  try_buffered "$N"
fi
NPL=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['placed']))" "$STATE" 2>/dev/null || echo 0)
NLEFT=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))))" "$BUFFER")
echo "GENERATOR CYCLE PASS: $SCEN $N boxes: pallets $PALLET_NO (changes $SWAPS), on the current pallet $NPL, buffered on the way $BUFFERED_TOTAL, left on the buffer $NLEFT, held ${#HELD[@]} (${HELD[*]:-none}); planner state $STATE; logs $RUN_DIR"
