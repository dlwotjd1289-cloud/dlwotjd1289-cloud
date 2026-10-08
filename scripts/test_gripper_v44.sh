#!/usr/bin/env bash
# V4.4 gripper interface check (no robot motion): attach/detach commands must
# change the DetachableJoint state and must not disturb the box at rest.
# Physical carrying is verified later with MoveIt2-planned motion.
# Requires: hdr50_workcell_v4_4_pick.launch.py running and v43_scale_box_5kg in the world.
set -eo pipefail
BOX="v43_scale_box_5kg"
STATE_LOG="$(mktemp)"
trap 'kill "$ECHO_PID" 2>/dev/null || true; rm -f "$STATE_LOG"' EXIT

box_pose() { timeout 10 ign model -m "$BOX" -p 2>/dev/null | sed -n 7p | tr -s ' '; }
wait_state() {  # wait until the latest gripper state equals $1
  for _ in $(seq 50); do
    [[ "$(grep -o '"[a-z]*"' "$STATE_LOG" | tail -1)" == "\"$1\"" ]] && return 0
    sleep 0.1
  done
  return 1
}

echo ">>> [그리퍼 1/4] V4.4 그리퍼·테스트 박스 확인 중..."
timeout 10 ign topic -l | grep -Fx /pac/gripper/state >/dev/null || { echo "FAIL: /pac/gripper/state not found (start V4.4 launch)"; exit 1; }
[[ -n "$(box_pose)" ]] || { echo "FAIL: $BOX not in world (run V4.3 first)"; exit 1; }
ign topic -e -t /pac/gripper/state > "$STATE_LOG" 2>&1 &
ECHO_PID=$!
sleep 1
P0="$(box_pose)"
echo "    box pose before: $P0"

echo ">>> [그리퍼 2/4] 흡착(attach) 명령 전송 중..."
ign topic -t /pac/gripper/attach -m ignition.msgs.Empty -p ' '
wait_state attached || { echo "FAIL: gripper state did not become 'attached'"; exit 1; }
echo "    state: attached"

echo ">>> [그리퍼 3/4] 해제(detach) 명령 전송 중..."
sleep 1
ign topic -t /pac/gripper/detach -m ignition.msgs.Empty -p ' '
wait_state detached || { echo "FAIL: gripper state did not become 'detached'"; exit 1; }
echo "    state: detached"

echo ">>> [그리퍼 4/4] 박스 위치 변화 확인 중..."
sleep 1
P1="$(box_pose)"
echo "    box pose after:  $P1"
python3 - "$P0" "$P1" <<'EOF'
import re, sys
a, b = ([float(v) for v in re.findall(r"-?\d+\.\d+", s)] for s in sys.argv[1:3])
d = max(abs(x - y) for x, y in zip(a, b))
print(f"    max position change: {d * 1000:.1f} mm")
sys.exit(0 if d < 0.005 else 1)
EOF
echo "GRIPPER PASS: attach/detach state changes OK; box undisturbed."
