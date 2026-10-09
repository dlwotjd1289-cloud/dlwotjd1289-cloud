#!/usr/bin/env bash
# V4.4 MoveIt2 pick & place: starts suction gripper + box-pose bridge, runs
# moveit_pick_place_v44.py, then stops everything it started.
# Preconditions: V4.4 launch + hdr50_moveit_v44.launch.py running, box at PICK (run V4.3 first).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'echo "[moveit pick] stopping..."; exit 130' INT TERM

setsid bash "$ROOT/scripts/run_suction_gripper.sh" &
PIDS+=($!)
# Box name from --box (default v43_scale_box_5kg); other args go to the client unchanged.
BOX="v43_scale_box_5kg"
ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do [[ "${ARGS[$i]}" == "--box" ]] && BOX="${ARGS[$((i + 1))]}"; done
setsid "$BRIDGE_BIN" "/model/$BOX/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" \
  > "$ROOT/logs/v44_moveit_pose_bridge.log" 2>&1 &
PIDS+=($!)
# Top-view CCTV localization (--pose-source camera): image bridge + perception node.
# The debug image (/pac/perception/debug_image) is shown in the RViz review layout.
if [[ " $* " == *" --pose-source camera "* && "${PERCEPTION_EXTERNAL:-0}" != 1 ]]; then
  echo ">>> [MoveIt 준비] PICK CCTV 영상 bridge·박스 인식 노드 시작 중..."
  setsid "$BRIDGE_BIN" '/pac/cctv_pick/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/pac/cctv_pallet/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
    > "$ROOT/logs/v44_cctv_bridge.log" 2>&1 &
  PIDS+=($!)
  setsid python3 -u "$ROOT/scripts/box_perception_v44.py" > "$ROOT/logs/v44_perception.log" 2>&1 &
  PIDS+=($!)
fi
sleep 3
python3 -u "$ROOT/scripts/moveit_pick_place_v44.py" "$@"
