#!/usr/bin/env bash
# V4.4 full cycle: automatic weighing (V4.3) -> MoveIt2 suction pick & place onto the pallet.
# Requires: Terminal 1  ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
#           Terminal 2  ros2 launch pac_bringup hdr50_moveit_v44.launch.py   (rviz:=false to hide RViz)
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo ">>> ===== 1단계: 자동 계량 (V4.3) ====="
bash "$ROOT/scripts/run_auto_scale_v43.sh"
echo ">>> ===== 2단계: MoveIt2 흡착 Pick & Place (V4.4) ====="
bash "$ROOT/scripts/run_moveit_pick_place_v44.sh"
