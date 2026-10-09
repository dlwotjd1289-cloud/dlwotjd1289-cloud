#!/usr/bin/env bash
# Stop what run_review_v44.sh started (own processes only, matched by their exact command lines).
for p in $(pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch pac_bringup hdr50_(moveit_v44|workcell_v4_4_pick)'); do
  kill -INT "$p" 2>/dev/null
done
for p in $(pgrep -f '^python3 [^ ]*scripts/run_ahead_simulator.py'); do kill -TERM "$p" 2>/dev/null; done
# Leftovers of an interrupted cycle (an orphan suction node once gripped the next box itself).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for p in $(pgrep -f "^bash $ROOT/scripts/(run_generator_cycle_v44|run_moveit_pick_place_v44|run_suction_gripper|run_auto_scale_v43)\.sh"); do
  kill -TERM "$p" 2>/dev/null
done
for p in $(pgrep -f "^python3 (-u )?$ROOT/scripts/(suction_gripper_node|box_perception_v44|moveit_pick_place_v44|run_auto_scale_v43)\.py"); do
  kill -TERM "$p" 2>/dev/null
done
sleep 6
for p in $(pgrep -f '^(ign gazebo|/opt/ros/humble/lib/(robot_state_publisher|ros_gz_bridge|moveit_ros_move_group|rviz2|controller_manager))'); do
  kill -9 "$p" 2>/dev/null
done
echo "review stopped"
