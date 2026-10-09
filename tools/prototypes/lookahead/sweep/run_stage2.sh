#!/usr/bin/env bash
REPO=${REPO:-$HOME/AHEAD/pac2026_integrated}; AUDIT=${AUDIT:-$HOME/AHEAD/audit_20261009}
cd $AUDIT/09_lookahead
export PYTHONPATH=$REPO/ros2_ws/src/pac_bringup:$REPO/ros2_ws/src/pac_candidates:$REPO/ros2_ws/src/pac_common:$REPO/ros2_ws/src/pac_eoat:$REPO/ros2_ws/src/pac_highlevel:$REPO/ros2_ws/src/pac_perception:$REPO/ros2_ws/src/pac_planning:$REPO/ros2_ws/src/pac_planning_interfaces:$REPO/ros2_ws/src/pac_robot:$REPO/ros2_ws/src/pac_simulation:$REPO/tools/virtual_data:$REPO/tools/ahead_dataset_generator/src:$REPO/scripts
V="team la_k3 la_k3;horizon=3 la_k3;hl.repack.min_gain=0.05 la_k3;hl.close.fill_before_buffer=0.62 la_k3;horizon=3;hl.repack.min_gain=0.05;hl.close.fill_before_buffer=0.62"
nice -n 5 python3 run_eval.py $V --episodes 54 --workers 18 --shock --out sweep_params/stage2_test.json
LA_DATA=$AUDIT/09_lookahead/../08_prototype/holdout_seed777 nice -n 5 python3 run_eval.py team la_k3 "la_k3;horizon=3;hl.repack.min_gain=0.05;hl.close.fill_before_buffer=0.62" --episodes 30 --workers 18 --shock --out sweep_params/stage2_holdout.json
echo STAGE2_DONE
