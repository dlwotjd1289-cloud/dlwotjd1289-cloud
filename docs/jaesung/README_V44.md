# PAC2026 V4.4 — Suction-cup gripper on HDR50-22 (Gazebo Fortress)

V4.2 world/launch files are unchanged. V4.4 adds new files only.

## What is implemented (verified 2026-10-08)

- Suction cup EOAT on `flange_link` (`ros2_ws/src/pac_bringup/urdf/hdr50_pedestal_gripper.urdf.xacro`)
  - placeholder geometry: mount Ø60×30 mm + lip Ø80×30 mm; **suction face (TCP) = flange +x 0.06 m**
  - built from official gz-sim 6 systems only (no maintained Fortress vacuum plugin exists;
    `gazebo_ros_vacuum_gripper` is Gazebo Classic only):
    contact sensor + `Contact` system, `TouchPlugin`, `DetachableJoint`
- World `ahead_workcell_v4_4_suction.sdf` = V4.2 world + `Contact` system (placed after
  `UserCommands`, otherwise sensors of the runtime-spawned robot are missed). World name unchanged.
- Launch `hdr50_workcell_v4_4_pick.launch.py` = V4.2 launch with the two files above.
- Vacuum logic `scripts/suction_gripper_node.py`: grips **only when vacuum is ON and the cup
  touches the box**; releases on OFF.
  - `/pac/suction/vacuum` (std_msgs/Bool) command, `/pac/suction/state` (OFF/SEARCHING/GRIPPED)
  - TouchPlugin enable is a Gazebo service, called with `ign service` (not bridgeable in Humble).

Test result: V4.3 auto weighing → suction test passed 3 times in a row
(box lifted 150 mm with the cup, released, displacement ≤ 1.0 mm). Logs: `logs/v44_e2e/`.

## Run

```bash
# Terminal 1
ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
# Terminal 4
bash scripts/run_auto_scale_v43.sh        # box to PICK (5 kg weighed)
bash scripts/run_suction_test_v44.sh      # suction grip / lift / release test
bash scripts/run_suction_gripper.sh       # gripper only (for other controllers, e.g. MoveIt2)
```

Notes: the DetachableJoint binds to the first `v43_scale_box_5kg` entity and attaches it on
spawn; `run_auto_scale_v43.sh` releases it and reuses the box (teleport to inlet) on repeat runs.

## MoveIt2 pick & place (verified 2026-10-08)

- `ros2_ws/src/pac_bringup/launch/hdr50_moveit_v44.launch.py`: move_group (+RViz) with the same
  robot description as Gazebo; SRDF `urdf/hdr50_22_suction.srdf.xacro` (tip `suction_tcp`, generated
  from hdr50_22_moveit_config); planner/limit/controller YAML reused from hdr50_22_moveit_config.
- `scripts/moveit_pick_place_v44.py`: planning scene (floor, pedestal, conveyor, pick stopper, pallet,
  buffer rack, camera pole, box) → OMPL to above box → Cartesian descent (checked to 1 cm, then a
  1.6 cm unchecked contact press) → vacuum ON / GRIPPED → attach box in MoveIt → move 1 cm off the
  stopper (unchecked; box rests against it) → level Cartesian lift/transfer/lower (collision-checked)
  → vacuum OFF → retreat → OMPL home → placement check (xy 30 mm, z 20 mm, tilt 5 deg).
- `scripts/moveit_reset_v44.py`: after an interrupted run, release the gripper and plan home.
- Result: full cycle (V4.3 weighing → MoveIt pick & place) 3/3 PASS in a row; box on pallet at
  (0.001, 1.199, 0.275) m, xy error 1.2–1.3 mm, tilt 0.00 deg. Logs: `logs/v44_e2e/final2_full*`.

Lessons recorded for later tuning:
- OMPL goals can land at wrist limits (j4 = -2π); goals constrain j4/j6 to ±π.
- Gazebo position-controlled joints lag on long fast moves (`PATH_TOLERANCE_VIOLATED`, tolerance
  0.2 rad in hdr_controllers.yaml): free moves run at 0.1 velocity scaling; transfer with the box is
  a level Cartesian move. OMPL is random — keep re-running the 3-cycle test after changes.

```bash
# Terminal 1
ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
# Terminal 2
ros2 launch pac_bringup hdr50_moveit_v44.launch.py          # rviz:=false to hide RViz
# Terminal 4
bash scripts/run_full_cycle_v44.sh                           # weighing -> pick & place
python3 scripts/moveit_reset_v44.py                          # only after an interrupted run
```

## Earlier experiments (reference only)

Superseded by the MoveIt2 pipeline above; kept as reference:

- `scripts/hdr50_kinematics.py` — FK/DLS-IK from the URDF chain (tests in `tests/test_pick_place_v44.py`)
- `scripts/pick_place_plan_v44.py`, `scripts/run_pick_place_v44.py`, `scripts/run_pick_place_v44.sh` —
  experimental pick→pallet sequence; first run stopped because joints had not settled at the
  trajectory end (0.098 rad) — check controller goal tolerances / settle time with MoveIt2.
- Workcell facts for MoveIt2: pallet `pallet_main` center (0, 1.20), deck top z = 0.15 m;
  PICK box center ≈ (-1.06, 1.20, 1.02); pick stopper x = -0.82 (top z 1.05).
- `scripts/test_suction_v44.py` moves the robot with this reference IK as a test fixture only.
