# PAC2026 — V4.6 3D Simulation Environment (2026-10-10 snapshot)

## Scope

This branch preserves the latest local V4.6 Gazebo Fortress + ROS 2 Humble workcell
for PAC2026 mixed-palletizing integration. It is a **3D simulation environment
snapshot**, not a claim that every automated robot scenario has passed.

- HDR50-22 robot with MoveIt2 and a suction-cup tool in Gazebo.
- Two fixed RGB cameras: scale top-view and pole CCTV (PICK, pallet, buffer).
- Automated scale/conveyor workflow, block pallet, two-bay buffer, and outbound pallet handling.
- Perception-based pick/placement checks and fail-closed vertical descent evidence.
- Gazebo world: `ros2_ws/src/pac_simulation/worlds/ahead_workcell_v4_6_two_cam.sdf`.
- Launch: `ros2_ws/src/pac_bringup/launch/hdr50_workcell_v4_6.launch.py`.
- Robot executor: `scripts/moveit_pick_place_v44.py` and
  `scripts/vertical_descent_v46.py`; cycle runner: `scripts/run_generator_cycle_v44.sh`.

## Validation boundary

**Observed in previous Gazebo runs**

- Previously placed cartons 1–12 were demonstrated; a later fixture restored
  the 12 recorded poses within 10 mm XY with stable settling.
- Scale and fixed-camera operation was exercised. Buffer-bay placement and
  pallet-outbound movement were checked in independent Gazebo tests.
- Two boxes with 8 mm gap were observed stable in an independent Gazebo static
  test (not evidence of an 8 mm robot placement trajectory).
- Controller alignment reached the existing 0.5 mm limit; first guarded descent
  reached a Z endpoint within 1 mm after passive settling.

**Still not verified end-to-end**

- The actual last 6 mm cup press can exceed the 1.5 mm lateral/orientation
  execution budget (~1.51–1.54 mm in the recorded trials). This remains open.
- Robot-driven buffer put/take; robot-driven 8 mm adjacent placement; and
  successful recovery from injected suction/place faults remain open.
- Any simulated/mock grip bypass is only a development aid and **is not**
  evidence of actual vacuum/contact success. No mock bypass is treated as a
  production safety change in this snapshot.
- The final integrated all-box Gazebo cycle with teammates' latest algorithm
  PRs has not yet been verified.

Do not label the workcell as fully validated or hide the unverified stages.

## Separation of ownership

This snapshot covers the V4.6 3D workcell, MoveIt/Gazebo simulation executor,
and test fixtures. Team members' planning/learning PRs are not merged by this
publication. Their interfaces require a separate integration review.

## Running

From the project root, after sourcing ROS 2 Humble and the built workspace,
launch the V4.6 cell and the matching MoveIt configuration in separate terminals:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch pac_bringup hdr50_workcell_v4_6.launch.py
# separate terminal, with same ROS 2 environment:
ros2 launch pac_bringup hdr50_moveit_v44.launch.py
```

Existing source scripts and `docs/jaesung/README_V44.md` describe the cycle
runner and configuration. Do not run the experimental mock-grip test tools as
normal production execution.
