# PAC 2026 — AHEAD Mixed Palletizing (HDP160-31)

Target robot: **HD Hyundai Robotics HDP160-31**.

This scaffold follows the team common development standard:

- Ubuntu 22.04 / ROS 2 Humble / Python 3.10
- monorepo with ROS 2 packages under `ros2_ws/src`
- canonical runtime dataclasses live only in `pac_common`
- only `StateManager` may commit ACTUAL `SystemState`
- SI units internally, every pose has `frame_id`
- JSON fixtures / YAML config / JSONL runtime logs
- AHEAD packing core stays robot-agnostic
- HDP160-31 logic is isolated in `pac_robot`
- primary perception boundary is fixed top-view camera
- robot integration fails closed until a verified HDP160-31 model is available

## Important robot-model rule

Do **not** copy another Hyundai robot's joint origins, inertias, collision meshes, or
controller tuning into HDP160-31 just to make MoveIt run.

The public Hyundai ROS 2 repositories are used as framework references. If HDP160-31
is not present, obtain a verified model / CAD / kinematic data before enabling the
robot backend.

## Layout

```text
config/               common config
external/             cloned reference repos (gitignored)
logs/                 JSONL runtime logs (gitignored)
ros2_ws/src/
  pac_common/         canonical dataclasses + StateManager
  pac_perception/     fixed top-view observation boundary
  pac_planning/       robot-agnostic AHEAD interfaces
  pac_robot/          HDP160-31 RobotCapability / feasibility adapter
  pac_simulation/     realistic workcell V2 assets
  pac_bringup/        launch orchestration
scripts/              preflight / refs / build / checks
test_data/            JSON fixtures
tests/                unit + contract tests
```

## First Ubuntu session

```bash
bash scripts/preflight.sh
bash scripts/fetch_hyundai_refs.sh
bash scripts/check_hdp160_support.sh
bash scripts/run_tests.sh
bash scripts/build_ros_ws.sh
```

The workcell-only Gazebo world can be developed and viewed before the HDP160-31
robot description is available.
