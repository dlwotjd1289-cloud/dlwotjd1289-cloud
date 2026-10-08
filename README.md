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
external/             four Hyundai reference repos (pinned Git submodules)
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
git clone https://github.com/dlwotjd1289-cloud/pac2026-ahead.git
cd pac2026-ahead
bash scripts/preflight.sh
bash scripts/fetch_hyundai_refs.sh
bash scripts/check_hdp160_support.sh
bash scripts/run_tests.sh
bash scripts/build_ros_ws.sh
```

The workcell-only Gazebo world can be developed and viewed before the HDP160-31
robot description is available.

## Updating Hyundai references

For an existing checkout, commit or back up local dependency changes before pulling
AHEAD updates, then run `bash scripts/fetch_hyundai_refs.sh` again. It is safe to
rerun after a successful initialization or after `git clone --recurse-submodules`.
The script uses `git submodule sync --recursive` and
`git submodule update --init --recursive --checkout` for the four Hyundai references,
including `hdr_client_driver`. It checks out the commits recorded in the AHEAD
index, not the latest `humble` branch. Do not use `--remote` for installation.
If you have deliberately staged different gitlink SHAs, those are the new pins;
commit them to make the dependency change reproducible for other users.

Existing clean standalone clones from the old installer are handled by Git's
submodule update. Dirty repositories (including untracked files and initialized
nested submodules) are rejected before updating any reference. A nonempty directory
without Git metadata must be backed up and moved aside manually. The script never
uses force, reset, or clean. Network failures can leave initialization incomplete;
resolve the network problem and rerun the script.

The four tracked `ros2_ws/src/hdr_*` symlinks point to
`../../external/hyundai_robotics/hdr_*`. Missing/replaced links are reported before
updating dependencies; restore the named link with `git restore -- <path>` after
backing up any local replacement. Links are checked again after initialization.
This requires a filesystem that supports symlinks (the target Ubuntu environment).

Run `python3 -m pytest -q tests/test_hyundai_submodules.py` for offline Git integration
regressions. They use local fixture repositories and do not build ROS 2 or validate
an HDP160-31 model. The upstream Hyundai model and driver sources are not modified.
