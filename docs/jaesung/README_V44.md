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
  buffer rack, camera pole, box) → IK from a fixed reference posture + **Pilz PTP** to above the box
  (deterministic; OMPL only as a logged fallback) → Cartesian descent (checked to 1 cm, then a 1.6 cm
  unchecked contact press) → vacuum ON / GRIPPED → attach box in MoveIt → **vertical lift** with
  box↔pick_stopper contact allowed only for that lift (like real infeeds: box rests on the end stop,
  conveyor stopped, robot lifts straight up) → level Cartesian transfer/lower (collision-checked)
  → vacuum OFF → retreat → Pilz PTP home → placement check (xy 30 mm, z 20 mm, tilt 5 deg).
- `scripts/moveit_reset_v44.py`: after an interrupted run, release the gripper and plan home.
- Result: full cycle (V4.3 weighing → MoveIt pick & place) 3/3 PASS in a row with Pilz PTP and
  vertical lift; box on pallet at (0.001, 1.199, 0.275) m, xy error 1.2–1.3 mm, tilt 0.00 deg.
  Logs: `logs/v44_e2e/vlift2_full*`.

Lessons recorded for later tuning:
- OMPL is random (different path each run; one run tilted the box ~15°) and its goals can land at
  wrist limits (j4 = -2π): free moves now use IK (wrist checked within ±π) + Pilz PTP.
- ACM edits must send the full matrix (a diff with an ACM replaces it): read, edit one pair, apply.
- First service calls right after start-up can time out (FastDDS response discovery): calls retry.
- Gazebo position-controlled joints lag on long fast moves (`PATH_TOLERANCE_VIOLATED`, tolerance
  0.2 rad in hdr_controllers.yaml): free moves run at 0.1 velocity scaling; transfer with the box is
  a level Cartesian move. Keep re-running the 3-cycle test after changes.

```bash
# Terminal 1
ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py
# Terminal 2
ros2 launch pac_bringup hdr50_moveit_v44.launch.py          # rviz:=false to hide RViz
# Terminal 4
bash scripts/run_full_cycle_v44.sh                           # weighing -> pick & place
python3 scripts/moveit_reset_v44.py                          # only after an interrupted run
```

## Multi-box stacking, top-view CCTV, pallet viewer sync, RViz review

- `scripts/run_stack_v44.sh [COUNT]`: per box `box_NN` → V4.3 weighing → MoveIt placement into the
  next slot → previously placed boxes checked (must not move > 5 mm) → AHEAD live viewer sync.
  - Gripper: one DetachableJoint per `box_01..box_12` (`/pac/gripper/box_NN/*`); suction node
    grips the box named on `/pac/suction/target`; TouchPlugin target `box_link` (all boxes).
  - Placed boxes are planning-scene obstacles (actual Gazebo poses); transfer height rises with the
    stack (carried box bottom ≥ 0.10 m above the highest box); vertical lower/retreat are
    collision-checked (cup↔released box allowed only for the retreat).
  - Slots: `scripts/stack_plan_v44.py` — placeholder 2 × 3 per layer, 20 mm gaps, far rows first;
    or `PLAN_FILE=plan.json` in the viewer `/api/place` format (AHEAD output), yaw 0 / 90 deg via
    `--slot-yaw`.
- `scripts/sync_pallet_viewer_v44.py`: Gazebo box pose (+ mass measured by the scale) → POST
  `http://127.0.0.1:4173/api/place` (start `python3 scripts/run_ahead_simulator.py`). Frame:
  Gazebo pallet centre (0, 1.20), deck top 0.15 → viewer origin pallet centre, z = 0 at deck top.
- Top-view CCTV (V4.4 world only): `cctv_pick_topview` (-1.06, 1.20, 2.60) and
  `cctv_pallet_topview` (0, 1.20, 3.20), straight down, **simulated RGB** (camera type/model is
  undecided). `scripts/box_perception_v44.py`: colour mask → projection onto the box-top plane
  (height from SKU data) → centre / yaw / size check → `/pac/perception/box_pose`, debug image.
  `--pose-source camera` (or `POSE_SOURCE=camera` for stacking) picks with this pose; error vs.
  Gazebo ground truth is printed. Tests: `tests/test_box_perception_v44.py` (synthetic images).
- RViz review: `ros2 launch pac_bringup hdr50_moveit_v44.launch.py` opens `rviz/v44_review.rviz`
  (robot, MoveIt planning scene with obstacles/placed boxes, planned paths, PICK CCTV perception
  overlay, pallet CCTV image).

## Closed loop: CCTV → AHEAD planner → robot (`scripts/run_ahead_cycle_v44.sh [COUNT]`)

Per box: ① V4.3 weighing (mass from the scale) → ② `perceive_once_v44.py` (stable CCTV detection:
pose, yaw, footprint) → ③ `ahead_planner_bridge_v44.py plan` (SKU matched by footprint, `BoxState`
with measured mass, Donghan `PlacementPlanner` from this repository
(`ros2_ws/src/pac_planning`), ReferenceBackend generator/validator, heuristic Top-K) → ④ MoveIt
suction pick with the camera pose and place at the planned slot/yaw → ⑤ actual Gazebo pose
`commit`ted as the next ACTUAL planner state (single writer, state_version++) → viewer sync.

Bridge choices (review with the planner owner): planner pallet origin = corner farthest from the
robot, axes reversed (180° rotation) so far slots fill first; boxes inflated by 10 mm in x/y for the
robot placement clearance; upright boxes, yaw 0/90° only (team contract; tipping about a horizontal
axis is not supported — decided 2026-10-08 together with the open 4/6-axis question); placeholder
SKU `GZ_TEST_40x30x25` (top-load 350 N is not measured); max stack height 1.0 m.
Taehyeon's generator/validator (`TeamPlacer`) is not available locally, so `ReferenceBackend` is used.

## Generator scenarios on the Gazebo workcell (`scripts/run_generator_cycle_v44.sh`, 2026-10-09)

The executor runs what the placement algorithm commands; the algorithm owns placement decisions.
Per arriving generator box (arrival order from `simulation_observations/<scenario>.jsonl`):
SKU box spawned (`make_box_sdf_v44.py`, size/mass) → V4.3 weighing (length-dependent thresholds,
`V43_TARE_KG=10` in the V4.4 world) → fixed pole CCTV coarse pose → planner (SKU from the arrival
identity, scale mass, remaining stock by SKU only) → MoveIt: gripper-camera refinement, suction
pick, place at the commanded slot/yaw → commit → viewer sync. No placement from the planner → the
box is moved to a hold area and the line continues. `RESUME_DIR=<run dir>` resumes in the same world.

Cameras (two only, as decided): the existing pole CCTV `camera_1_cctv_base` (`/pac/top_camera/image`)
and a gripper camera beside the cup (`wrist_camera_link`, `/pac/wrist_camera/image`). Floating CCTVs
were removed. Simulated as RGB; box height from the SKU (identity), footprint checked by vision.

World changes (V4.4 file only): conveyor rollers 0.19 → 0.095 m pitch (small SKUs fell between
rollers); 3 extra rollers on the scale platform → tare 10 kg. Gripper: 30 per-box connections.

S0001 (24 boxes, sample_seed20261009): 20 placed by the robot as commanded, 4 held (no placement
from the planner). Placement error vs. command 0.0–1.1 mm (2.6 mm under heavy CPU load), stacked
up to the 4th layer, already-placed boxes moved ≤ 0.2 mm. Logs: `logs/v44_generator_cycle/S0001_*`.

Executor fixes found on the way: CCTV/gripper-camera colour mask (pallet boards V≈173 vs box top
V≈199) and blob selection (other box tops in view); slow final lowering + 10 mm gap; commit at the
planned corner (keeps the planner model consistent); 90° turns done in place, slowly, choosing the
turn direction that keeps j6 inside ±π; trajectory points with duplicate times dropped; retries for
`ign model` timeouts; wait for the pose stream after lifting (Gazebo ran at 0.2× under CPU load).

## AHEAD live 3D physics model + robot cell (PyBullet / Three.js, 2026-10-09)

Original simulator backed up unchanged in `~/AHEAD/backup_ahead_live_sim_original_20261009`.
Added (`ros2_ws/src/pac_simulation/pac_simulation/ahead_sim/robot_cell.py`, small hooks in
`world.py` / `simulator.py` / `server.py`, viewer additions):

- HDR50-22 with suction cup and gripper camera from the same URDF as Gazebo/MoveIt
  (`scripts/make_pybullet_robot_urdf_v44.py` → `models/pybullet/hdr50_22_suction.urdf`), pedestal,
  conveyor end with PICK stopper, pole CCTV (viewer: housing + field of view).
  Simulator frame = Gazebo world − (0, 1.20, 0.15).
- `POST /api/robot_place` (same JSON as `/api/place`): the box is delivered to PICK, picked by
  suction (fixed constraint), lifted, turned, carried above the stack, lowered slowly and
  **released**; Bullet decides how it settles. Joints follow interpolated targets under
  POSITION_CONTROL; IK = damped least squares on a shadow robot. Boxes in transit are excluded from
  the pallet metrics. `GET /api/render.png?view=iso|top|side` renders the physics world in software.
- Viewer: robot STL meshes (`/mesh/<file>`, URDF meshes only), infeed rollers, stopper, pedestal,
  CCTV frustum, robot panel (state, queue, per-box placement error, log).
- `scripts/replay_to_robot_cell_v44.py <run dirs>` replays a Gazebo run's planner commands on the
  robot cell; `VIEWER_ROBOT=1 bash scripts/run_generator_cycle_v44.sh ...` mirrors live.
- Result: all 20 planner commands of the S0001 run placed by the PyBullet robot (release + physics),
  error 0.1–0.3 mm, tilt 0.0°. Renders: `logs/v44_3d_renders/S0001_robot_cell_{iso,top,side}.png`.

```bash
python3 scripts/run_ahead_simulator.py            # http://127.0.0.1:4173
python3 scripts/replay_to_robot_cell_v44.py logs/v44_generator_cycle/S0001_<...> ...
curl -o render.png "http://127.0.0.1:4173/api/render.png?view=iso"
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

## 2026-10-09 (afternoon): review run, pipelining, pallet deck unified

- One-command review: `bash scripts/run_review_v44.sh [SCEN] [COUNT]` (fresh Gazebo V4.4 + MoveIt/RViz
  review layout + browser viewer http://127.0.0.1:4173 with `VIEWER_ROBOT=1` + generator scenario);
  stop with `bash scripts/stop_review_v44.sh`. `NO_BROWSER=1` skips opening Firefox.
- Pipelining in `run_generator_cycle_v44.sh` (`PIPELINE=0` disables): while the cup presses the current
  box (arm still) the next box is created at the inlet (`SPAWN_GATE`), and once the box is lifted off
  PICK (marker `PICK 비움`) the next V4.3 weighing runs in parallel (`V43_BOX_AT_INLET=1`, no teleport).
  S0001 box cycle ~95 s -> ~55 s at RTF 1.0.
  Why not spawn while the arm moves: the gripper's DetachableJoint attaches every new box_NN on
  appearance; a spawn during motion tied the far box to the moving arm (joint 3 tolerance abort,
  box flung off the inlet). Why no parking + teleport: after the spawn-time attach/detach, a
  set_pose teleport left the later suction joint without effect (box did not follow the cup).
- Suction node: attach/detach publishers are created when the target is set and GRIPPED is reported
  only after the attach went to a matched subscriber (a publisher created at touch time lost its
  first message: "box did not follow the suction cup", also box_13 on 2026-10-09 night).
- `moveit_pick_place_v44.py`: placed-box poses from one `/world/<w>/pose/info` snapshot (~0.4 s)
  instead of `ign model -p` per box (~4 s each).
- RTF 2 (2 ms step) was tried and dropped: light box overshot the scale (V4.3 FAIL), heavy boxes
  placed with 15-27 mm error.
- Pallet top boards of the V4.4 world now match `config/ahead_simulator.yaml` (5 x 143 mm, pitch
  239.25 mm, 96 mm gap; placeholders, not an official spec). With the former 130 mm / 112 mm gap,
  S0001 box_05 (0.25 x 0.20 m) tipped 18-20 deg in Gazebo (COM 7 mm over the gap) but not in the
  browser. `scripts/compare_tipping_v44.py {bullet|gazebo}` drops the same box at COM offsets around
  a board edge: with identical geometry Gazebo 19.4 deg / Bullet 19.7 deg at -7 mm (engines agree;
  the mismatch was the geometry). After unifying, no offset tips in either; S0001 1-5 PASS
  (0.0-0.8 mm) in Gazebo and in the browser robot cell.
- Off-centre arrivals: `ARRIVAL_JITTER=1` (with `ARRIVAL_DY_M` 0.08, `ARRIVAL_YAW_DEG` 10, `ARRIVAL_SEED`)
  spawns each box at the inlet with a reproducible lateral offset / yaw (`arrival_jitter.tsv` in the
  run dir; V4.3 runner `V43_SPAWN_Y` / `V43_SPAWN_YAW`). S0001 1-5 (seed 20261009, offsets -37..+53 mm,
  yaw -5.8..+9.0 deg; the stopper does not square the box: up to -61 mm / 8.6 deg at PICK): all PASS,
  CCTV 0.7-1.2 mm, gripper camera 0.7-0.9 mm, placement 0.1-0.7 mm. No mechanical centring guide is
  needed for this line (mixed SKU: vision-corrected pick, as in mixed-case palletizing cells).

## 2026-10-09 (late afternoon): failure recovery, pallet re-check, performance

- `moveit_pick_place_v44.py` recovery: `Failure` = retry, `Fatal` = cell stops (operator).
  - Pick (PICK_ATTEMPTS 3): camera timeout / disagreement, no GRIPPED, box not following the cup,
    aborted motion -> vacuum off, retreat, READY, far CCTV + gripper camera again, grip again.
  - Controller abort (-4) on any PTP or straight line: re-planned once from the current state.
  - After the release the gripper camera (from the retreat pose, no extra move) measures the placed
    box (`box_perception_v44.py` target with `base_z`, `expect`, `expect_yaw`: SKU-sized rectangle
    fit, since same-coloured neighbours touch it in the image). Off by > 30 mm / 3 deg ->
    re-grip on the pallet and place again (REPLACE_ATTEMPTS 1); not confirmed (tilted / fallen) ->
    Fatal. Box dropped in transfer / lowering (box pose stream used as the vacuum sensor) -> Fatal.
  - The measured pose goes to `--result-json` and is what the cycle commits to the planner state
    (was Gazebo ground truth). Ground truth is still used for the final tolerance / disturbance
    report (simulation-only check).
  - Test faults: `PAC_FAULT=place_offset:box_01,suction_once:box_02` (40 mm off release; GRIPPED
    reported once without holding). Verified: re-place 40 mm -> 1.4 mm, suction miss -> re-grip ->
    0.7 mm (S0001 1-2, `logs/v44_generator_cycle/S0001_20261009_144920`).
- `stop_review_v44.sh` also stops leftover cycle nodes (an orphan suction node once attached the
  next box itself and masked the injected fault).
- Performance / robustness of the review launch:
  - `IGN_IP=127.0.0.1` exported (gz-transport on loopback): after a Wi-Fi -> hotspot change new
    processes could not discover Gazebo (next box not created).
  - NVIDIA PRIME render offload for Gazebo / RViz (`USE_NVIDIA=0` to skip); desktop default was the
    AMD iGPU. `GZ_GUI=0`: Gazebo server only (no window, ~2 CPU cores less).
  - RTF was 0.20 with another session's 16-worker evaluation at nice 0 and all cores at ~2.0 GHz
    (HP OMEN power limit, Tctl ~57 C). With that job at nice 15: 0.52 (GUI on), 0.28 (later run).
- Buffer rack reach (IK, rack modelled by its posts/shelves instead of the solid scene block): both
  top-shelf bays (z 1.5) reachable for contact and approach, except the far bay with a 0.34 m box
  at approach height. Lower shelves are not usable for top-down suction (0.58 m shelf pitch).

## 2026-10-09 (evening): stack height, gripper payload, buffer table, pallet change — IN PROGRESS

User decisions: gripper rated 30 kg (heavier boxes -> larger pad or support forks as an extension),
stack height nominal 1.5 m / up to 1.6 m allowed (above the deck), and no pallet change on the
first box without a slot: fill the remaining space with the following boxes and use the buffer
first; change the pallet only when the buffer is full (or the box does not fit a bay).

- `ahead_planner_bridge_v44.py plan`: nominal 1.5 m, then 1.6 m allowance (`PAC_STACK_NOMINAL_M`,
  `PAC_STACK_MAX_M` env for tests); candidates the arm cannot carry to are skipped (transfer TCP
  over the boxes under the PICK->slot corridor <= 1.95 m, IK grid 2026-10-09); `--pick-xy`;
  plan JSON has `height_mode`, `stack_limit_m`, `transfer_tcp_z`; failure = `PLAN FAIL: NO_SLOT (...)`
  with rejection-code counts.
- `moveit_pick_place_v44.py`: `--mass`, `--gripper pad|pad_xl|pad_fork` (30 / 45 / 60 kg rated;
  extension values are illustrative until the EOAT is selected), `--pick-from buffer --pick-pose`,
  `--place-on buffer`, `--extra-placed-json` (buffer boxes as obstacles); buffer moves are PTP with
  the 180-deg-symmetric yaw that turns the wrist least; READY with the least wrist turn; large wrist
  turns slowed; transfer height from the corridor, not the whole pallet; stale `placed_*` scene
  objects removed (pallet change).
- V4.4 world: buffer rack -> 2-bay buffer table (top shelf removed, posts cut to 0.97 m, shelf top
  0.9425 m; bays at x 1.235 / 1.665, y -0.55; bay fits <= 0.37 x 0.56 m). MoveIt scene updated.
- `run_generator_cycle_v44.sh`: payload check (> rated -> hold area with the extension hint),
  buffer put / re-plan buffered boxes after every placement / pallet change
  (`scripts/pallet_swap_v44.py`: copy of the pallet in the outbound lane, boxes moved onto it,
  new `pallet_N_state.json`, viewer reset) / buffered boxes placed at the end.
- Pallet re-check perception: known-size rectangle fit + edge refinement (~1 mm); accepted on the
  filled fraction only (a box on a larger lower box has a same-coloured band around it).

Status (not finished): with a 0.30 / 0.35 m test height, S0001 boxes 1-8 placed (camera-measured
deviation 1.1-4.0 mm), box_09 went to buffer bay 0 (first buffer use) and the camera check after
that release failed when the run was stopped for the git integration - cause NOT analysed yet.
Pallet change and picking back from the buffer have NOT been run in Gazebo yet. Before that
(single-path changes): failure recovery verified (re-place 40 mm -> 1.4 mm, suction miss -> re-grip
0.7 mm), off-centre arrivals S0001 1-5 PASS, unified pallet deck S0001 1-5 PASS.

## 2026-10-09 (night): block pallet, V4.6 two-camera layout — partly verified

Block pallet (Gazebo V4.4/V4.6 world + Bullet + browser viewer, same geometry, placeholders):
bottom boards 3 x 22 mm, 9 blocks 145 x 145 x 71 mm, 3 cross boards 145 x 22 mm (perpendicular to
the top boards), 5 top boards 143 mm / 96 mm gap / 35 mm; total 0.150 m. Bullet compound shapes hold
at most 16 children (the old 17-part pallet silently lost a block and the bottom boards): the pallet
is now 2 static bodies (`pallet_extra_ids`, all named PALLET). The viewer draws the pallet from the
same config (`PalletConfig.as_dict` exposes all ratios; `stringer_board_*` keys, default 0 = old).

V4.6 world (`scripts/make_world_v46.py` -> `ahead_workcell_v4_6_two_cam.sdf`, launch
`hdr50_workcell_v4_6.launch.py`; V4.4 unchanged):
- conveyor +2.185 m (64 rollers, 6.18 m); infeed + scale moved upstream: scale at the conveyor start
  (centre x -5.985), inlet x -6.635 (`PAC_SCALE_SHIFT_M=-2.185` shifts the V4.3 positions);
- two fixed cameras only: camera 1 `camera_scale_top` (straight down over the scale,
  `/pac/scale_camera/image`), camera 2 = pole CCTV raised to 3.6 m, hfov 1.6, 1920x1080 (sees PICK,
  the pallet and the buffer table); wrist camera removed (`wrist_camera:=false` xacro arg);
- buffer table next to PICK on the robot side, parallel to the conveyor (centre (-1.05, 0.40),
  `PAC_BUFFER_X/Y`).
- flow (`LAYOUT=v46 bash scripts/run_review_v44.sh ...` exports PAC_LAYOUT=v46, PAC_NO_WRIST=1, ...):
  camera-1 perception starts with the weighing (stable once the box rests on the scale); right after
  `WEIGHED` the planner runs while the box travels to PICK; the robot picks from the pole CCTV pose,
  checks the placed box with the pole CCTV after going back to READY, and uses the CCTV for the
  buffer table too.
Verified: world generation (64 rollers at 0.095 m), both camera images, xacro toggle, box_01 weighed
at the conveyor start, camera 1 OK, plan made while travelling, pick + transfer + release. NOT yet
verified (run interrupted by a reboot): CCTV check after release, buffer put/take, pallet change,
and whether the arm at READY hides the buffer from the CCTV.
How to verify: see `docs/jaesung/handoff_gpt/CONVERSATION_SUMMARY.md` ("How to verify").
