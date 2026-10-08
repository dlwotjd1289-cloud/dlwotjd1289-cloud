#!/usr/bin/env python3
"""V4.5: ask the team planner (pac-mission1-shared) where to place the weighed box.

Runs Taehyeon's CandidateBackend (5-1/5-2) + Donghan's PlacementPlanner (5-3..5-6)
through Donghan's own `plan_request` entry point -- the same function behind the
/pac/plan_placement ROS service -- and converts the PLANNED ranking to Gazebo
world box centres for the robot node. Output is still PLANNED: the robot node
validates IK per candidate and only an ExecutionResult reports what happened.

Run WITHOUT this scaffold's ROS overlay on PYTHONPATH (the overlay contains a
different `pac_common`); run_mission_v45.sh uses `env -u PYTHONPATH`.

Environment:
  PAC_PLANNER_ROOT  .../pac-mission1-shared (branch feature/donghan-placement-planner)
  PAC_TEAM_ROOT     checkout of branch claude/pensive-pasteur-dwbu3g (pac_candidates)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mission_bridge_v45 as MB  # noqa: E402

DEFAULT_ROOT = Path.home() / "AHEAD" / "team_checkouts"


def _team_paths():
    planner = Path(os.environ.get("PAC_PLANNER_ROOT", DEFAULT_ROOT / "planner" / "pac-mission1-shared"))
    team = Path(os.environ.get("PAC_TEAM_ROOT", DEFAULT_ROOT / "candidates"))
    for p in (planner / "ros2_ws/src/pac_common", planner / "ros2_ws/src/pac_planning",
              team / "ros2_ws/src/pac_candidates"):
        if not p.is_dir():
            raise SystemExit(f"ERROR: team package not found: {p} (set PAC_PLANNER_ROOT / PAC_TEAM_ROOT)")
        sys.path.insert(0, str(p))
    return planner, team


def team_limits(team_root: Path):
    """Pallet total-mass limit and McKee top-load model from Taehyeon's candidates.yaml.

    Same source the team's virtual-data pipeline uses (build_catalog), so the
    Gazebo box and the Bullet multi-box check see identical limits.
    """
    from pac_candidates import load_candidate_config
    from pac_candidates.loads import mckee_capacity_n

    cfg = load_candidate_config(str(team_root / "config/taehyeon/candidates.yaml"))
    lm = cfg.constraints.load_model

    def top_load_n(size_m):
        return round(mckee_capacity_n(size_m[0], size_m[1], lm.ect_n_per_m, lm.board_thickness_m,
                                      lm.safety_factor), 3)

    return cfg, cfg.constraints.default_pallet_max_weight_kg, top_load_n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--measured-kg", type=float, required=True, help="V4.3 scale reading")
    ap.add_argument("--pick-center", type=float, nargs=3, default=(-1.06, 1.20, 1.02),
                    help="box centre at PICK, world m (only recorded in BoxState.pose)")
    ap.add_argument("--box-size", type=float, nargs=3, default=(0.40, 0.30, 0.25))
    ap.add_argument("--box-id", default="B001")
    ap.add_argument("--sku-id", default="V43_TEST_5KG")
    ap.add_argument("--nominal-kg", type=float, default=5.0)
    ap.add_argument("--state-version", type=int, default=1)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", required=True, help="placement JSON for run_mission_v45.py")
    args = ap.parse_args()

    planner_root, team_root = _team_paths()
    from pac_planning.config import load_config
    from pac_planning.planning_service import plan_request

    print(">>> [계획 1/2] 계량 결과로 상태 snapshot 생성 중...", flush=True)
    cand_cfg, pallet_max_kg, top_load_n = team_limits(team_root)
    stamp = time.time()
    state, context = MB.build_planning_request(
        box_id=args.box_id, sku_id=args.sku_id, size_m=args.box_size, measured_kg=args.measured_kg,
        pick_center_world=args.pick_center, state_version=args.state_version, stamp_sec=stamp,
        nominal_kg=args.nominal_kg, top_load_capacity_n=top_load_n(args.box_size),
        pallet_max_weight_kg=pallet_max_kg,
    )

    print(">>> [계획 2/2] 후보 생성(태현)·순위 평가(동한) 중...", flush=True)
    result = json.loads(plan_request(
        json.dumps(state), json.dumps(context), args.box_id, args.state_version,
        cand_cfg,
        load_config(str(planner_root / "config/default.yaml")),
        seed=args.seed, use_time_budget=False,
    ))
    if not result["ranked"]:
        print(f"V4.5 PLAN FAIL: no valid candidate; rejected={result['rejected']}", flush=True)
        return 1

    placements = []
    for cand in result["ranked"]:
        x, y, z, yaw = MB.candidate_to_world(cand["target_pose"], args.box_size)
        placements.append({
            "candidate_id": cand["candidate_id"], "box_id": cand["box_id"],
            "base_state_version": cand["base_state_version"], "score": cand["score"],
            "target_pose_pallet": cand["target_pose"],
            "center_world": {"x": x, "y": y, "z": z, "yaw": yaw},
        })
    out = {
        "schema": "pac-v45-placement-1", "state_mode": result["state_mode"],
        "requires_robot_validation": True, "box_id": args.box_id, "box_size_m": list(args.box_size),
        "measured_kg": args.measured_kg, "pallet": MB.GazeboPallet().__dict__,
        "placements": placements, "state": state, "context": context,
        "diagnostics": result["diagnostics"],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=list))
    best = placements[0]
    c = best["center_world"]
    print(f"V4.5 PLAN OK: {len(placements)} ranked; best {best['candidate_id']} -> world centre "
          f"({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f}) yaw {c['yaw']:.2f} rad "
          f"[{result['diagnostics']['model_status']}, {result['diagnostics']['planning_time_sec']:.2f} s]",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
