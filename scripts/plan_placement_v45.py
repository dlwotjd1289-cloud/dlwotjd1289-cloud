#!/usr/bin/env python3
"""V4.5: ask the team planner where to place the weighed box.

Runs Taehyeon's CandidateBackend (5-1/5-2) + Donghan's PlacementPlanner (5-3..5-6)
through Donghan's own `plan_request` entry point -- the same function behind the
/pac/plan_placement ROS service -- and converts the PLANNED ranking to Gazebo
world box centres for the robot node. Output is still PLANNED: the robot node
validates IK per candidate and only an ExecutionResult reports what happened.

Limits: pallet footprint / cargo height / total mass from config/default.yaml,
carton top load from Taehyeon's McKee model (config/taehyeon/candidates.yaml).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mission_bridge_v45 as MB  # noqa: E402

ROOT = HERE.parent
PACKAGES = ("pac_common", "pac_planning", "pac_candidates")


def ensure_team_packages():
    """Make the monorepo packages importable when not run from colcon/pytest."""
    for name in reversed(PACKAGES):
        path = str(ROOT / "ros2_ws" / "src" / name)
        if path not in sys.path:
            sys.path.insert(0, path)


def team_limits():
    """(candidate config, pallet max load kg, top-load function) from the team configs."""
    ensure_team_packages()
    from pac_candidates import load_candidate_config
    from pac_candidates.loads import mckee_capacity_n
    from pac_common.config import load_common_config

    cfg = load_candidate_config(str(ROOT / "config/taehyeon/candidates.yaml"))
    lm = cfg.constraints.load_model

    def top_load_n(size_m):
        return round(mckee_capacity_n(size_m[0], size_m[1], lm.ect_n_per_m, lm.board_thickness_m,
                                      lm.safety_factor), 3)

    return cfg, load_common_config().pallet.max_load_kg, top_load_n


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

    cand_cfg, pallet_max_kg, top_load_n = team_limits()
    from pac_planning.config import load_config
    from pac_planning.planning_service import plan_request

    print(">>> [계획 1/2] 계량 결과로 상태 snapshot 생성 중...", flush=True)
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
        load_config(str(ROOT / "config/default.yaml")),
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
        "measured_kg": args.measured_kg, "pallet": MB.gazebo_pallet().__dict__,
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
