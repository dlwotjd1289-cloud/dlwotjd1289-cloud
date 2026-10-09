#!/usr/bin/env python3
"""V4.5: ask the team pipeline where to place the weighed box.

Stage 5: Taehyeon's CandidateBackend (5-1/5-2) + Donghan's PlacementPlanner (5-3..5-6)
through `plan_request` (the function behind the /pac/plan_placement ROS service).
Stage 6: pac_robot_check on every ranked candidate (config/taehyeon/robot_check.yaml).
Both run in mission_bridge_v45.plan_ranked. The stage-6 survivors are converted to
Gazebo world box centres for the robot node, best first. Output is still PLANNED;
only an ExecutionResult reports what happened.

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
ensure_team_packages = MB.ensure_team_packages


def team_limits():
    """(candidate config, pallet max load kg, top-load function) from the team configs."""
    cand, _, _, max_kg, top_load_n = MB.team_configs()
    return cand, max_kg, top_load_n


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

    cand_cfg, planner_cfg, robot_cfg, pallet_max_kg, top_load_n = MB.team_configs()

    print(">>> [계획 1/2] 계량 결과로 상태 snapshot 생성 중...", flush=True)
    stamp = time.time()
    state, context = MB.build_planning_request(
        box_id=args.box_id, sku_id=args.sku_id, size_m=args.box_size, measured_kg=args.measured_kg,
        pick_center_world=args.pick_center, state_version=args.state_version, stamp_sec=stamp,
        nominal_kg=args.nominal_kg, top_load_capacity_n=top_load_n(args.box_size),
        pallet_max_weight_kg=pallet_max_kg,
    )

    print(">>> [계획 2/2] 후보 생성(태현)·순위 평가(동한)·로봇 검증(6단계) 중...", flush=True)
    result = MB.plan_ranked(state, context, args.box_id, candidate_config=cand_cfg, planner_config=planner_cfg,
                            robot_config=robot_cfg, seed=args.seed)
    if not result["ranked"]:
        print(f"V4.5 PLAN FAIL: no valid candidate; rejected={result['rejected']}", flush=True)
        return 1
    if not result["executable"]:
        codes = sorted({c for cand in result["ranked"] for c in cand["robot"]["codes"]})
        print(f"V4.5 PLAN FAIL: {len(result['ranked'])} ranked, none passed stage 6 ({codes})", flush=True)
        return 1

    placements = []
    for cand in result["executable"]:
        x, y, z, yaw = MB.candidate_to_world(cand["target_pose"], args.box_size)
        placements.append({
            "candidate_id": cand["candidate_id"], "box_id": cand["box_id"],
            "base_state_version": cand["base_state_version"], "score": cand["score"],
            "target_pose_pallet": cand["target_pose"],
            "center_world": {"x": x, "y": y, "z": z, "yaw": yaw},
            "robot": cand["robot"]["details"],
        })
    robot_rejected = [{"candidate_id": c["candidate_id"], "codes": c["robot"]["codes"]}
                      for c in result["ranked"] if not c["robot"]["success"]]
    out = {
        "schema": "pac-v45-placement-1", "state_mode": result["state_mode"],
        "robot_validated": "pac_robot_check", "robot_rejected": robot_rejected, "box_id": args.box_id, "box_size_m": list(args.box_size),
        "measured_kg": args.measured_kg, "pallet": MB.gazebo_pallet().__dict__,
        "placements": placements, "state": state, "context": context,
        "diagnostics": result["diagnostics"],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=1, default=list))
    best = placements[0]
    c = best["center_world"]
    print(f"V4.5 PLAN OK: {len(placements)} executable of {len(result['ranked'])} ranked; best {best['candidate_id']} -> world centre "
          f"({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f}) yaw {c['yaw']:.2f} rad "
          f"[{result['diagnostics']['model_status']}, {result['diagnostics']['planning_time_sec']:.2f} s]",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
