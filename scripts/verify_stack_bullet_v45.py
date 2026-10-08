#!/usr/bin/env python3
"""V4.5 multi-box check: team planner -> robot IK check -> PyBullet pallet, box after box.

Gazebo holds a single test box, so stacking is verified in the standalone Bullet
simulator (pac_simulation.ahead_sim) with the SAME pieces the Gazebo mission uses:
  arrivals   : Jaesung dataset generator (ground_truth arrival order, simulator-owned)
  catalog    : Taehyeon build_catalog (McKee top load) + candidates.yaml limits
  planning   : Donghan plan_request (Taehyeon candidates/hard mask inside)
  robot check: pick_place_plan_v45 IK on the Gazebo workcell geometry (no execution)
  execution  : Bullet places the box; gravity/contact decide where it ends up
  state      : if the settled Bullet pose matches the plan within the commit
               tolerance, the PLANNED pose is committed as PlacedBox and the next
               state_version is built (test-only state keeping; the real State
               Manager is owned elsewhere). Raw contact poses are not committed:
               they sink ~0.01 mm into supports and the planner rejects any
               existing box outside the pallet / overlapping by more than 1e-8 m.

All results are SIMULATED. Run without the ROS overlay on PYTHONPATH.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import hdr50_kinematics as K  # noqa: E402
import mission_bridge_v45 as MB  # noqa: E402
from plan_placement_v45 import _team_paths, team_limits  # noqa: E402
from pick_place_plan_v45 import plan_pick_place_yaw  # noqa: E402

DEFAULT_DATASET = (Path.home() / "AHEAD/pac-mission1-shared/pac-mission1-shared/tools/"
                   "ahead_dataset_generator/generated/sample_dataset")
CONVEYOR_TOP_Z = 0.895          # Gazebo V4.2 PICK: 0.25 m box centre at z = 1.02
PICK_XY = (-1.06, 1.20)
SETTLE_S = 2.0
TOL_TILT_DEG = 5.0
COMMIT_XY_M, COMMIT_Z_M, COMMIT_YAW_DEG = 0.005, 0.003, 1.0   # measured vs planned, also stack drift


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    ap.add_argument("--scenario", default="S0001")
    ap.add_argument("--max-boxes", type=int, default=0, help="0 = all arrivals")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--skip-unplaceable", action="store_true",
                    help="record a box with no planner/IK candidate as SKIPPED and continue "
                         "(stands in for the high-level BUFFER/NG decision, which is not run here)")
    ap.add_argument("--output", type=Path, default=ROOT / "logs" / f"v45_bullet_{time.strftime('%Y%m%d_%H%M%S')}")
    args = ap.parse_args()

    planner_root, team_root = _team_paths()
    sys.path.insert(0, str(team_root / "tools/virtual_data"))
    sys.path.insert(0, str(ROOT / "ros2_ws/src/pac_simulation"))
    from pac_common import PlacedBox, Pose3D, plain
    from pac_common.planning import PlanningContext
    from pac_planning.config import load_config
    from pac_planning.planning_service import plan_request
    from pac_simulation.ahead_sim import AheadLiveSimulator, BoxSpec
    from pac_simulation.ahead_sim.config import load_config as load_sim_config
    from virtual_data import load_virtual_config
    from virtual_data.scenario_source import build_catalog, load_dataset, stack_height_limit

    cand_cfg, _, _ = team_limits(team_root)
    virtual = load_virtual_config(team_root / "config/taehyeon/virtual_data.yaml")
    planner_cfg = load_config(str(planner_root / "config/default.yaml"))
    dataset = load_dataset(args.dataset)
    spec = next((s for s in dataset.scenarios if s.scenario_id == args.scenario), None)
    if spec is None:
        raise SystemExit(f"ERROR: unknown scenario {args.scenario}")
    arrivals = spec.arrivals[:args.max_boxes] if args.max_boxes else spec.arrivals

    pallet_max_kg = spec.max_load_kg if spec.max_load_kg is not None else virtual.pallet.default_max_load_kg
    context = PlanningContext(
        catalog=build_catalog(dataset.sku_ranges, virtual, cand_cfg.constraints.load_model),
        pallet_max_weight_kg=float(pallet_max_kg), buffer_capacity=0,
    )
    stack_h = stack_height_limit(spec, virtual)
    gz = MB.GazeboPallet(max_stack_height_m=stack_h)
    bullet_frame = MB.GazeboPallet(top_center_world=(0.0, 0.0, 0.0), size_xy=gz.size_xy,
                                   max_stack_height_m=stack_h)
    if tuple(spec.pallet_xy) != gz.size_xy:
        raise SystemExit(f"ERROR: dataset pallet {spec.pallet_xy} != workcell pallet {gz.size_xy}")

    sim_cfg, _ = load_sim_config(ROOT / "config/ahead_simulator.yaml")
    if (sim_cfg.pallet.length_m, sim_cfg.pallet.width_m) != gz.size_xy:
        raise SystemExit("ERROR: Bullet simulator pallet differs from planner pallet")
    sim = AheadLiveSimulator(sim_cfg)
    steps_per_s = sim_cfg.physics.physics_hz

    args.output.mkdir(parents=True, exist_ok=True)
    log = (args.output / "boxes.jsonl").open("w", encoding="utf-8")
    # Anonymous stock covers the whole scenario even when --max-boxes stops early,
    # so a shortened run sees the same future inventory as the full one.
    remaining = Counter(b.sku_id for b in spec.arrivals)
    placed: list = []            # committed PlacedBox (planned pose, verified by Bullet)
    anchor: dict = {}            # box_id -> committed world centre, for drift checks
    version = 0
    status, reason = "PASS", ""
    skipped: list = []
    print(f">>> 시나리오 {spec.scenario_id} ({spec.family}): {len(arrivals)}박스, 팔레트 "
          f"{gz.size_xy[0]:.2f}x{gz.size_xy[1]:.2f} m, 적재 높이 {stack_h:.2f} m, 총중량 {pallet_max_kg:.0f} kg",
          flush=True)

    for i, arrival in enumerate(arrivals, 1):
        remaining[arrival.sku_id] -= 1   # the arrived box is tracked, no longer anonymous stock
        size = (arrival.size.x, arrival.size.y, arrival.size.z)
        box = replace(arrival, status=type(arrival.status)("READY_FOR_PICK"), stamp_sec=float(version))
        state = {
            "state_version": version, "stamp_sec": float(version),
            "pallet": {"pallet_id": spec.pallet_id,
                       "size": {"x": gz.size_xy[0], "y": gz.size_xy[1], "z": stack_h},
                       "boxes": [plain(b) for b in placed]},
            "inventory": {"tracked_boxes": {box.box_id: plain(box)},
                          "remaining_by_sku": {k: v for k, v in remaining.items() if v > 0}},
        }
        row = {"index": i, "box_id": box.box_id, "sku_id": box.sku_id, "size_m": size,
               "weight_kg": box.weight_kg, "state_version": version}
        result = json.loads(plan_request(json.dumps(state), json.dumps(plain(context)), box.box_id, version,
                                         cand_cfg, planner_cfg, seed=args.seed + i, use_time_budget=False))
        row["ranked"] = len(result["ranked"])
        row["planner_rejected"] = dict(Counter(code for v in result["rejected"].values() for code in v["codes"]))

        # Robot validation on the Gazebo workcell geometry (IK only).
        chosen, ik_rejected = None, []
        pick = (PICK_XY[0], PICK_XY[1], CONVEYOR_TOP_Z + size[2] / 2)
        for cand in result["ranked"]:
            x, y, z, yaw = MB.candidate_to_world(cand["target_pose"], size, gz)
            try:
                plan_pick_place_yaw(K.HOME, pick, (x, y, z), size, MB.gripper_yaw_delta(yaw, 0.0))
            except ValueError as exc:
                ik_rejected.append({"candidate_id": cand["candidate_id"], "reason": str(exc)})
                continue
            chosen = cand
            break
        row["ik_rejected"] = ik_rejected
        if chosen is None and args.skip_unplaceable:
            why = (f"no candidate {row['planner_rejected']}" if not result["ranked"] else "all candidates IK_FAIL")
            row["result"] = "SKIPPED: " + why
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
            skipped.append(box.box_id)
            print(f"    [{i:02d}] {box.box_id} {box.sku_id} {size}: 건너뜀 — {why}", flush=True)
            continue
        if chosen is None:
            status = "STOP"
            reason = (f"planner returned no candidate {row['planner_rejected']}" if not result["ranked"]
                      else "all candidates IK_FAIL")
            row["result"] = reason
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
            print(f"    [{i:02d}] {box.box_id}: 중단 — {reason}", flush=True)
            break

        bx, by, bz, byaw = MB.candidate_to_world(chosen["target_pose"], size, bullet_frame)
        cap = context.catalog[box.sku_id].top_load_capacity_n
        try:
            sim.place_box(BoxSpec(box.box_id, size, box.weight_kg, (bx, by, bz), byaw,
                                  source="v45_planner", max_top_load_n=cap))
        except ValueError as exc:
            status, reason = "FAIL", f"{box.box_id}: Bullet rejected placement ({exc})"
            row["result"] = reason
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
            print(f"    [{i:02d}] 실패 — {reason}", flush=True)
            break
        sim.step(int((SETTLE_S - 0.5) * steps_per_s))
        for _ in range(10):                      # fill the 0.5 s contact-force average window
            sim.step(int(0.05 * steps_per_s))
            snap = sim.snapshot()

        boxes = {b["id"]: b for b in snap["boxes"]}
        b = boxes[box.box_id]
        pos = np.array(b["position_m"])
        roll, pitch, yaw_m = b["euler_rad"]
        yaw_err = abs(math.degrees(MB.gripper_yaw_delta(byaw, yaw_m)))
        err_xy = float(np.hypot(pos[0] - bx, pos[1] - by))
        err_z = float(pos[2] - bz)
        tilt = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
        drift = max((float(np.linalg.norm(np.array(boxes[k]["position_m"]) - v)) for k, v in anchor.items()),
                    default=0.0)
        m = snap["metrics"]
        overloaded = snap["strength"]["overloaded_box_ids"]
        row.update(candidate_id=chosen["candidate_id"], score=chosen["score"],
                   target_pallet=chosen["target_pose"], err_xy_mm=err_xy * 1000, err_z_mm=err_z * 1000,
                   tilt_deg=tilt, yaw_err_deg=yaw_err, stack_drift_mm=drift * 1000, moving=m["moving_box_ids"],
                   overloaded=overloaded, height_m=m["current_height_m"], com_offset_m=m["com_xy_offset_m"],
                   utilization=m["allowed_volume_utilization"])
        problems = []
        if err_xy > COMMIT_XY_M or abs(err_z) > COMMIT_Z_M or yaw_err > COMMIT_YAW_DEG:
            problems.append(f"pose differs from plan: xy {err_xy*1000:.1f} mm z {err_z*1000:+.1f} mm "
                            f"yaw {yaw_err:.1f} deg (re-perception needed before next plan)")
        if tilt > TOL_TILT_DEG:
            problems.append(f"tilt {tilt:.1f} deg")
        if drift > COMMIT_XY_M:
            problems.append(f"stack moved {drift*1000:.0f} mm")
        if m["moving_box_ids"]:
            problems.append(f"still moving {m['moving_box_ids']}")
        if overloaded:
            problems.append(f"overloaded {overloaded}")
        row["result"] = "; ".join(problems) or "OK"
        log.write(json.dumps(row, ensure_ascii=False) + "\n")
        print(f"    [{i:02d}] {box.box_id} {box.sku_id} {box.weight_kg:5.2f} kg -> {chosen['candidate_id']} "
              f"(IK 탈락 {len(ik_rejected)}) | 오차 xy {err_xy*1000:4.1f} mm z {err_z*1000:+5.1f} mm "
              f"기울기 {tilt:4.2f}° 기존스택이동 {drift*1000:4.1f} mm | 높이 {m['current_height_m']:.2f} m "
              f"| {row['result']}", flush=True)
        if problems:
            status, reason = "FAIL", f"{box.box_id}: " + "; ".join(problems)
            break

        # Test-only state commit: verified execution -> planned pose -> next snapshot version.
        placed.append(PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg,
                                Pose3D(**chosen["target_pose"])))
        anchor[box.box_id] = np.array([bx, by, bz])
        version += 1

    snap = sim.snapshot()
    summary = {
        "scope": "SIMULATED_BULLET", "robot_execution": "IK_ONLY", "status": status, "reason": reason,
        "scenario_id": spec.scenario_id, "arrivals": len(arrivals), "placed": len(anchor), "skipped": skipped,
        "final_height_m": snap["metrics"]["current_height_m"],
        "volume_utilization": snap["metrics"]["allowed_volume_utilization"],
        "com_xy_offset_m": snap["metrics"]["com_xy_offset_m"],
        "pallet": {"size_xy": gz.size_xy, "stack_height_m": stack_h, "max_load_kg": pallet_max_kg},
    }
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    log.close()
    sim.close()
    print(f"\nV4.5 BULLET {status}: {len(anchor)}/{len(arrivals)} placed, {len(skipped)} skipped, height "
          f"{summary['final_height_m']:.2f} m, volume {summary['volume_utilization']*100:.1f}% "
          f"{('— ' + reason) if reason else ''}\n    logs: {args.output}", flush=True)
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
