#!/usr/bin/env python3
"""V4.5 multi-box check: team planner -> robot IK check -> PyBullet pallet, box after box.

Gazebo holds a single test box, so stacking is verified in the standalone Bullet
simulator (pac_simulation.ahead_sim) with the SAME pieces the Gazebo mission uses:
  arrivals   : Jaesung dataset generator (ground_truth arrival order, simulator-owned)
  catalog    : Taehyeon build_catalog (McKee top load) + candidates.yaml limits
  planning   : Donghan plan_request (Taehyeon candidates/hard mask inside)
  robot check: pick_place_plan_v45 IK on the Gazebo workcell geometry (no execution)
  execution  : Bullet places the box; gravity/contact decide where it ends up
  state      : pac_common.StateManager (single writer): observation -> plan
               registration -> ExecutionResult with the settled Bullet pose ->
               commit (planned pose within xy 5 mm / z 3 mm / yaw 1 deg, else
               FAILED). Boxes without a candidate are recorded REJECTED (stands in
               for the high-level NG/BUFFER decision, which is not run here).

All results are SIMULATED.
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
from plan_placement_v45 import ensure_team_packages, team_limits  # noqa: E402
from pick_place_plan_v45 import plan_pick_place_yaw  # noqa: E402

GENERATOR_ROOT = ROOT / "tools" / "ahead_dataset_generator"
DEFAULT_DATASET = ROOT / "runs" / "v45_dataset"   # generated on first use (sample mode)
CONVEYOR_TOP_Z = 0.895          # Gazebo V4.2 PICK: 0.25 m box centre at z = 1.02
PICK_XY = (-1.06, 1.20)
SETTLE_S = 2.0
TOL_TILT_DEG = 5.0
STACK_DRIFT_M = 0.005          # earlier boxes may not move more than this


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

    ensure_team_packages()
    for extra in (ROOT / "tools/virtual_data", ROOT / "ros2_ws/src/pac_simulation"):
        if str(extra) not in sys.path:
            sys.path.insert(0, str(extra))
    from pac_common import BoxStatus, ExecutionResult, InventoryState, PalletState, Pose3D, Size3D, SystemState, plain
    from pac_common.adapters import candidate_from_json
    from pac_common.state_manager import StateManager
    from pac_common.planning import PlanningContext
    from pac_planning.config import load_config
    from pac_planning.planning_service import plan_request
    from pac_simulation.ahead_sim import AheadLiveSimulator, BoxSpec
    from pac_simulation.ahead_sim.config import load_config as load_sim_config
    from virtual_data import load_virtual_config
    from virtual_data.scenario_source import build_catalog, load_dataset, run_generator, stack_height_limit

    cand_cfg, _, _ = team_limits()
    virtual = load_virtual_config(ROOT / "config/taehyeon/virtual_data.yaml")
    planner_cfg = load_config(str(ROOT / "config/default.yaml"))
    if not (args.dataset / "manifest.json").exists():
        print(f">>> 데이터셋 생성 중 (재성 생성기, sample): {args.dataset}", flush=True)
        run_generator(GENERATOR_ROOT, ROOT / "ros2_ws/src/pac_common", args.dataset)
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
    gz = MB.gazebo_pallet()
    bullet_frame = MB.gazebo_pallet(top_center_world=(0.0, 0.0, 0.0))
    if abs(stack_h - gz.max_stack_height_m) > 1e-9:
        raise SystemExit(f"ERROR: dataset cargo height {stack_h} != team config {gz.max_stack_height_m}")
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
    sm = StateManager(SystemState(0, 0.0, PalletState(spec.pallet_id, Size3D(*gz.size_xy, stack_h), ()),
                                  InventoryState({}, dict(Counter(b.sku_id for b in spec.arrivals)))))
    anchor: dict = {}            # box_id -> committed world centre, for drift checks
    status, reason = "PASS", ""
    skipped: list = []
    print(f">>> 시나리오 {spec.scenario_id} ({spec.family}): {len(arrivals)}박스, 팔레트 "
          f"{gz.size_xy[0]:.2f}x{gz.size_xy[1]:.2f} m, 적재 높이 {stack_h:.2f} m, 총중량 {pallet_max_kg:.0f} kg",
          flush=True)

    for i, arrival in enumerate(arrivals, 1):
        size = (arrival.size.x, arrival.size.y, arrival.size.z)
        stamp = float(sm.snapshot.state_version + 1)
        box = replace(arrival, status=BoxStatus.READY_FOR_PICK, stamp_sec=stamp)
        state = sm.commit_observation(box, stamp)      # leaves anonymous stock here
        version = state.state_version
        row = {"index": i, "box_id": box.box_id, "sku_id": box.sku_id, "size_m": size,
               "weight_kg": box.weight_kg, "state_version": version}
        result = json.loads(plan_request(json.dumps(plain(state)), json.dumps(plain(context)), box.box_id, version,
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
            sm.commit_observation(replace(box, status=BoxStatus.REJECTED), stamp + 0.5)
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

        sm.register_plan(candidate_from_json(chosen))
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
        # Execution result with the measured pose; the State Manager decides what is committed.
        corner = MB.world_to_pallet_corner(pos, yaw_m, size, bullet_frame)
        corner["yaw"] += MB.gripper_yaw_delta(yaw_m - bullet_frame.yaw, corner["yaw"])  # keep measured yaw error
        outcome = sm.commit_execution(ExecutionResult(True, box.box_id, chosen["candidate_id"],
                                                      Pose3D(**corner), (), stamp + 0.5))
        row["state_commit"] = "PLACED" if outcome.placed else f"NOT_PLACED {[c.value for c in outcome.codes]}"
        problems = []
        if not outcome.placed:
            problems.append(f"state manager: {outcome.reason}")
        if tilt > TOL_TILT_DEG:
            problems.append(f"tilt {tilt:.1f} deg")
        if drift > STACK_DRIFT_M:
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

        anchor[box.box_id] = np.array([bx, by, bz])

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
