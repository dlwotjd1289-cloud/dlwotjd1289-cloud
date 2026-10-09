#!/usr/bin/env python3
"""Top-view perception + scale -> team stages 5/6 -> robot slot, and stage 8 commit (V4.4 cycle).

  plan:   python3 scripts/ahead_planner_bridge_v44.py plan --state S.json --box-id box_03 \
              --mass 5.0 --perception P.json --remaining 5 --out PLAN.json
          prints "x y z yaw" (world frame box centre for moveit_pick_place_v44.py --slot/--slot-yaw)
  commit: python3 scripts/ahead_planner_bridge_v44.py commit --state S.json --box-id box_03 \
              --mass 5.0 --world X Y Z YAW --planned PLAN.json
          records the ACTUALLY placed box (Gazebo pose) as the next ACTUAL pallet state.

Only team implementations decide (mission_bridge_v45.plan_ranked):
  5   Taehyeon CandidateBackend (5-1/5-2) + Donghan PlacementPlanner (5-3..5-6)
  6   pac_robot_check on the team cell (config/taehyeon/robot_check.yaml)
  8   pac_common.StateManager commit rule (planned pose within xy 5 mm / z 3 mm / yaw 1 deg,
      otherwise the measured pose)
Pallet footprint, cargo height and total mass come from config/default.yaml, the pallet frame
(origin at the deck corner farthest from the robot) from config/workcell.yaml, carton top load
from Taehyeon's McKee model. The state file keeps the planner frame ("pallet", lower corner).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mission_bridge_v45 as MB  # noqa: E402

MB.ensure_team_packages()
QUARTER = math.pi / 2

# Robot placement clearance: the planner packs boxes flush (0 gap); a suction robot lowering a box
# between neighbours needs a small gap, so the planner sees each box this much larger in x/y.
# 10 mm (5 mm gap) was too tight for a 13.8 kg box lowered at speed 0.2 -> 20 mm (10 mm gap).
PLACE_CLEARANCE_M = 0.020
# Transfer: the box is carried from PICK to the slot above the boxes under the corridor.
# pac_robot_check covers the place pose and the vertical approach, not this transfer, so the
# carry height is still bounded here (HDR50-22 on the pedestal, IK grid 2026-10-09).
REACH_TCP_MAX_Z = 1.95
PICK_TOP_Z = 0.895                  # conveyor top (box bottom at PICK)
APPROACH_M, STACK_CLEAR_M = 0.20, 0.10
# SKU master data. Default: the Gazebo 5 kg test box. --catalog loads the AHEAD dataset generator
# sku_catalog instead; top load is Taehyeon's McKee model for every SKU.
CATALOG = {
    "GZ_TEST_40x30x25": {"sku_id": "GZ_TEST_40x30x25", "size": {"x": 0.40, "y": 0.30, "z": 0.25},
                          "weight_kg": 5.0, "allowed_yaws_rad": [0.0, QUARTER]},
}
FOOTPRINT_TOL_M = 0.03


def load_generator_catalog(path: str) -> None:
    import yaml
    cfg = yaml.safe_load(open(path))
    CATALOG.clear()
    for s in cfg["sku_catalog"]:
        size = {k: float(s["size_m"][k]) for k in ("x", "y", "z")}
        CATALOG[s["sku_id"]] = {
            "sku_id": s["sku_id"], "size": size,
            "weight_kg": round((float(s["weight_min_kg"]) + float(s["weight_max_kg"])) / 2, 3),
            "allowed_yaws_rad": [float(v) for v in s.get("allowed_yaws_rad", [0.0, QUARTER])]}


def load_state(path: Path) -> dict:
    if path.exists():
        return json.loads(path.read_text())
    return {"state_version": 1, "stamp_sec": 0.0, "placed": []}


def match_sku(length: float, width: float) -> str:
    meas = sorted((length, width))
    for sku, spec in CATALOG.items():
        ref = sorted((spec["size"]["x"], spec["size"]["y"]))
        if all(abs(a - b) <= FOOTPRINT_TOL_M for a, b in zip(meas, ref)):
            return sku
    raise SystemExit(f"PLAN FAIL: no SKU matches the measured footprint {length:.3f} x {width:.3f} m")


def planner_catalog(top_load_n) -> dict:
    out = {}
    for sku, spec in CATALOG.items():
        s = dict(spec)
        s["size"] = {"x": spec["size"]["x"] + PLACE_CLEARANCE_M, "y": spec["size"]["y"] + PLACE_CLEARANCE_M,
                     "z": spec["size"]["z"]}
        s["top_load_capacity_n"] = top_load_n((spec["size"]["x"], spec["size"]["y"]))
        out[sku] = s
    return out


def snapshot(st: dict, pallet: MB.GazeboPallet, tracked: dict, remaining) -> dict:
    return {"state_version": st["state_version"], "stamp_sec": st["stamp_sec"],
            "pallet": {"pallet_id": pallet.pallet_id,
                       "size": {"x": pallet.size_xy[0], "y": pallet.size_xy[1], "z": pallet.max_stack_height_m},
                       "boxes": st["placed"]},
            "inventory": {"tracked_boxes": tracked, "remaining_by_sku": remaining}}


def placed_world_aabbs(st: dict, pallet: MB.GazeboPallet):
    """World-frame (xmin, xmax, ymin, ymax, ztop) of the committed boxes."""
    out = []
    for b in st["placed"]:
        sz = b["size"]
        x, y, z, yaw = MB.candidate_to_world(b["pose"], (sz["x"], sz["y"], sz["z"]), pallet)
        dx, dy, dz = MB.rotated_dims((sz["x"], sz["y"], sz["z"]), MB.wrap_pi(yaw))
        out.append((x - dx / 2, x + dx / 2, y - dy / 2, y + dy / 2, z + dz / 2))
    return out


def corridor_top(aabbs, start_xy, end_xy, half_width, floor_z):
    """Highest box top whose footprint comes within half_width of the segment start->end."""
    import numpy as np
    a, b = np.array(start_xy, float), np.array(end_xy, float)
    top = floor_z
    for x0, x1, y0, y1, zt in aabbs:
        # distance segment <-> rectangle, sampled along the segment (5 cm)
        n = max(2, int(np.linalg.norm(b - a) / 0.05) + 1)
        for t in np.linspace(0.0, 1.0, n):
            q = a + t * (b - a)
            dx = max(x0 - q[0], 0.0, q[0] - x1)
            dy = max(y0 - q[1], 0.0, q[1] - y1)
            if math.hypot(dx, dy) <= half_width:
                top = max(top, zt)
                break
    return top


def transfer_tcp_z(st, pallet, pick_xy, slot_xy, size):
    h = size["z"]
    half = math.hypot(size["x"], size["y"]) / 2 + 0.05
    top = corridor_top(placed_world_aabbs(st, pallet), pick_xy, slot_xy, half, pallet.top_center_world[2])
    return max(PICK_TOP_Z + h + APPROACH_M, top + STACK_CLEAR_M + h)


def cmd_plan(a) -> int:
    st = load_state(Path(a.state))
    per = json.loads(Path(a.perception).read_text())
    if per.get("status") != "OK":
        raise SystemExit(f"PLAN FAIL: perception status {per.get('status')}")
    if a.catalog:
        load_generator_catalog(a.catalog)
    if a.sku:
        # Identity from the arrival event (label / WMS); the camera footprint must agree with it.
        sku = a.sku
        ref = sorted((CATALOG[sku]["size"]["x"], CATALOG[sku]["size"]["y"]))
        meas = sorted((per["length"], per["width"]))
        if any(abs(m - r) > FOOTPRINT_TOL_M for m, r in zip(meas, ref)):
            raise SystemExit(f"PLAN FAIL: camera footprint {meas[1]:.3f} x {meas[0]:.3f} m does not match SKU "
                             f"{sku} {ref[1]:.3f} x {ref[0]:.3f} m")
    else:
        sku = match_sku(per["length"], per["width"])
    remaining = json.loads(a.remaining_json) if a.remaining_json else {sku: max(0, a.remaining)}
    cand_cfg, planner_cfg, robot_cfg, max_load_kg, top_load_n = MB.team_configs()
    model = None
    if a.model:
        from pac_planning.model import DualHeadRanker
        model = DualHeadRanker.load(a.model)
    pallet = MB.gazebo_pallet()
    catalog = planner_catalog(top_load_n)
    spec = catalog[sku]
    current = {"box_id": a.box_id, "sku_id": sku, "size": spec["size"], "weight_kg": a.mass,
               "pose": {"frame_id": "conveyor", "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
               "allowed_yaws_rad": spec["allowed_yaws_rad"], "status": "READY_FOR_PICK",
               "confidence": 1.0, "stamp_sec": st["stamp_sec"], "source": "CCTV_PICK_TOPVIEW+SCALE"}
    state = snapshot(st, pallet, {a.box_id: current}, remaining)
    context = {"catalog": catalog, "pallet_max_weight_kg": max_load_kg, "buffer_capacity": 0}
    result = MB.plan_ranked(state, context, a.box_id, candidate_config=cand_cfg, planner_config=planner_cfg,
                            robot_config=robot_cfg, seed=a.seed, model=model, use_time_budget=True)
    pick_xy = (per.get("x", -1.03), per.get("y", 1.20)) if a.pick_xy is None else tuple(a.pick_xy)
    true_size = CATALOG[sku]["size"]
    chosen, notes, carry = None, [], 0
    for cand in result["executable"]:          # best first; stage 6 passed, transfer still checked
        world = MB.candidate_to_world(cand["target_pose"], (spec["size"]["x"], spec["size"]["y"], spec["size"]["z"]),
                                      pallet)
        tz = transfer_tcp_z(st, pallet, pick_xy, world[:2], true_size)
        if tz <= REACH_TCP_MAX_Z:
            chosen = (cand, world, tz)
            break
        carry += 1
    stage6 = len(result["ranked"]) - len(result["executable"])
    if stage6:
        codes = sorted({c for cand in result["ranked"] for c in cand["robot"]["codes"]})
        notes.append(f"{stage6} candidate(s) rejected by stage 6 {codes}")
    if carry:
        notes.append(f"{carry} candidate(s) out of reach for the transfer")
    if not result["ranked"]:
        from collections import Counter
        codes = Counter(c for v in result["rejected"].values() for c in v["codes"])
        notes.append(f"no valid candidate ({len(result['rejected'])} rejected: {dict(codes)})")
    if chosen is None:
        raise SystemExit("PLAN FAIL: NO_SLOT (" + "; ".join(notes) + ")")
    cand, world, tz = chosen
    t = cand["target_pose"]
    out = {"box_id": a.box_id, "sku_id": sku, "mass_kg": a.mass, "state_version": st["state_version"],
           "candidate_id": cand["candidate_id"], "score": cand["score"],
           "target_corner_pallet": [t["x"], t["y"], t["z"]], "yaw_rad": t["yaw"],
           "planner_size": spec["size"], "slot_world": [round(v, 4) for v in world],
           "n_ranked": len(result["ranked"]), "n_executable": len(result["executable"]),
           "n_rejected": len(result["rejected"]), "stack_limit_m": pallet.max_stack_height_m,
           "transfer_tcp_z": round(tz, 3), "notes": notes, "robot": cand["robot"]["details"],
           "planning_time_sec": result["diagnostics"].get("planning_time_sec"),
           "perception": per, "planner": "team plan_request (pac_candidates + pac_planning) + pac_robot_check",
           "model": a.model or "heuristic (no learned model)"}
    Path(a.out).write_text(json.dumps(out, indent=2, default=float))
    print(f"{world[0]:.4f} {world[1]:.4f} {world[2]:.4f} {t['yaw']:.4f}")   # yaw: planner yaw (box is 180 deg symmetric)
    return 0


def cmd_commit(a) -> int:
    from pac_common import BoxState, BoxStatus, Pose3D, Size3D, StateManager, plain
    from pac_common.adapters import state_from_json
    from pac_common.state_manager import within_tolerance

    if a.catalog:
        load_generator_catalog(a.catalog)
    path = Path(a.state)
    st = load_state(path)
    pallet = MB.gazebo_pallet()
    x, y, z, yaw = a.world
    local = MB.wrap_pi(yaw - pallet.yaw)
    if abs(local - round(local / QUARTER) * QUARTER) > math.radians(5):
        raise SystemExit(f"COMMIT FAIL: placed yaw {math.degrees(yaw):.1f} deg is not axis-aligned")
    true = CATALOG[a.sku]["size"]
    true_size = (true["x"], true["y"], true["z"])
    plan = json.loads(Path(a.planned).read_text()) if a.planned else None
    # The planner reasons with clearance-inflated boxes; the state keeps that size when the box
    # landed on the plan (stage 8 commit rule) so the planner model stays self-consistent, and
    # the true size at the measured pose otherwise (the deviation is not hidden).
    planned = None
    size = true_size
    if plan is not None:
        ps = plan["planner_size"]
        inflated = (ps["x"], ps["y"], ps["z"])
        planned = Pose3D("pallet", *plan["target_corner_pallet"], yaw=plan["yaw_rad"])
        if within_tolerance(Pose3D(**MB.world_to_pallet_corner((x, y, z), yaw, inflated, pallet)), planned):
            size = inflated
    measured = MB.world_to_pallet_corner((x, y, z), yaw, size, pallet)
    measured["z"] = max(0.0, measured["z"])
    tracked = {}
    for b in st["placed"]:
        tracked[b["box_id"]] = {**b, "allowed_yaws_rad": [0.0, QUARTER], "status": "PLACED", "confidence": 1.0,
                                "stamp_sec": st["stamp_sec"], "source": "STATE_FILE"}
    sm = StateManager(state_from_json(snapshot(st, pallet, tracked, {})))
    box = BoxState(a.box_id, a.sku, Size3D(*size), a.mass, Pose3D("conveyor", 0.0, 0.0, 0.0),
                   (0.0, QUARTER), BoxStatus.READY_FOR_PICK, 1.0, st["stamp_sec"], "CCTV_PICK_TOPVIEW+SCALE")
    if any(b["box_id"] == a.box_id for b in st["placed"]):
        raise SystemExit(f"COMMIT FAIL: {a.box_id} is already on the pallet")
    sm.commit_observation(box, st["stamp_sec"], from_stock=False)
    measured_pose = Pose3D(**measured)
    if size == true_size:        # outside the tolerance (or no plan): resolve sensor overlaps
        measured_pose, _ = sm.reconcile(box.size, measured_pose)
    sm.place(a.box_id, measured_pose, planned_pose=planned if size != true_size else None)
    snap = sm.snapshot()
    on_plan = size != true_size
    st["placed"] = [json.loads(json.dumps(plain(p))) for p in snap.pallet.boxes]
    st["state_version"] += 1
    st["stamp_sec"] = float(st["state_version"])
    path.write_text(json.dumps(st, indent=2))
    pose = snap.pallet.boxes[-1].pose
    basis = "planned corner (within commit tolerance)" if on_plan else "measured pose"
    print(f"COMMIT OK: {a.box_id} at {basis}, corner {(round(pose.x, 3), round(pose.y, 3), round(pose.z, 3))} "
          f"yaw {math.degrees(pose.yaw):.0f} deg; {len(st['placed'])} boxes, state_version {st['state_version']}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--state", required=True)
    p.add_argument("--box-id", required=True)
    p.add_argument("--mass", type=float, required=True)
    p.add_argument("--perception", required=True, help="JSON from /pac/perception/box_info")
    p.add_argument("--remaining", type=int, default=0, help="boxes still to come (same SKU)")
    p.add_argument("--remaining-json", help='boxes still to come by SKU, e.g. {"K01": 2} (order unknown)')
    p.add_argument("--sku", help="SKU id of this box (arrival identity); default: match by footprint")
    p.add_argument("--catalog", help="AHEAD dataset generator config YAML (sku_catalog)")
    p.add_argument("--model", default=None, help="learned ranker JSON (default: heuristic Top-K)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--pick-xy", type=float, nargs=2, default=None,
                   help="where the robot picks the box (default: perception x, y; buffer bay for buffered boxes)")
    p.add_argument("--out", required=True)
    c = sub.add_parser("commit")
    c.add_argument("--state", required=True)
    c.add_argument("--box-id", required=True)
    c.add_argument("--sku", default="GZ_TEST_40x30x25")
    c.add_argument("--catalog", help="AHEAD dataset generator config YAML (sku_catalog)")
    c.add_argument("--planned", help="plan JSON of this box (from `plan --out`)")
    c.add_argument("--mass", type=float, required=True)
    c.add_argument("--world", type=float, nargs=4, required=True, metavar=("X", "Y", "Z", "YAW"))
    a = ap.parse_args()
    return cmd_plan(a) if a.cmd == "plan" else cmd_commit(a)


if __name__ == "__main__":
    sys.exit(main())
