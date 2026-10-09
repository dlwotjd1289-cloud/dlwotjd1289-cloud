#!/usr/bin/env python3
"""Top-view perception + scale -> AHEAD low-level placement planner -> robot slot (V4.4).

  plan:   python3 scripts/ahead_planner_bridge_v44.py plan --state S.json --box-id box_03 \
              --mass 5.0 --perception P.json --remaining 5 --out PLAN.json
          prints "x y z yaw" (world frame box centre for moveit_pick_place_v44.py --slot/--slot-yaw)
  commit: python3 scripts/ahead_planner_bridge_v44.py commit --state S.json --box-id box_03 \
              --mass 5.0 --world X Y Z YAW
          records the ACTUALLY placed box (Gazebo pose) as the next ACTUAL pallet state.

Planner: Donghan PlacementPlanner from this repository (ros2_ws/src/pac_planning) with its
ReferenceBackend generator / validator (switching to pac_candidates is still open). Contract (docs/integration.md):
pallet frame origin = lower corner of the usable deck, z = 0 on the deck, target_pose = rotated
box AABB minimum corner, upright boxes, yaw in allowed_yaws_rad (0 / 90 deg). This bridge is the
single writer of the ACTUAL state file (state_version increments on each commit).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]  # the integrated monorepo's own pac_common / pac_planning
for sub in ("ros2_ws/src/pac_common", "ros2_ws/src/pac_planning"):
    sys.path.insert(0, str(REPO / sub))

from pac_common.adapters import context_from_json, state_from_json  # noqa: E402
from pac_planning import PlacementPlanner, load_config  # noqa: E402
from pac_planning.geometry import center_pose  # noqa: E402
from pac_planning.reference_backend import ReferenceBackend  # noqa: E402

# Gazebo workcell <-> planner pallet frame. pallet_main: centre (0, 1.20), 1.10 x 1.10 m, deck z 0.15.
# The planner fills from its origin corner, so the origin is put at the corner FARTHEST from the
# robot (world (0.55, 1.75)) with both axes reversed (a 180 deg rotation about z, not a mirror):
# far slots are filled first and the arm never reaches over placed boxes to a farther slot.
PALLET_SIZE = (1.10, 1.10)
PALLET_ORIGIN_WORLD = (0.0 + PALLET_SIZE[0] / 2, 1.20 + PALLET_SIZE[1] / 2, 0.15)


def pallet_to_world(px, py, pz):
    return PALLET_ORIGIN_WORLD[0] - px, PALLET_ORIGIN_WORLD[1] - py, PALLET_ORIGIN_WORLD[2] + pz


def world_to_pallet(x, y, z):
    return PALLET_ORIGIN_WORLD[0] - x, PALLET_ORIGIN_WORLD[1] - y, z - PALLET_ORIGIN_WORLD[2]
import os
# Stack height above the deck: nominal 1.5 m, up to 1.6 m allowed when nothing fits below 1.5 m
# (user decision 2026-10-09; was a 1.0 m commissioning limit). Env overrides for tests.
NOMINAL_STACK_HEIGHT_M = float(os.environ.get("PAC_STACK_NOMINAL_M", "1.5"))
MAX_STACK_HEIGHT_M = float(os.environ.get("PAC_STACK_MAX_M", "1.6"))
# Robot reach (HDR50-22 on the pedestal, IK grid 2026-10-09): TCP z <= 1.95 m over the whole pallet
# (far row fails from 2.05 m). A slot is only used if the box can be carried to it: transfer TCP
# = max(pick approach, top of the boxes under the PICK->slot corridor + clearance + box height).
REACH_TCP_MAX_Z = 1.95
PICK_TOP_Z = 0.895                  # conveyor top (box bottom at PICK)
APPROACH_M, STACK_CLEAR_M = 0.20, 0.10
PALLET_MAX_WEIGHT_KG = 250.0
QUARTER = math.pi / 2

# Robot placement clearance: the planner packs boxes flush (0 gap); a suction robot lowering a box
# between neighbours needs a small gap, so the planner sees each box this much larger in x/y.
# Placed boxes are committed at their true size, so the real gap to a neighbour is clearance/2:
# 10 mm (5 mm gap) was too tight for a 13.8 kg box lowered at speed 0.2 -> 20 mm (10 mm gap).
PLACE_CLEARANCE_M = 0.020
PLANNED_TOL_M = 0.005             # placed within this of the plan -> committed at the planned corner
# SKU master data. Default: the Gazebo 5 kg test box. --catalog loads the AHEAD dataset generator
# sku_catalog instead. Top-load capacity is NOT in the generator and NOT measured: placeholder
# 4 kPa x footprint area (K01 ~ 167 N ... K13 ~ 1000 N) until carton strength data exist.
CATALOG = {
    "GZ_TEST_40x30x25": {"sku_id": "GZ_TEST_40x30x25", "size": {"x": 0.40, "y": 0.30, "z": 0.25},
                          "weight_kg": 5.0, "allowed_yaws_rad": [0.0, QUARTER], "top_load_capacity_n": 350.0},
}
PLACEHOLDER_TOP_LOAD_PA = 4000.0


def load_generator_catalog(path: str) -> None:
    import yaml
    cfg = yaml.safe_load(open(path))
    CATALOG.clear()
    for s in cfg["sku_catalog"]:
        size = {k: float(s["size_m"][k]) for k in ("x", "y", "z")}
        CATALOG[s["sku_id"]] = {
            "sku_id": s["sku_id"], "size": size,
            "weight_kg": round((float(s["weight_min_kg"]) + float(s["weight_max_kg"])) / 2, 3),
            "allowed_yaws_rad": [float(v) for v in s.get("allowed_yaws_rad", [0.0, QUARTER])],
            "top_load_capacity_n": round(PLACEHOLDER_TOP_LOAD_PA * size["x"] * size["y"], 1)}
FOOTPRINT_TOL_M = 0.03


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


def planner_catalog() -> dict:
    out = {}
    for sku, spec in CATALOG.items():
        s = dict(spec)
        s["size"] = {"x": spec["size"]["x"] + PLACE_CLEARANCE_M, "y": spec["size"]["y"] + PLACE_CLEARANCE_M,
                     "z": spec["size"]["z"]}
        out[sku] = s
    return out


def scenario(st: dict, box_id: str, sku: str, mass: float, yaw_on_conveyor: float, remaining,
             max_height: float = MAX_STACK_HEIGHT_M) -> dict:
    spec = planner_catalog()[sku]
    pose0 = {"frame_id": "conveyor", "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    current = {"box_id": box_id, "sku_id": sku, "size": spec["size"], "weight_kg": mass, "pose": pose0,
               "allowed_yaws_rad": spec["allowed_yaws_rad"], "status": "READY_FOR_PICK",
               "confidence": 1.0, "stamp_sec": st["stamp_sec"], "source": "CCTV_PICK_TOPVIEW+SCALE"}
    return {
        "state": {"state_version": st["state_version"], "stamp_sec": st["stamp_sec"],
                  "pallet": {"pallet_id": "GZ_PALLET_MAIN",
                             "size": {"x": PALLET_SIZE[0], "y": PALLET_SIZE[1], "z": max_height},
                             "boxes": st["placed"]},
                  "inventory": {"tracked_boxes": {box_id: current},
                                "remaining_by_sku": (remaining if isinstance(remaining, dict)
                                                     else {sku: max(0, remaining)})}},
        "context": {"catalog": planner_catalog(), "pallet_max_weight_kg": PALLET_MAX_WEIGHT_KG, "buffer_capacity": 0},
    }


def placed_world_aabbs(st: dict):
    """World-frame (xmin, xmax, ymin, ymax, ztop) of the committed boxes."""
    out = []
    for b in st["placed"]:
        p, sz = b["pose"], b["size"]
        dx, dy = (sz["y"], sz["x"]) if round(p["yaw"] / QUARTER) % 2 else (sz["x"], sz["y"])
        x0, y0, z0 = pallet_to_world(p["x"], p["y"], p["z"])     # min corner in pallet = max in world
        out.append((x0 - dx, x0, y0 - dy, y0, z0 + sz["z"]))
    return out


def corridor_top(aabbs, start_xy, end_xy, half_width):
    """Highest box top whose footprint comes within half_width of the segment start->end."""
    import numpy as np
    a, b = np.array(start_xy, float), np.array(end_xy, float)
    top = PALLET_ORIGIN_WORLD[2]
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


def transfer_tcp_z(st, pick_xy, slot_xy, size):
    h = size["z"]
    half = math.hypot(size["x"], size["y"]) / 2 + 0.05
    top = corridor_top(placed_world_aabbs(st), pick_xy, slot_xy, half)
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
    remaining = json.loads(a.remaining_json) if a.remaining_json else a.remaining
    config = load_config(str(PLANNER_ROOT / "config" / "default.yaml"))
    pick_xy = (per.get("x", -1.03), per.get("y", 1.20)) if a.pick_xy is None else tuple(a.pick_xy)
    true_size = CATALOG[sku]["size"]
    chosen, notes = None, []
    # Nominal height first; the 1.6 m allowance only when nothing fits below 1.5 m.
    for height, mode in ((NOMINAL_STACK_HEIGHT_M, "nominal"), (MAX_STACK_HEIGHT_M, "allowance")):
        data = scenario(st, a.box_id, sku, a.mass, per["yaw"], remaining, height)
        state = state_from_json(data["state"])
        context = context_from_json(data["context"])
        box = state.inventory.tracked_boxes[a.box_id]
        backend = ReferenceBackend(context, config)
        planner = PlacementPlanner(context=context, config=config, model_path=a.model,
                                   generate_candidates=backend.generate_candidates,
                                   validate_constraints=backend.validate_constraints)
        result = planner.plan(box, state, seed=a.seed, mode="ahead", use_time_budget=True)
        unreachable = 0
        for cand in result.ranked:                  # best first; skip slots the arm cannot carry to
            c = center_pose(box, cand)
            w = pallet_to_world(c.x, c.y, c.z)
            tz = transfer_tcp_z(st, pick_xy, w[:2], true_size)
            if tz <= REACH_TCP_MAX_Z:
                chosen = (cand, c, w, tz, mode)
                break
            unreachable += 1
        if unreachable:
            notes.append(f"{mode} {height:.2f} m: {unreachable} candidate(s) out of reach")
        if chosen:
            break
        if not result.ranked:
            from collections import Counter
            codes = Counter(c.value for v in result.rejected.values() for c in v.codes)
            notes.append(f"{mode} {height:.2f} m: no valid candidate ({len(result.rejected)} rejected: {dict(codes)})")
        if MAX_STACK_HEIGHT_M <= NOMINAL_STACK_HEIGHT_M:
            break
    if chosen is None:
        raise SystemExit("PLAN FAIL: NO_SLOT (" + "; ".join(notes) + ")")
    cand, c, world, tz, mode = chosen
    out = {"box_id": a.box_id, "sku_id": sku, "mass_kg": a.mass, "state_version": st["state_version"],
           "candidate_id": cand.candidate_id, "score": cand.score,
           "target_corner_pallet": [cand.target_pose.x, cand.target_pose.y, cand.target_pose.z],
           "yaw_rad": cand.target_pose.yaw, "center_pallet": [c.x, c.y, c.z],
           "slot_world": [round(v, 4) for v in world], "n_ranked": len(result.ranked),
           "n_rejected": len(result.rejected), "height_mode": mode,
           "stack_limit_m": NOMINAL_STACK_HEIGHT_M if mode == "nominal" else MAX_STACK_HEIGHT_M,
           "transfer_tcp_z": round(tz, 3), "notes": notes,
           "planning_time_sec": result.diagnostics.get("planning_time_sec"),
           "perception": per, "planner": "donghan PlacementPlanner @7860043 + ReferenceBackend",
           "model": a.model or "heuristic (no learned model)"}
    Path(a.out).write_text(json.dumps(out, indent=2, default=float))
    print(f"{world[0]:.4f} {world[1]:.4f} {world[2]:.4f} {cand.target_pose.yaw:.4f}")
    return 0


def cmd_commit(a) -> int:
    if a.catalog:
        load_generator_catalog(a.catalog)
    path = Path(a.state)
    st = load_state(path)
    x, y, z, yaw = a.world
    q = round(yaw / QUARTER)
    if abs(yaw - q * QUARTER) > math.radians(5):
        raise SystemExit(f"COMMIT FAIL: placed yaw {math.degrees(yaw):.1f} deg is not axis-aligned")
    # The planner reasons with clearance-inflated boxes, so the committed state uses the same size:
    # - placed within PLANNED_TOL_M of the plan -> committed AT the planned corner (inflated size).
    #   Keeps the planner model self-consistent: true sizes broke same-size stacking (reference
    #   validator needs one supporter that fully contains the box), and inflated boxes at the
    #   measured pose overlapped by the ~1-2 mm placement error ("Existing boxes overlap").
    # - otherwise the measured pose with the true size (the deviation is not hidden).
    spec = planner_catalog()[a.sku]
    true_size = CATALOG[a.sku]["size"]
    yaw_snap = (q % 2) * QUARTER        # planner geometry: upright, 0 or 90 deg
    cx, cy, cz = world_to_pallet(x, y, z)
    plan = json.loads(Path(a.planned).read_text()) if a.planned else None
    dev = None
    if plan is not None:
        pcx, pcy, pcz = plan["center_pallet"]
        dev = math.hypot(cx - pcx, cy - pcy)
    if plan is not None and dev <= PLANNED_TOL_M and abs(plan["yaw_rad"] - yaw_snap) < 1e-6:
        sx, sy, sz = spec["size"]["x"], spec["size"]["y"], spec["size"]["z"]
        corner = tuple(plan["target_corner_pallet"])
        size_out, basis = spec["size"], f"planned corner (measured deviation {dev * 1000:.1f} mm)"
    else:
        sx, sy, sz = true_size["x"], true_size["y"], true_size["z"]
        dx, dy = (sy, sx) if q % 2 else (sx, sy)
        corner = (cx - dx / 2, cy - dy / 2, max(0.0, cz - sz / 2))
        size_out = true_size
        basis = "measured pose" + (f" (deviation {dev * 1000:.1f} mm > tolerance)" if dev is not None else "")
    st["placed"] = [b for b in st["placed"] if b["box_id"] != a.box_id] + [{
        "box_id": a.box_id, "sku_id": a.sku, "size": size_out, "weight_kg": a.mass,
        "pose": {"frame_id": "pallet", "x": round(corner[0], 4), "y": round(corner[1], 4),
                 "z": round(corner[2], 4), "roll": 0.0, "pitch": 0.0, "yaw": yaw_snap}}]
    st["state_version"] += 1
    st["stamp_sec"] = float(st["state_version"])
    path.write_text(json.dumps(st, indent=2))
    print(f"COMMIT OK: {a.box_id} at {basis}, corner {tuple(round(v, 3) for v in corner)} yaw {math.degrees(yaw_snap):.0f} deg; "
          f"{len(st['placed'])} boxes, state_version {st['state_version']}")
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
