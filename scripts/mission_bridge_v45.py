"""V4.5 bridge between the Gazebo workcell (V4.3 weighing, V4.4 robot) and the
team planning pipeline in pac-mission1-shared (Taehyeon candidates + Donghan ranking).

JSON-only on purpose: the shared repo and this scaffold both ship a package named
`pac_common` with different contents, so this module never imports either one.
The planner runs in its own process (plan_placement_v45.py) and exchanges
plain(SystemState) / plain(PlanningContext) / plain(PlanResult) JSON.

Planner convention (shared docs/integration.md): target_pose is in frame
"pallet", xyz is the lower x/y/z corner of the box AABB after yaw, z=0 is the
pallet deck top. The robot uses box centres in the Gazebo world frame.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Optional, Sequence

SCHEMA = "pac-common-v0.2+planning-v1"


@dataclass(frozen=True)
class GazeboPallet:
    """pallet_main in ahead_workcell_v4_2_physical_scale.sdf (deck top, not wood centre).

    Team footprint 1100 x 1100 mm (generator / Bullet simulator); the Gazebo top
    boards span 1.100 x 1.098 m, deck top z = 0.15 m. Stack height 1.35 m above the
    deck = generator max_height_m 1.5 incl. 0.15 m deck (Taehyeon, 2026-10-07).
    Note config/workcell.yaml still holds the older (1.35, -1.00) / 1.2 x 1.0 layout.
    """

    top_center_world: tuple = (0.0, 1.20, 0.15)
    yaw: float = 0.0
    size_xy: tuple = (1.10, 1.10)
    max_stack_height_m: float = 1.35
    pallet_id: str = "PALLET_MAIN"


def parse_scale_pass(text: str) -> float:
    """Measured mass from run_auto_scale_v43 output ('V4.3 PASS: 5.012 kg; ...')."""
    m = re.search(r"V4\.3 PASS:\s*([0-9.]+)\s*kg", text)
    if m is None:
        raise ValueError("V4.3 did not report PASS; no measured mass")
    return float(m.group(1))


def _pose(frame, x, y, z, yaw=0.0):
    return {"frame_id": frame, "x": x, "y": y, "z": z, "roll": 0.0, "pitch": 0.0, "yaw": yaw}


def build_planning_request(*, box_id: str, sku_id: str, size_m: Sequence[float], measured_kg: float,
                           pick_center_world: Sequence[float], state_version: int, stamp_sec: float,
                           pallet: GazeboPallet = GazeboPallet(), placed_boxes: Sequence[dict] = (),
                           remaining_by_sku: Optional[dict] = None, nominal_kg: Optional[float] = None,
                           top_load_capacity_n: float, pallet_max_weight_kg: float,
                           allowed_yaws_rad: Sequence[float] = (0.0, math.pi / 2)) -> tuple[dict, dict]:
    """plain(SystemState), plain(PlanningContext) for one weighed box waiting at PICK.

    weight_kg is the V4.3 scale reading; the catalog keeps the nominal SKU mass.
    top_load_capacity_n / pallet_max_weight_kg come from the team config
    (plan_placement_v45.team_limits), never hard-coded here.
    """
    if measured_kg <= 0:
        raise ValueError("measured mass must be positive")
    size = {"x": float(size_m[0]), "y": float(size_m[1]), "z": float(size_m[2])}
    yaws = [float(a) for a in allowed_yaws_rad]
    box = {
        "box_id": box_id, "sku_id": sku_id, "size": size, "weight_kg": float(measured_kg),
        "pose": _pose("world", *map(float, pick_center_world)),
        "allowed_yaws_rad": yaws, "status": "READY_FOR_PICK", "confidence": 1.0,
        "stamp_sec": float(stamp_sec), "source": "GAZEBO_SCALE_V43_GROUND_TRUTH_POSE",
    }
    state = {
        "state_version": int(state_version), "stamp_sec": float(stamp_sec),
        "pallet": {
            "pallet_id": pallet.pallet_id,
            "size": {"x": pallet.size_xy[0], "y": pallet.size_xy[1], "z": pallet.max_stack_height_m},
            "boxes": list(placed_boxes),
        },
        "inventory": {"tracked_boxes": {box_id: box}, "remaining_by_sku": dict(remaining_by_sku or {})},
    }
    context = {
        "catalog": {sku_id: {
            "sku_id": sku_id, "size": size,
            "weight_kg": float(nominal_kg if nominal_kg is not None else measured_kg),
            "allowed_yaws_rad": yaws, "top_load_capacity_n": float(top_load_capacity_n),
        }},
        "pallet_max_weight_kg": float(pallet_max_weight_kg),
        "buffer_capacity": 0,
    }
    return state, context


def rotated_dims(size_m: Sequence[float], yaw: float) -> tuple[float, float, float]:
    """AABB footprint after yaw; only multiples of 90 deg (planner's allowed yaws)."""
    quarter = round(yaw / (math.pi / 2))
    if abs(yaw - quarter * math.pi / 2) > 1e-6:
        raise ValueError(f"Only 90-degree yaws are supported, got {yaw}")
    sx, sy, sz = size_m
    return (sy, sx, sz) if quarter % 2 else (sx, sy, sz)


def candidate_to_world(target_pose: dict, size_m: Sequence[float],
                       pallet: GazeboPallet = GazeboPallet()) -> tuple[float, float, float, float]:
    """Planner lower-corner pose (frame 'pallet') -> box centre (x, y, z, yaw) in Gazebo world."""
    if target_pose["frame_id"] != "pallet":
        raise ValueError("Expected a pallet-frame candidate")
    if abs(target_pose.get("roll", 0.0)) > 1e-8 or abs(target_pose.get("pitch", 0.0)) > 1e-8:
        raise ValueError("Only upright boxes are supported")
    yaw = float(target_pose["yaw"])
    dx, dy, dz = rotated_dims(size_m, yaw)
    lx = target_pose["x"] + dx / 2 - pallet.size_xy[0] / 2
    ly = target_pose["y"] + dy / 2 - pallet.size_xy[1] / 2
    c, s = math.cos(pallet.yaw), math.sin(pallet.yaw)
    ox, oy, oz = pallet.top_center_world
    return (ox + c * lx - s * ly, oy + s * lx + c * ly, oz + target_pose["z"] + dz / 2, pallet.yaw + yaw)


def world_to_pallet_corner(center_world: Sequence[float], yaw_world: float, size_m: Sequence[float],
                           pallet: GazeboPallet = GazeboPallet()) -> dict:
    """Measured box centre -> planner lower-corner pose; inverse of candidate_to_world.

    yaw is snapped to the nearest 90 deg so the corner matches the AABB the
    planner reasons about; tilt is checked separately by the robot node.
    """
    yaw = wrap_pi(yaw_world - pallet.yaw)
    yaw = round(yaw / (math.pi / 2)) * (math.pi / 2)
    yaw = 0.0 if abs(yaw) < 1e-9 else yaw
    dx, dy, dz = rotated_dims(size_m, yaw)
    ox, oy, oz = pallet.top_center_world
    wx, wy = center_world[0] - ox, center_world[1] - oy
    c, s = math.cos(pallet.yaw), math.sin(pallet.yaw)
    lx, ly = c * wx + s * wy, -s * wx + c * wy
    return _pose("pallet", lx + pallet.size_xy[0] / 2 - dx / 2, ly + pallet.size_xy[1] / 2 - dy / 2,
                 center_world[2] - oz - dz / 2, yaw)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q: Sequence[float]) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def gripper_yaw_delta(target_yaw: float, box_yaw_at_pick: float) -> float:
    """Wrist rotation that turns the picked box to target_yaw.

    A box footprint is symmetric under 180 deg, so the result is folded to
    [-90, 90] deg to keep the wrist turn short.
    """
    d = wrap_pi(target_yaw - box_yaw_at_pick)
    if d > math.pi / 2:
        d -= math.pi
    elif d < -math.pi / 2:
        d += math.pi
    return d


def execution_result(*, success: bool, box_id: str, candidate_id: str, actual_pose: Optional[dict],
                     codes: Sequence[str], stamp_sec: float, detail: Optional[dict] = None) -> dict:
    """plain(ExecutionResult) for the State Manager. This module never commits ACTUAL state."""
    out = {"success": bool(success), "box_id": box_id, "candidate_id": candidate_id,
           "actual_pose": actual_pose, "codes": list(codes), "stamp_sec": float(stamp_sec)}
    if detail:
        out["detail"] = detail
    return out
