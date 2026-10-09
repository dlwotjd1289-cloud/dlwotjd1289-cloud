"""V4.5 bridge between the Gazebo workcell (V4.3 weighing, V4.4 robot) and the
team planning pipeline (Taehyeon candidates + Donghan ranking).

Plain-JSON helpers (plain(SystemState) / plain(PlanningContext) / plain(PlanningResult))
so the ROS robot node and the planner process exchange files without sharing
Python objects.

Planner convention (common standard v0.3, section 14): target_pose is in frame
"pallet", xyz is the lower x/y/z corner of the box AABB after yaw, z=0 is the
pallet deck top. The robot uses box centres in the Gazebo world frame.
"""
from __future__ import annotations

import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
TEAM_PACKAGES = ("pac_common", "pac_candidates", "pac_planning", "pac_robot_check")


def ensure_team_packages() -> None:
    """Make the monorepo packages importable when not run from colcon/pytest."""
    for name in reversed(TEAM_PACKAGES):
        path = str(ROOT / "ros2_ws" / "src" / name)
        if path not in sys.path:
            sys.path.insert(0, path)

SCHEMA = "pac-common-v0.2+planning-v1"


@dataclass(frozen=True)
class GazeboPallet:
    """Pallet deck in a world frame: top centre, yaw, footprint, cargo height.

    Build it with gazebo_pallet(): footprint / cargo height come from the team
    config (config/default.yaml), the position and the frame orientation
    (``frame_yaw_rad``: origin at the deck corner farthest from the robot) from
    config/workcell.yaml (pallet_main in the V4.2 world, deck top z = 0.15 m).
    """

    top_center_world: tuple
    size_xy: tuple
    max_stack_height_m: float
    yaw: float = 0.0
    pallet_id: str = "P001"


def gazebo_pallet(top_center_world=None) -> GazeboPallet:
    """Workcell pallet from the shared config; top_center_world overrides the position
    (e.g. (0, 0, 0) for the standalone Bullet simulator whose deck top is the origin)."""
    from pac_common.config import load_common_config

    spec = load_common_config().pallet
    layout = workcell_layout()
    if top_center_world is None:
        cx, cy, cz = layout["pallet"]["center_world_m"]
        top_center_world = (cx, cy, cz + spec.deck_height_m / 2)
    return GazeboPallet(tuple(top_center_world), (spec.size_x_m, spec.size_y_m),
                        spec.max_stack_height_m, yaw=float(layout["pallet"].get("frame_yaw_rad", 0.0)),
                        pallet_id=spec.pallet_id)


def workcell_layout() -> dict:
    """config/workcell.yaml ``layout`` (next to the team config/default.yaml)."""
    import yaml
    from pac_common.config import load_common_config

    path = load_common_config().path.parent / "workcell.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["layout"]


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
                           pallet: Optional[GazeboPallet] = None, placed_boxes: Sequence[dict] = (),
                           remaining_by_sku: Optional[dict] = None, nominal_kg: Optional[float] = None,
                           top_load_capacity_n: float, pallet_max_weight_kg: float,
                           allowed_yaws_rad: Sequence[float] = (0.0, math.pi / 2)) -> tuple[dict, dict]:
    """plain(SystemState), plain(PlanningContext) for one weighed box waiting at PICK.

    weight_kg is the V4.3 scale reading; the catalog keeps the nominal SKU mass.
    top_load_capacity_n / pallet_max_weight_kg come from the team config
    (plan_placement_v45.team_limits), never hard-coded here.
    """
    pallet = pallet or gazebo_pallet()
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
    """AABB footprint after yaw; only multiples of 90 deg (pac_common.frames)."""
    ensure_team_packages()
    from pac_common import Size3D
    from pac_common.frames import rotated_dims as common

    return common(Size3D(*map(float, size_m)), yaw)


def candidate_to_world(target_pose: dict, size_m: Sequence[float],
                       pallet: Optional[GazeboPallet] = None) -> tuple[float, float, float, float]:
    """Planner lower-corner pose (frame 'pallet') -> box centre (x, y, z, yaw) in Gazebo world."""
    pallet = pallet or gazebo_pallet()
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
                           pallet: Optional[GazeboPallet] = None) -> dict:
    """Measured box centre -> planner lower-corner pose; inverse of candidate_to_world.

    yaw is snapped to the nearest 90 deg so the corner matches the AABB the
    planner reasons about; tilt is checked separately by the robot node.
    """
    pallet = pallet or gazebo_pallet()
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


def team_configs():
    """(candidate config, planner config, robot-check config, pallet max load kg, top-load fn)
    from the team files: config/taehyeon/candidates.yaml, config/default.yaml,
    config/taehyeon/robot_check.yaml. Top load = Taehyeon's McKee model."""
    ensure_team_packages()
    from pac_candidates import load_candidate_config
    from pac_candidates.loads import mckee_capacity_n
    from pac_common.config import load_common_config
    from pac_planning.config import load_config
    from pac_robot_check import load_robot_check_config

    cand = load_candidate_config(str(ROOT / "config/taehyeon/candidates.yaml"))
    lm = cand.constraints.load_model

    def top_load_n(size_m):
        return round(mckee_capacity_n(size_m[0], size_m[1], lm.ect_n_per_m, lm.board_thickness_m,
                                      lm.safety_factor), 3)

    return (cand, load_config(str(ROOT / "config/default.yaml")),
            load_robot_check_config(ROOT / "config/taehyeon/robot_check.yaml"),
            load_common_config().pallet.max_load_kg, top_load_n)


def plan_ranked(state: dict, context: dict, box_id: str, *, candidate_config, planner_config, robot_config,
                seed: int = 42, model=None, use_time_budget: bool = False) -> dict:
    """Stages 5 and 6 on one snapshot, with the team implementations only.

    5: ``pac_planning.planning_service.plan_request`` (Taehyeon candidates / hard
       mask + Donghan ranking; the function behind /pac/plan_placement).
    6: ``pac_robot_check.RobotFeasibility`` on every ranked candidate (team cell
       of config/taehyeon/robot_check.yaml).

    Returns the plan_request result with ``ranked[i]["robot"]`` = {success, codes,
    details} and ``executable`` = ranked candidates that passed stage 6, best first.
    """
    ensure_team_packages()
    from pac_common.adapters import candidate_from_json, state_from_json
    from pac_planning.planning_service import plan_request
    from pac_robot_check import RobotFeasibility

    result = json.loads(plan_request(json.dumps(state), json.dumps(context), box_id, state["state_version"],
                                     candidate_config, planner_config, seed=seed, model=model,
                                     use_time_budget=use_time_budget))
    snapshot = state_from_json(state)
    box = snapshot.inventory.tracked_boxes[box_id]
    robot = RobotFeasibility(robot_config)
    executable = []
    for cand in result["ranked"]:
        verdict = robot.validate_robot_motion(box, candidate_from_json(cand), snapshot)
        cand["robot"] = {"success": verdict.success, "codes": [c.value for c in verdict.codes],
                         "details": json.loads(json.dumps(dict(verdict.details), default=float))}
        if verdict.success:
            executable.append(cand)
    result["executable"] = executable
    return result


def execution_result(*, success: bool, box_id: str, candidate_id: str, actual_pose: Optional[dict],
                     codes: Sequence[str], stamp_sec: float, detail: Optional[dict] = None) -> dict:
    """plain(ExecutionResult) for the State Manager. This module never commits ACTUAL state."""
    out = {"success": bool(success), "box_id": box_id, "candidate_id": candidate_id,
           "actual_pose": actual_pose, "codes": list(codes), "stamp_sec": float(stamp_sec)}
    if detail:
        out["detail"] = detail
    return out
