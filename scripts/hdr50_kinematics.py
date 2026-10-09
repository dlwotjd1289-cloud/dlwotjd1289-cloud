"""HDR50-22 kinematics for the Gazebo workcell scripts (V4.4 / V4.5).

Thin adapter over the team's single HDR50-22 model, ``pac_robot_check.kinematics``
(joint table from the official hdr_description URDF, closed-form IK). This module
only changes frames: the robot base_link stands on the 0.40 m pedestal
(pac_bringup/urdf/hdr50_22_pedestal.urdf.xacro world_joint), and the target frame
is ``flange_link`` whose +x axis is the tool direction (flange face at the origin).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

try:
    from pac_robot_check import kinematics as _kin
except ImportError:  # running from a plain checkout without colcon / pip install
    for _pkg in ("pac_robot_check", "pac_common"):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src" / _pkg))
    from pac_robot_check import kinematics as _kin

BASE_Z_M = 0.40
_BASE = np.array([0.0, 0.0, BASE_Z_M])
_FLANGE_TO_TOOL0 = _kin._TOOL0[:3, :3]   # tool0 = flange rotated by pitch +pi/2 (URDF)
LOWER = _kin.LOWER
UPPER = _kin.UPPER
VEL_LIMIT = _kin.VELOCITY
HOME = np.array([0.0, 1.5707, 0.0, 0.0, 0.0, 0.0])  # hdr50_22 initial_positions.yaml


def fk(q: Sequence[float], return_frames: bool = False):
    """World pose of flange_link as (p[3], R[3x3]); with ``return_frames`` also
    [(joint origin, joint axis)] in world, for Jacobians."""
    frames = _kin.joint_frames(np.asarray(q, dtype=float))
    flange = frames[5]
    p = flange[:3, 3] + _BASE
    R = flange[:3, :3]
    if not return_frames:
        return p, R
    joints = [(T[:3, 3] + _BASE, T[:3, :3] @ np.asarray(j.axis, dtype=float))
              for T, j in zip(frames[:6], _kin.HDR50_22)]
    return p, R, joints


def tool_down_rotation(yaw: float = 0.0) -> np.ndarray:
    """Flange +x pointing down; flange +y along world yaw (box stays level)."""
    x = np.array([0.0, 0.0, -1.0])
    y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return np.column_stack([x, y, np.cross(x, y)])


def _rot_angle(R: np.ndarray, R_target: np.ndarray) -> float:
    c = max(-1.0, min(1.0, (np.trace(R_target @ R.T) - 1.0) / 2.0))
    return math.acos(c)


def ik(p_target: Sequence[float], R_target: np.ndarray, seed: Sequence[float],
       tol_m: float = 1e-4, tol_rad: float = 1e-3, iters: int = 300) -> Optional[np.ndarray]:
    """IK branch closest to ``seed`` (within joint limits) or None. ``iters`` is
    kept for call compatibility; the solver is closed form."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R_target, dtype=float) @ _FLANGE_TO_TOOL0
    T[:3, 3] = np.asarray(p_target, dtype=float) - _BASE
    q = _kin.ik(T, 0.0, seeds=[np.asarray(seed, dtype=float)])
    if q is None:
        return None
    p, R = fk(q)
    if np.linalg.norm(p - np.asarray(p_target, dtype=float)) > max(tol_m, 1e-4) or \
            _rot_angle(R, R_target) > max(tol_rad, 1e-3):
        return None
    return q


def ik_near(p_target, R_target, seeds) -> Optional[np.ndarray]:
    """Try seeds in order; return the first solution (closest to first seed preferred)."""
    best = None
    for s in seeds:
        sol = ik(p_target, R_target, s)
        if sol is not None:
            if best is None or np.abs(sol - seeds[0]).max() < np.abs(best - seeds[0]).max():
                best = sol
            if np.abs(sol - seeds[0]).max() < 1.0:
                return sol
    return best


def cartesian_path(q_start, p_end, R_target, step_m: float = 0.02):
    """Straight-line flange path from FK(q_start) to p_end; joint waypoints or None."""
    p0, _ = fk(q_start)
    p_end = np.array(p_end, dtype=float)
    n = max(1, int(math.ceil(np.linalg.norm(p_end - p0) / step_m)))
    q = np.array(q_start, dtype=float)
    out = []
    for k in range(1, n + 1):
        sol = ik(p0 + (p_end - p0) * k / n, R_target, q)
        if sol is None or np.abs(sol - q).max() > 0.35:  # reject branch flips
            return None
        out.append(sol)
        q = sol
    return out
