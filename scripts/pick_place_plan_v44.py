"""ROS-independent pick & place path plan for one box (V4.4).

Box poses are box *centers* in the world frame; the flange (tool down) grips
the box top with a small gap, like a vacuum pad. Collision checking is not
performed: the path stays above the conveyor/pick stopper (transfer height)
and descends vertically, which is a commissioning assumption, not a proof.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

import hdr50_kinematics as K


@dataclass(frozen=True)
class PlanConfig:
    grip_gap_m: float = 0.005          # flange face above box top when attaching
    place_drop_m: float = 0.003        # box bottom above pallet top when releasing
    approach_height_m: float = 0.30    # vertical approach / retreat length
    transfer_z_m: float = 1.60         # flange z during transfer (box clears stopper)
    cart_speed_m_s: float = 0.20
    joint_speed_ratio: float = 0.25    # fraction of URDF velocity limits
    min_segment_s: float = 1.0
    tool_yaw_rad: float = 0.0


@dataclass
class Segment:
    name: str
    status: str                        # progress text shown to the operator
    points: List[np.ndarray]
    duration_s: float
    gripper_after: Optional[str] = None  # "attach" / "detach"


def _seed_for(xy: Sequence[float]) -> np.ndarray:
    # Elbow-up, non-flipped wrist reference posture facing the target.
    return np.array([math.atan2(xy[1], xy[0]), 1.2, 0.0, 0.0, -1.0, 0.0])


def _duration(points: List[np.ndarray], q_from: np.ndarray, cart_len_m: float, cfg: PlanConfig) -> float:
    dq = np.abs(points[-1] - q_from)
    path = np.abs(np.diff(np.vstack([q_from] + points), axis=0)).sum(axis=0)
    t_joint = float(np.max(np.maximum(dq, path) / (K.VEL_LIMIT * cfg.joint_speed_ratio)))
    return max(cfg.min_segment_s, t_joint, cart_len_m / cfg.cart_speed_m_s)


def _joint_move(name, status, q_from, q_to, cfg, n=20) -> Segment:
    pts = [q_from + (q_to - q_from) * k / n for k in range(1, n + 1)]
    return Segment(name, status, pts, _duration(pts, q_from, 0.0, cfg))


def _line(name, status, q_from, p_to, R, cfg, gripper_after=None) -> Segment:
    pts = K.cartesian_path(q_from, p_to, R)
    if pts is None:
        raise ValueError(f"No continuous IK path for segment '{name}' to {np.round(p_to, 3)}")
    length = float(np.linalg.norm(np.array(p_to) - K.fk(q_from)[0]))
    return Segment(name, status, pts, _duration(pts, q_from, length, cfg), gripper_after)


def plan_pick_place(q_start: Sequence[float], pick_center: Sequence[float], place_center: Sequence[float],
                    box_size: Sequence[float], cfg: PlanConfig = PlanConfig()) -> List[Segment]:
    R = K.tool_down_rotation(cfg.tool_yaw_rad)
    h = float(box_size[2])
    pick_grip = np.array([pick_center[0], pick_center[1], pick_center[2] + h / 2 + cfg.grip_gap_m])
    place_grip = np.array([place_center[0], place_center[1],
                           place_center[2] + h / 2 + cfg.grip_gap_m + cfg.place_drop_m])
    pick_above = pick_grip + [0, 0, cfg.approach_height_m]
    pick_transfer = np.array([pick_grip[0], pick_grip[1], max(cfg.transfer_z_m, pick_above[2])])
    place_transfer = np.array([place_grip[0], place_grip[1], pick_transfer[2]])

    q0 = np.array(q_start, dtype=float)
    q_pick_transfer = K.ik(pick_transfer, R, _seed_for(pick_grip))
    if q_pick_transfer is None:
        raise ValueError("Pick transfer pose unreachable")

    segs: List[Segment] = []
    segs.append(_joint_move("to_pick_transfer", "로봇: 박스 위 대기 위치로 이동 중...", q0, q_pick_transfer, cfg))
    q = q_pick_transfer
    for name, status, target, grip in (
        ("descend_pick", "로봇: 박스로 하강 중...", pick_grip, "attach"),
        ("lift", "로봇: 박스 파지 후 들어올리는 중...", pick_transfer, None),
        ("transfer", "로봇: 팔레트 위로 이송 중...", place_transfer, None),
        ("descend_place", "로봇: 팔레트로 하강 중...", place_grip, "detach"),
        ("retreat", "로봇: 박스 놓고 상승 중...", place_grip + [0, 0, cfg.approach_height_m], None),
    ):
        seg = _line(name, status, q, target, R, cfg, grip)
        segs.append(seg)
        q = seg.points[-1]
    segs.append(_joint_move("home", "로봇: 홈 자세로 복귀 중...", q, K.HOME, cfg))
    return segs
