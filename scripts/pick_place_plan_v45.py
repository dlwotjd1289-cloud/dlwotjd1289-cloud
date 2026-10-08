"""V4.5 pick & place plan: V4.4 path plus a wrist turn so the box lands at the planner's yaw.

Reuses the V4.4 segment builders unchanged. The turn happens at transfer height
above the place point (clear of the stopper and pallet), then the tool keeps
the new orientation for descend / release / retreat. Same commissioning
assumption as V4.4: no collision checking.
"""
from __future__ import annotations

from typing import List, Sequence

import numpy as np

import hdr50_kinematics as K
from pick_place_plan_v44 import PlanConfig, Segment, _joint_move, _line, _seed_for

MAX_TURN_ARM_CHANGE_RAD = 0.5   # j1..j5 may move only a little while the wrist turns


def plan_pick_place_yaw(q_start: Sequence[float], pick_center: Sequence[float], place_center: Sequence[float],
                        box_size: Sequence[float], yaw_delta_rad: float,
                        cfg: PlanConfig = PlanConfig()) -> List[Segment]:
    R_pick = K.tool_down_rotation(cfg.tool_yaw_rad)
    R_place = K.tool_down_rotation(cfg.tool_yaw_rad + yaw_delta_rad)
    h = float(box_size[2])
    pick_grip = np.array([pick_center[0], pick_center[1], pick_center[2] + h / 2 + cfg.grip_gap_m])
    place_grip = np.array([place_center[0], place_center[1],
                           place_center[2] + h / 2 + cfg.grip_gap_m + cfg.place_drop_m])
    pick_transfer = np.array([pick_grip[0], pick_grip[1], max(cfg.transfer_z_m, pick_grip[2] + cfg.approach_height_m)])
    place_transfer = np.array([place_grip[0], place_grip[1], pick_transfer[2]])

    q0 = np.array(q_start, dtype=float)
    q = K.ik(pick_transfer, R_pick, _seed_for(pick_grip))
    if q is None:
        raise ValueError("Pick transfer pose unreachable")
    segs: List[Segment] = [_joint_move("to_pick_transfer", "로봇: 박스 위 대기 위치로 이동 중...", q0, q, cfg)]

    def line(name, status, target, R, grip=None):
        nonlocal q
        seg = _line(name, status, q, target, R, cfg, grip)
        segs.append(seg)
        q = seg.points[-1]

    line("descend_pick", "로봇: 박스로 하강 중...", pick_grip, R_pick, "attach")
    line("lift", "로봇: 박스 파지 후 들어올리는 중...", pick_transfer, R_pick)
    line("transfer", "로봇: 팔레트 위로 이송 중...", place_transfer, R_pick)
    if abs(yaw_delta_rad) > 1e-6:
        q_turn = K.ik(place_transfer, R_place, q)
        if q_turn is None:
            raise ValueError("Wrist turn at place transfer unreachable")
        if np.abs(q_turn[:5] - q[:5]).max() > MAX_TURN_ARM_CHANGE_RAD:
            raise ValueError("Wrist turn needs a large arm reconfiguration")
        segs.append(_joint_move("turn", "로봇: 적재 방향으로 박스 회전 중...", q, q_turn, cfg))
        q = q_turn
    line("descend_place", "로봇: 팔레트로 하강 중...", place_grip, R_place, "detach")
    line("retreat", "로봇: 박스 놓고 상승 중...", place_grip + [0, 0, cfg.approach_height_m], R_place)
    segs.append(_joint_move("home", "로봇: 홈 자세로 복귀 중...", q, K.HOME, cfg))
    return segs
