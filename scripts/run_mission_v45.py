#!/usr/bin/env python3
"""V4.5 robot node: place the weighed box where the team planner ranked it.

Input : placement JSON from plan_placement_v45.py (PLANNED, ranked, world centres).
Output: plain(ExecutionResult) JSON for the State Manager (this node never commits state).

Robot validation (team flow step R): candidates are tried in planner order and
the first one with a continuous IK path is executed; IK-rejected candidates are
recorded with code IK_FAIL. Reuses the V4.4 node for ROS I/O, trajectory
execution and gripper handling; V4.4 files are not modified.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mission_bridge_v45 as MB  # noqa: E402
from pick_place_plan_v44 import PlanConfig  # noqa: E402
from pick_place_plan_v45 import plan_pick_place_yaw  # noqa: E402
from run_pick_place_v44 import (LANE_TOL, LANE_Y, LIFT_MIN_RISE_M, PICK_MIN_X, PLACE_MAX_TILT_RAD,  # noqa: E402
                                PLACE_TOL_XY_M, PLACE_TOL_Z_M, Failure, PickPlace, _raise_interrupt)

PLACE_TOL_YAW_RAD = math.radians(5.0)


class MissionPickPlace(PickPlace):
    def __init__(self, placement: dict) -> None:
        super().__init__()
        self.placement = placement
        self.box_size = tuple(placement["box_size_m"])
        self.chosen = None
        self.robot_rejected = []

    def choose(self, q, pick, box_yaw):
        for cand in self.placement["placements"]:
            c = cand["center_world"]
            delta = MB.gripper_yaw_delta(c["yaw"], box_yaw)
            try:
                segs = plan_pick_place_yaw(q, pick, (c["x"], c["y"], c["z"]), self.box_size, delta, PlanConfig())
            except ValueError as exc:
                self.robot_rejected.append({"candidate_id": cand["candidate_id"], "code": "IK_FAIL",
                                            "reason": str(exc)})
                self.get_logger().warning(f"{cand['candidate_id']}: IK rejected ({exc})")
                continue
            return cand, segs
        raise Failure("No ranked candidate passed robot IK validation (IK_FAIL for all).")

    def run(self) -> str:
        print(">>> [로봇 1/4] 관절 상태·박스 위치 확인 중...", flush=True)
        if not self.traj.wait_for_server(timeout_sec=10.0):
            raise Failure("joint_trajectory_controller action server not available.")
        self.wait_for(lambda: self.q is not None, 10.0, "/joint_states")
        self.wait_for(lambda: self.box is not None, 10.0, "box pose")
        p, quat, _ = self.fresh_box()
        if not (p[0] >= PICK_MIN_X and abs(p[1] - LANE_Y) < LANE_TOL):
            raise Failure(f"Box is not at PICK: x={p[0]:.3f}, y={p[1]:.3f}.")
        p_before = p.copy()
        self.spin_for(0.5)
        if np.linalg.norm(self.fresh_box()[0] - p_before) > 0.005:
            raise Failure("Box is still moving at PICK.")
        box_yaw = MB.yaw_from_quat(quat)

        print(">>> [로봇 2/4] 플래너 후보별 IK 검증·경로 계산 중...", flush=True)
        self.chosen, segs = self.choose(self.q, p, box_yaw)
        c = self.chosen["center_world"]
        target = np.array([c["x"], c["y"], c["z"]])
        self.get_logger().info(f"Candidate {self.chosen['candidate_id']} (rank "
                               f"{self.placement['placements'].index(self.chosen) + 1}): centre "
                               f"({c['x']:.3f},{c['y']:.3f},{c['z']:.3f}) yaw {math.degrees(c['yaw']):.0f} deg, "
                               f"{len(segs)} segments, {sum(s.duration_s for s in segs):.1f} s")

        print(">>> [로봇 3/4] Pick & Place 실행 중...", flush=True)
        z_attach = None
        for seg in segs:
            print(f"    - {seg.status}", flush=True)
            self.run_segment(seg)
            if seg.gripper_after == "attach":
                print("    - 그리퍼: 박스 흡착(attach)", flush=True)
                z_attach = self.fresh_box()[0][2]
                self.gripper("attach")
            elif seg.gripper_after == "detach":
                print("    - 그리퍼: 박스 놓기(detach)", flush=True)
                self.gripper("detach")
                self.spin_for(1.0)
            if seg.name == "lift":
                rise = self.fresh_box()[0][2] - z_attach
                if rise < LIFT_MIN_RISE_M:
                    raise Failure(f"Box did not follow the gripper (rise {rise:.3f} m); check attach.")

        print(">>> [로봇 4/4] 적재 결과 확인 중...", flush=True)
        self.spin_for(1.0)
        p, quat, _ = self.fresh_box()
        x, y, _, _ = quat
        tilt = 2.0 * math.asin(min(1.0, math.hypot(x, y)))
        yaw_err = abs(MB.gripper_yaw_delta(c["yaw"], MB.yaw_from_quat(quat)))
        err_xy = float(np.hypot(*(p[:2] - target[:2])))
        err_z = float(p[2] - target[2])
        self.actual = (p, MB.yaw_from_quat(quat))
        summary = (f"{self.chosen['candidate_id']} placed at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m; "
                   f"xy {err_xy*1000:.1f} mm, z {err_z*1000:+.1f} mm, yaw {math.degrees(yaw_err):.1f} deg, "
                   f"tilt {math.degrees(tilt):.2f} deg")
        self.get_logger().info(summary)
        if (err_xy > PLACE_TOL_XY_M or abs(err_z) > PLACE_TOL_Z_M or tilt > PLACE_MAX_TILT_RAD
                or yaw_err > PLACE_TOL_YAW_RAD):
            raise Failure("Placed box outside tolerance (xy 30 mm, z 20 mm, yaw 5 deg, tilt 5 deg): " + summary)
        return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--placement", required=True)
    ap.add_argument("--result-out", required=True)
    args = ap.parse_args()
    placement = json.loads(Path(args.placement).read_text())
    if placement.get("schema") != "pac-v45-placement-1" or not placement["placements"]:
        print("V4.5 FAIL: invalid placement file", flush=True)
        return 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = MissionPickPlace(placement)
    box_id = placement["box_id"]
    ok, codes, summary = False, ["EXECUTION_FAIL"], ""
    try:
        summary = node.run()
        ok, codes = True, []
        print(f">>> [완료] 플래너 후보대로 적재 성공\n\nV4.5 PASS: {summary}", flush=True)
        return 0
    except Failure as exc:
        summary = str(exc)
        if not node.chosen:
            codes = ["IK_FAIL"] if node.robot_rejected else ["INVALID_STATE"]
        node.get_logger().error(summary)
        print(f">>> [중단] 로봇 동작 실패, 정지\n\nV4.5 FAIL: {summary}", flush=True)
        return 1
    except KeyboardInterrupt:
        if node.active_goal is not None:
            fut = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        summary = "stopped by user"
        print("\nStopped by user; robot motion cancelled (gripper state unchanged).", flush=True)
        return 130
    finally:
        actual = None
        if getattr(node, "actual", None) is not None:
            p, yaw = node.actual
            actual = MB.world_to_pallet_corner(p, yaw, node.box_size)
        result = MB.execution_result(
            success=ok, box_id=box_id, candidate_id=(node.chosen or {}).get("candidate_id", ""),
            actual_pose=actual, codes=codes, stamp_sec=time.time(),
            detail={"summary": summary, "robot_rejected": node.robot_rejected,
                    "base_state_version": (node.chosen or {}).get("base_state_version"),
                    "measured_kg": placement["measured_kg"]},
        )
        Path(args.result_out).write_text(json.dumps(result, ensure_ascii=False, indent=1))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
