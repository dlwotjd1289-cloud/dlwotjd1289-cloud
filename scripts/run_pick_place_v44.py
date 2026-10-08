#!/usr/bin/env python3
"""V4.4 commissioning node: HDR50-22 picks the weighed box at PICK and places it on the pallet.

Requires the V4.4 launch (V4.2 world + simulated vacuum gripper) and a bridge for
box pose and gripper attach/detach. Box pose is Gazebo ground truth (test harness),
not Top-view perception. No collision checking: see pick_place_plan_v44.py.
"""
from __future__ import annotations

import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hdr50_kinematics as K
from pick_place_plan_v44 import PlanConfig, plan_pick_place

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
BOX = "v43_scale_box_5kg"
BOX_SIZE = (0.40, 0.30, 0.25)
PLACE_CENTER = (0.0, 1.20, 0.15 + BOX_SIZE[2] / 2)  # pallet_main center, deck top z=0.15
PICK_MIN_X = -1.10           # V4.3 DONE zone (pick_x_m=-1.08 minus margin)
LANE_Y, LANE_TOL = 1.20, 0.05
LIFT_MIN_RISE_M = 0.20       # box must follow the flange after attach
PLACE_TOL_XY_M = 0.03
PLACE_TOL_Z_M = 0.02
PLACE_MAX_TILT_RAD = math.radians(5.0)


class Failure(RuntimeError):
    pass


class PickPlace(Node):
    def __init__(self) -> None:
        super().__init__("pac_pick_place_v44")
        self.q = None
        self.box = None          # (p[3], quat[4], stamp)
        self.active_goal = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(PoseStamped, f"/model/{BOX}/pose", self.on_pose, qos_profile_sensor_data)
        self.attach_pub = self.create_publisher(Empty, "/pac/gripper/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/pac/gripper/detach", 10)
        self.traj = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")

    def on_joints(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.q = np.array([msg.position[idx[j]] for j in JOINTS])

    def on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self.box = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]), time.monotonic())

    # -- helpers -----------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, cond, timeout: float, what: str) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return
        raise Failure(f"Timed out waiting for {what} ({timeout:.0f} s).")

    def fresh_box(self):
        if self.box is None or time.monotonic() - self.box[2] > 0.5:
            raise Failure("Box pose stream is stale.")
        return self.box

    def gripper(self, action: str) -> None:
        pub = self.attach_pub if action == "attach" else self.detach_pub
        for _ in range(3):
            pub.publish(Empty())
            self.spin_for(0.1)
        self.spin_for(0.5)

    def run_segment(self, seg) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        dt = seg.duration_s / len(seg.points)
        for k, q in enumerate(seg.points, start=1):
            t = dt * k
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise Failure(f"Trajectory '{seg.name}' was rejected by joint_trajectory_controller.")
        self.active_goal = handle
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seg.duration_s + 10.0)
        if res.result() is None:
            raise Failure(f"Trajectory '{seg.name}' did not finish in time.")
        self.active_goal = None
        code = res.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise Failure(f"Trajectory '{seg.name}' failed (error_code={code}).")
        err = np.abs(self.q - seg.points[-1]).max() if self.q is not None else float("nan")
        if not err < 0.02:
            raise Failure(f"Robot did not reach end of '{seg.name}' (max joint error {err:.3f} rad).")

    # -- cycle -------------------------------------------------------------
    def run(self) -> str:
        print(">>> [로봇 1/4] 관절 상태·박스 위치 확인 중...", flush=True)
        if not self.traj.wait_for_server(timeout_sec=10.0):
            raise Failure("joint_trajectory_controller action server not available.")
        self.wait_for(lambda: self.q is not None, 10.0, "/joint_states")
        self.wait_for(lambda: self.box is not None, 10.0, f"/model/{BOX}/pose")
        p, _, _ = self.fresh_box()
        if not (p[0] >= PICK_MIN_X and abs(p[1] - LANE_Y) < LANE_TOL):
            raise Failure(f"Box is not at PICK: x={p[0]:.3f}, y={p[1]:.3f}.")
        p_before = p.copy()
        self.spin_for(0.5)
        if np.linalg.norm(self.fresh_box()[0] - p_before) > 0.005:
            raise Failure("Box is still moving at PICK.")

        print(">>> [로봇 2/4] 집기·놓기 경로 계산 중 (IK)...", flush=True)
        try:
            segs = plan_pick_place(self.q, p, PLACE_CENTER, BOX_SIZE, PlanConfig())
        except ValueError as exc:
            raise Failure(f"Path planning failed: {exc}") from exc
        total = sum(s.duration_s for s in segs)
        self.get_logger().info(f"Plan: {len(segs)} segments, {total:.1f} s, pick=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")

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
                self.get_logger().info(f"Box lifted with gripper: rise {rise:.3f} m")

        print(">>> [로봇 4/4] 적재 결과 확인 중...", flush=True)
        self.spin_for(1.0)
        p, quat, _ = self.fresh_box()
        x, y, z, w = quat
        tilt = 2.0 * math.asin(min(1.0, math.hypot(x, y)))  # angle of box z-axis from vertical (approx.)
        err_xy = float(np.hypot(p[0] - PLACE_CENTER[0], p[1] - PLACE_CENTER[1]))
        err_z = float(p[2] - PLACE_CENTER[2])
        self.get_logger().info(f"Placed box: ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}), xy err {err_xy*1000:.1f} mm, "
                               f"z err {err_z*1000:+.1f} mm, tilt {math.degrees(tilt):.2f} deg")
        if err_xy > PLACE_TOL_XY_M or abs(err_z) > PLACE_TOL_Z_M or tilt > PLACE_MAX_TILT_RAD:
            raise Failure("Placed box outside tolerance (xy 30 mm, z 20 mm, tilt 5 deg).")
        return (f"placed at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m; xy error {err_xy*1000:.1f} mm, "
                f"z error {err_z*1000:+.1f} mm, tilt {math.degrees(tilt):.2f} deg")


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main() -> int:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = PickPlace()
    try:
        result = node.run()
        print(f">>> [완료] 로봇 적재 성공\n\nV4.4 PASS: {result}", flush=True)
        return 0
    except Failure as exc:
        node.get_logger().error(str(exc))
        print(f">>> [중단] 로봇 동작 실패, 정지\n\nV4.4 FAIL: {exc}", flush=True)
        return 1
    except KeyboardInterrupt:
        if node.active_goal is not None:
            # Cancelling makes joint_trajectory_controller hold the current position.
            fut = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        print("\nStopped by user; robot motion cancelled (gripper state unchanged).", flush=True)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
