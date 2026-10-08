#!/usr/bin/env python3
"""V4.4 suction-cup physical test (commissioning fixture, not a motion planner).

Preconditions: V4.4 launch running, V4.3 cycle done (box resting at PICK),
run_suction_gripper.sh running. Moves the robot with a simple vertical
approach computed by scripts/hdr50_kinematics.py (reference IK; path planning
and collision checking will move to MoveIt2), then checks:
  vacuum on + contact -> GRIPPED, box lifts with the cup,
  vacuum off -> released, box rests again on the conveyor.
"""
from __future__ import annotations

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
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hdr50_kinematics as K

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
BOX = "v43_scale_box_5kg"
BOX_H = 0.25
CUP_LEN = 0.06          # flange face -> suction face (hdr50_pedestal_gripper.urdf.xacro)
PRESS = 0.006           # commanded suction face below box top (joint sag ~3 mm + lip compression)
APPROACH = 0.15         # vertical approach / lift height
SETTLE_S = 1.5          # position-controlled joints lag the trajectory end slightly
JOINT_TOL = 0.01


class Failure(RuntimeError):
    pass


class SuctionTest(Node):
    def __init__(self):
        super().__init__("pac_suction_test_v44")
        self.q = None
        self.box = None
        self.suction = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, f"/model/{BOX}/pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(String, "/pac/suction/state", lambda m: setattr(self, "suction", m.data), 10)
        self.vacuum = self.create_publisher(Bool, "/pac/suction/vacuum", 10)
        self.traj = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self.goal = None

    def on_js(self, m):
        i = {n: k for k, n in enumerate(m.name)}
        if all(j in i for j in JOINTS):
            self.q = np.array([m.position[i[j]] for j in JOINTS])

    def on_pose(self, m):
        self.box = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return
        raise Failure(f"timeout waiting for {what}")

    def move(self, points, duration):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        for k, q in enumerate(points, 1):
            t = duration * k / len(points)
            g.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(v) for v in q], time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
        f = self.traj.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)
        if f.result() is None or not f.result().accepted:
            raise Failure("trajectory rejected")
        self.goal = f.result()
        r = self.goal.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=duration + 10)
        self.goal = None
        if r.result() is None or r.result().result.error_code != 0:
            raise Failure("trajectory failed")
        self.wait(lambda: np.abs(self.q - points[-1]).max() < JOINT_TOL, SETTLE_S + 3, "joints to settle")

    def line(self, p_flange, duration):
        pts = K.cartesian_path(self.q, p_flange, K.tool_down_rotation(0.0), step_m=0.01)
        if pts is None:
            raise Failure(f"no IK path to {np.round(p_flange, 3)}")
        self.move(pts, duration)

    def set_vacuum(self, on):
        for _ in range(3):
            self.vacuum.publish(Bool(data=on))
            self.spin_for(0.05)

    def run(self):
        print(">>> [흡착시험 1/6] 로봇·박스·흡착 그리퍼 상태 확인 중...", flush=True)
        if not self.traj.wait_for_server(timeout_sec=10):
            raise Failure("joint_trajectory_controller not available")
        self.wait(lambda: self.q is not None and self.box is not None and self.suction is not None, 10,
                  "/joint_states, box pose, /pac/suction/state (is run_suction_gripper.sh running?)")
        if self.suction != "OFF":
            raise Failure(f"suction must start OFF (is {self.suction})")
        b0 = self.box.copy()
        if not (b0[0] >= -1.10 and abs(b0[1] - 1.20) < 0.05):
            raise Failure(f"box is not resting at PICK (x={b0[0]:.3f}, y={b0[1]:.3f}); run V4.3 first")
        top = b0[2] + BOX_H / 2
        above = np.array([b0[0], b0[1], top + APPROACH + CUP_LEN])
        contact = np.array([b0[0], b0[1], top - PRESS + CUP_LEN])
        print(f"    box at ({b0[0]:.3f}, {b0[1]:.3f}, {b0[2]:.3f}) m", flush=True)

        print(">>> [흡착시험 2/6] 박스 위로 이동 중...", flush=True)
        seed = [np.arctan2(b0[1], b0[0]), 1.2, 0, 0, -1.0, 0]
        q_above = K.ik(above, K.tool_down_rotation(0.0), seed)
        if q_above is None:
            raise Failure("above-box pose unreachable")
        self.move([self.q + (q_above - self.q) * k / 30 for k in range(1, 31)], 6.0)

        print(">>> [흡착시험 3/6] 흡착컵을 박스 윗면에 접촉시키는 중...", flush=True)
        self.line(contact, 3.0)
        if self.suction != "OFF" or np.linalg.norm(self.box - b0) > 0.01:
            raise Failure("box moved or suction changed before vacuum was turned on")

        print(">>> [흡착시험 4/6] 진공 ON, 흡착 확인 중...", flush=True)
        self.set_vacuum(True)
        self.wait(lambda: self.suction == "GRIPPED", 5, "suction GRIPPED (cup-box contact)")

        print(">>> [흡착시험 5/6] 박스를 들어올리는 중...", flush=True)
        self.line(above, 3.0)
        self.spin_for(1.0)
        rise = self.box[2] - b0[2]
        print(f"    lifted box z={self.box[2]:.3f} m (rise {rise * 1000:.0f} mm, target {APPROACH * 1000:.0f} mm)", flush=True)
        if rise < APPROACH - 0.02:
            raise Failure(f"box did not follow the suction cup (rise {rise:.3f} m)")

        print(">>> [흡착시험 6/6] 박스 내려놓고 진공 OFF(해제) 중...", flush=True)
        self.line(contact, 3.0)
        self.set_vacuum(False)
        self.wait(lambda: self.suction == "OFF", 5, "suction OFF")
        self.spin_for(0.5)
        self.line(above, 2.0)
        self.spin_for(1.0)
        d = float(np.linalg.norm(self.box - b0))
        print(f"    box after release ({self.box[0]:.3f}, {self.box[1]:.3f}, {self.box[2]:.3f}) m, moved {d * 1000:.1f} mm", flush=True)
        if self.box[2] - b0[2] > 0.01:
            raise Failure("box still lifted after vacuum off (not released)")
        self.move([self.q + (K.HOME - self.q) * k / 30 for k in range(1, 31)], 6.0)
        return rise, d


def _raise(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    node = SuctionTest()
    try:
        rise, d = node.run()
        print(f"\nSUCTION PASS: gripped on contact, lifted {rise * 1000:.0f} mm with the cup, "
              f"released on vacuum off (box displacement after cycle {d * 1000:.1f} mm).", flush=True)
        return 0
    except Failure as e:
        node.set_vacuum(False) if node.suction == "SEARCHING" else None
        print(f"\nSUCTION FAIL: {e}", flush=True)
        return 1
    except KeyboardInterrupt:
        if node.goal is not None:
            rclpy.spin_until_future_complete(node, node.goal.cancel_goal_async(), timeout_sec=2)
        print("\nStopped by user; robot motion cancelled.", flush=True)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
