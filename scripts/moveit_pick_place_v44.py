#!/usr/bin/env python3
"""V4.4 MoveIt2 pick & place: weighed box at PICK -> pallet, with the suction cup.

Requires: hdr50_workcell_v4_4_pick.launch.py, hdr50_moveit_v44.launch.py,
run_suction_gripper.sh, and a bridge for /model/v43_scale_box_5kg/pose.
Planning (deterministic): free moves without payload = IK from a fixed reference
posture + Pilz PTP (same start/goal -> same path; collision-checked by MoveIt),
OMPL only as a logged fallback if PTP is blocked; move_group Cartesian paths
(collision-checked) for approach, level transfer with the box, and retreat.
Pick pose: Gazebo ground truth (--pose-source gazebo) or the fixed pole CCTV + gripper camera
(--pose-source camera; ground truth then only reports the perception error).
"""
from __future__ import annotations

import json
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, BoundingVolume, CollisionObject,
                             Constraints, PlanningSceneComponents,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

GROUP = "hdr_manipulator"
TCP = "suction_tcp"
FRAME = "world"
BOX = "v43_scale_box_5kg"
BOX_ID = "box_5kg"
BOX_SIZE = (0.40, 0.30, 0.25)
TOUCH_LINKS = ["suction_cup_link", "suction_tcp", "flange_link"]
HOME = {"j1": 0.0, "j2": 1.5707, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0}

PALLET_TOP_Z = 0.15
PLACE_XY = (0.0, 1.20)            # pallet_main center
APPROACH_M = 0.20                 # vertical approach / retreat
CHECKED_GAP_M = 0.010             # collision-checked descent stops this far above contact
PRESS_M = 0.006                   # final unchecked press (joint sag ~3 mm + lip)
PLACE_GAP_M = 0.005               # box bottom above pallet when released
VEL_SCALE = 0.2                   # joint_limits.yaml: keep <= 0.2
# Speeds (fraction of joint limits). joint_limits.yaml suggests <= 0.2 for the real controller;
# in Gazebo the position-controlled joints abort (PATH_TOLERANCE_VIOLATED, 0.2 rad) on a long
# PTP move at 0.4 (home -> PICK, 2026-10-09); 0.25 / 0.20 are 2.5x / 2x the earlier 0.1.
# Contact moves stay slow.
FREE_SPEED = 0.18                 # Pilz PTP, empty gripper (0.25 aborted on a long return move once)
CART_SPEED = 0.20                 # straight lines (lift / transfer / lower, with the box)
CONTACT_SPEED = 0.10              # final approach and cup press on the box, last part of lowering
SLOW_LOWER_M = 0.06               # lowered slowly between the neighbours
# Idle pose between cycles: above the PICK zone (cup down, facing the conveyor), offset toward the
# robot so the arm does not hide the PICK box from the pole CCTV. Replaces the HOME pose, which
# points away from the conveyor and lengthens every cycle.
READY_TCP = (-0.95, 0.95, 1.55)
PLACE_TOL_XY_M, PLACE_TOL_Z_M, PLACE_TILT_DEG = 0.03, 0.02, 5.0
PLACE_YAW_DEG = 3.0
INSPECT_M = 0.40                  # gripper-camera viewpoint height above the box top
STACK_CLEAR_M = 0.10              # carried box bottom above the highest placed box during transfer
DISTURB_TOL_M = 0.005             # an already placed box may not move more than this
BUFFER_TOP_Z = 0.9425            # buffer table shelf top
# Gripper payload (rated, safety factor included): foam/area suction pad sized for 30 kg cartons.
# Extension options for heavier boxes (illustrative ratings until the EOAT is selected):
GRIPPERS = {"pad": 30.0, "pad_xl": 45.0, "pad_fork": 60.0}
PICK_ATTEMPTS = 3                 # pick tries (re-perceive + re-grip after a failed one)
REPLACE_ATTEMPTS = 1              # re-grip on the pallet when the camera finds the box off target

# Workcell planning scene (world frame), from ahead_workcell_v4_2_physical_scale.sdf
# collision geometry; boxes as (name, center xyz, size xyz).
SCENE = [
    ("floor", (-1.0, -0.5, -0.05), (9.0, 7.0, 0.1)),
    ("pedestal", (0.0, 0.0, 0.195), (0.90, 0.90, 0.39)),           # top kept 1 cm below base_link
    ("conveyor", (-2.95, 1.20, 0.445), (4.18, 0.80, 0.89)),        # supports + rollers, top 0.89
    ("pick_stopper", (-0.82, 1.20, 0.96), (0.05, 0.66, 0.18)),
    ("pallet", (0.0, 1.20, 0.075), (1.10, 1.10, 0.15)),
    # buffer table (V4.4: 2 bays on the 0.92 m shelf, posts 0.97 m)
    *[(f"buffer_post_{i}", (1.45 + dx, -0.55 + dy, 0.485), (0.055, 0.055, 0.97))
      for i, (dx, dy) in enumerate(((-0.43, -0.26), (0.43, -0.26), (-0.43, 0.26), (0.43, 0.26)))],
    *[(f"buffer_divider_{i}", (1.45, -0.55 + dy, 0.485), (0.04, 0.04, 0.97)) for i, dy in enumerate((-0.26, 0.26))],
    ("buffer_shelf_low", (1.45, -0.55, 0.34), (0.90, 0.58, 0.045)),
    ("buffer_shelf", (1.45, -0.55, 0.92), (0.90, 0.58, 0.045)),
    ("camera_pole", (-0.60, 1.95, 1.17), (0.24, 0.24, 2.34)),
]


class Failure(RuntimeError):
    pass


class Fatal(Failure):
    """Not recoverable by the executor (cell stops, operator)."""


def gz_world_poses(world="ahead_workcell_v4_2_physical_scale"):
    """{model name: (xyz, quat xyzw)} for all top-level models, from one /world/<w>/pose/info
    message (~0.4 s; `ign model -p` takes ~4 s per model)."""
    import re
    import subprocess
    for _ in range(3):
        try:
            out = subprocess.run(["ign", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
                                 capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            out = ""
        poses = {}
        for blk in out.split("pose {")[1:]:
            m = re.search(r'name: "([^"]+)"', blk)
            v = [float(x) for x in re.findall(r"[xyzw]: (-?[0-9.e+-]+)", blk)[:7]]
            if m and len(v) == 7 and m.group(1) not in poses:
                poses[m.group(1)] = (np.array(v[:3]), tuple(v[3:7]))
        if poses:
            return poses
        time.sleep(0.5)
    return {}


def gz_model_pose(name, snapshot=None):
    """(xyz, quat xyzw) of a Gazebo model (world pose snapshot, else `ign model -p`), or None."""
    snap = gz_world_poses() if snapshot is None else snapshot
    if name in snap:
        return snap[name]
    return gz_model_pose_slow(name)


def gz_model_pose_slow(name):
    """(xyz, quat xyzw) of a Gazebo model from `ign model -p`, or None if absent."""
    import re
    import subprocess
    # `ign model` occasionally returns nothing (transient service timeout): retry before
    # concluding the model is absent.
    for _ in range(3):
        try:
            out = subprocess.run(["ign", "model", "-m", name, "-p"], capture_output=True, text=True,
                                 timeout=15).stdout
        except subprocess.TimeoutExpired:
            out = ""
        if f"Name: {name}" in out and "Pose" in out:
            break
        time.sleep(1.0)
    else:
        return None
    nums = [float(v) for v in re.findall(r"-?\d+\.\d+", out.split("Pose")[-1])[:6]]
    r, p, y = nums[3:6]
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    quat = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    return np.array(nums[:3]), quat


def seg_rect_dist(a, b, c, size, yaw):
    """Distance from segment a-b (xy) to an axis-aligned-or-90 deg box footprint centred at c."""
    sx, sy = (size[1], size[0]) if abs(math.sin(yaw)) > 0.7 else (size[0], size[1])
    a, b = np.asarray(a, float), np.asarray(b, float)
    best = 1e9
    for t in np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(b - a) / 0.05) + 1)):
        q = a + t * (b - a)
        dx = max(abs(q[0] - c[0]) - sx / 2, 0.0)
        dy = max(abs(q[1] - c[1]) - sy / 2, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def tool_down_quat(yaw: float) -> Quaternion:
    """TCP +x pointing down, TCP +y along world yaw (rotation matrix -> quaternion)."""
    x = np.array([0.0, 0.0, -1.0])
    y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    R = np.column_stack([x, y, np.cross(x, y)])
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-6:
        return Quaternion(x=(R[2, 1] - R[1, 2]) / (4 * w), y=(R[0, 2] - R[2, 0]) / (4 * w),
                          z=(R[1, 0] - R[0, 1]) / (4 * w), w=w)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2.0
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4.0
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return Quaternion(x=q[0], y=q[1], z=q[2], w=(R[k, j] - R[j, k]) / s)


def yaw_of(q) -> float:
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def box_object(name, center, size, quat=(0.0, 0.0, 0.0, 1.0), op=CollisionObject.ADD) -> CollisionObject:
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = name
    co.operation = op
    if op == CollisionObject.ADD:
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in center)
        p.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        co.primitive_poses = [p]
    return co


class MoveItPickPlace(Node):
    def __init__(self, box=BOX):
        super().__init__("pac_moveit_pick_place_v44",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.box = None
        self.suction = None
        self.create_subscription(PoseStamped, f"/model/{box}/pose", self.on_pose, qos_profile_sensor_data)
        self.target_pub = self.create_publisher(String, "/pac/suction/target", 10)
        self.percept = {}
        self.percept_target_pub = self.create_publisher(String, "/pac/perception/target", 10)
        for cam in ("far", "wrist"):
            self.create_subscription(PoseStamped, f"/pac/perception/{cam}/box_pose",
                                     lambda m, c=cam: self.on_percept(m, c), 10)
        self.create_subscription(String, "/pac/suction/state", lambda m: setattr(self, "suction", m.data), 10)
        self.vacuum_pub = self.create_publisher(Bool, "/pac/suction/vacuum", 10)
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.planners_used = []
        self.active = None
        self.recoveries = []
        self.placed_measured = None
        self.joints = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def on_percept(self, m, cam):
        p, o = m.pose.position, m.pose.orientation
        self.percept[cam] = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w), time.monotonic())

    def on_pose(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.box = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w), time.monotonic())

    # ---- ROS helpers -----------------------------------------------------
    def wait(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return
        raise Failure(f"timeout waiting for {what}")

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, req, timeout=5.0, attempts=3):
        # Retry: right after start-up the server may not yet have discovered this client's
        # response reader (move_group logged "failed to send response ... (timeout)").
        # Only idempotent requests (scene edits, IK, Cartesian queries) go through here.
        for _ in range(attempts):
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            if fut.result() is not None:
                return fut.result()
            client.remove_pending_request(fut)
        raise Failure(f"service {client.srv_name} did not answer ({attempts} attempts)")

    def run_action(self, client, goal, timeout, what):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise Failure(f"{what}: goal rejected")
        self.active = handle
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        self.active = None
        if res.result() is None:
            raise Failure(f"{what}: no result within {timeout:.0f} s")
        code = res.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise Failure(f"{what}: MoveIt error_code {code}")
        self.spin_for(0.3)  # let position-controlled joints settle

    def fresh_box(self):
        if self.box is None or time.monotonic() - self.box[2] > 0.5:
            raise Failure("box pose stream is stale")
        return self.box

    def apply_scene(self, objects=(), attached=(), check=True):
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = list(objects)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = list(attached)
        if not self.call(self.scene_cli, ApplyPlanningScene.Request(scene=ps)).success and check:
            raise Failure("apply_planning_scene failed")

    def set_allowed_collision(self, a, b, allowed):
        """Edit one pair of the allowed collision matrix. A diff with an ACM replaces the
        whole matrix, so read the current one, change the pair, and send it back."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call(self.get_scene_cli, req).scene.allowed_collision_matrix
        names = list(acm.entry_names)
        for n in (a, b):
            if n not in names:
                names.append(n)
                for e in acm.entry_values:
                    e.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        acm.entry_names = names
        ia, ib = names.index(a), names.index(b)
        acm.entry_values[ia].enabled[ib] = allowed
        acm.entry_values[ib].enabled[ia] = allowed
        ps = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        if not self.call(self.scene_cli, ApplyPlanningScene.Request(scene=ps)).success:
            raise Failure(f"could not update allowed collision {a}<->{b}")

    def set_vacuum(self, on):
        for _ in range(3):
            self.vacuum_pub.publish(Bool(data=on))
            self.spin_for(0.05)

    # ---- motion ------------------------------------------------------------
    def ik(self, xyz, yaw, what):
        """Joint goal for a tool-down TCP pose, seeded from a fixed reference posture
        (elbow up, wrist unflipped) so the same target always gives the same joints."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TCP
        r.avoid_collisions = True
        r.robot_state.joint_state.name = list(HOME)
        r.robot_state.joint_state.position = [math.atan2(xyz[1], xyz[0]), 1.2, 0.0, 0.0, -1.0, 0.0]
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in xyz)
        ps.pose.orientation = tool_down_quat(yaw)
        r.pose_stamped = ps
        r.timeout.sec = 1
        res = self.call(self.ik_cli, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise Failure(f"{what}: no collision-free IK (error_code {res.error_code.val})")
        js = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        goal = {j: js[j] for j in HOME}
        if abs(goal["j4"]) > math.pi or abs(goal["j6"]) > math.pi:
            raise Failure(f"{what}: IK wrist outside +-pi (j4={goal['j4']:.2f}, j6={goal['j6']:.2f})")
        return goal

    def plan_pose(self, xyz, yaw, what):
        self.plan_joints(self.ik(xyz, yaw, what), what)

    def go_ready(self, what):
        """READY above PICK with an empty cup: the tool yaw is free, so take the one (0/90/180/-90 deg)
        that turns the wrist least. After a 90 deg placement j6 sat at -3.5 rad and a fixed yaw-0
        READY needed a 3.5 rad j6 turn that lagged past the 0.2 rad path tolerance (box_07)."""
        goals = []
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            try:
                g = self.ik(READY_TCP, yaw, what)
            except Failure:
                continue
            goals.append((sum(abs(g[j] - self.joints.get(j, g[j])) for j in ("j4", "j6")), g))
        if not goals:
            raise Failure(f"{what}: no IK for READY")
        self.plan_joints(min(goals, key=lambda t: t[0])[1], what)

    def plan_joints(self, joints, what):
        # Large wrist turns lag in Gazebo's position-controlled joints: scale the speed down so that
        # j4 / j6 moves of more than ~1.5 rad stay inside the 0.2 rad path tolerance.
        wrist = max((abs(joints[j] - self.joints[j]) for j in ("j4", "j6") if j in self.joints), default=0.0)
        self.speed = FREE_SPEED if wrist <= 1.5 else max(0.06, FREE_SPEED * 1.5 / wrist)
        if wrist > 1.5:
            print(f"    - {what}: wrist turn {wrist:.2f} rad, speed {self.speed:.2f}", flush=True)
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=v, tolerance_above=0.005,
                                               tolerance_below=0.005, weight=1.0) for n, v in joints.items()]
        try:
            self.plan_and_execute(c, what, pipeline="pilz", planner="PTP")
            self.planners_used.append(f"{what}: Pilz PTP")
        except Failure as exc:
            if "no result" in str(exc):
                raise
            if f"error_code {MoveItErrorCodes.CONTROL_FAILED}" in str(exc):
                # Controller abort (tracking error): the arm stopped where it was. Re-plan once
                # from the current state (same goal); a second abort stops the cycle.
                print(f"    ! {what}: {exc}; re-planning once from the current state", flush=True)
                self.recoveries.append(f"{what}: controller abort, re-planned")
                self.spin_for(1.0)
                self.plan_and_execute(c, what + " (retry)", pipeline="pilz", planner="PTP")
                self.planners_used.append(f"{what}: Pilz PTP retry")
                return
            print(f"    ! {what}: Pilz PTP blocked ({exc}); falling back to OMPL", flush=True)
            self.plan_and_execute(c, what, pipeline="ompl", planner="")
            self.planners_used.append(f"{what}: OMPL fallback")

    def plan_and_execute(self, constraints, what, pipeline="ompl", planner=""):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = GROUP
        r.pipeline_id = pipeline
        r.planner_id = planner
        r.num_planning_attempts = 5
        r.allowed_planning_time = 10.0
        # Gazebo's position-controlled joints lag on long, fast OMPL moves (large j6 turns
        # aborted with PATH_TOLERANCE_VIOLATED at 0.2): free moves run at half speed.
        r.max_velocity_scaling_factor = getattr(self, "speed", FREE_SPEED)
        r.max_acceleration_scaling_factor = getattr(self, "speed", FREE_SPEED)
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        g.planning_options.plan_only = False
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.run_action(self.move_ac, g, 120.0, what)

    def line(self, xyz, yaw, what, avoid_collisions=True, speed=CART_SPEED, _retry=False):
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state.is_diff = True
        req.group_name = GROUP
        req.link_name = TCP
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
        p.orientation = tool_down_quat(yaw)
        req.waypoints = [p]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed
        res = self.call(self.cart_cli, req)
        if res.fraction < 0.999:
            raise Failure(f"{what}: Cartesian path only {res.fraction * 100:.0f}% feasible")
        traj = res.solution
        # joint_trajectory_controller rejects a goal whose time_from_start is not strictly
        # increasing. A near-zero first segment (cup still touching the released box) produced
        # two points at t=0 once; drop such duplicates before sending.
        pts, kept, last_t = traj.joint_trajectory.points, [], -1.0
        for pt in pts:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if t > last_t + 1e-6 or not kept:
                kept.append(pt)
                last_t = t
        if len(kept) < 2:
            print(f"    - {what}: already there (no motion needed)", flush=True)
            return
        traj.joint_trajectory.points = kept
        g = ExecuteTrajectory.Goal(trajectory=traj)
        try:
            self.run_action(self.exec_ac, g, 60.0, what)
        except Failure as exc:
            if f"error_code {MoveItErrorCodes.CONTROL_FAILED}" not in str(exc) or _retry:
                raise
            # Controller abort: recompute the straight line from where the arm stopped, once.
            print(f"    ! {what}: {exc}; recomputing the line from the current state", flush=True)
            self.recoveries.append(f"{what}: controller abort, line recomputed")
            self.spin_for(1.0)
            self.line(xyz, yaw, what + " (retry)", avoid_collisions, speed, _retry=True)

    # ---- cycle -------------------------------------------------------------
    def wrist_look(self, box, size, xy, yaw, top, base_z=None, timeout=10.0, move=True):
        """Gripper camera from INSPECT_M above `xy` (box top at `top`): (centre, quat) or Failure.
        base_z: surface the box rests on (pallet re-perception); default: PICK on the conveyor."""
        if base_z is not None:
            msg = {"box": box, "size": list(size), "base_z": float(base_z), "expect": [float(xy[0]), float(xy[1])],
                   "expect_yaw": float(yaw)}
            for _ in range(3):
                self.percept_target_pub.publish(String(data=json.dumps(msg)))
                self.spin_for(0.05)
        if move:
            self.plan_pose((xy[0], xy[1], top + INSPECT_M), yaw, "move above box for gripper camera")
        # let the arm settle (image and TF must belong to the same, still pose)
        self.spin_for(0.5 if move else 1.0)
        t_req = time.monotonic()
        self.percept.pop("wrist", None)
        self.wait(lambda: "wrist" in self.percept and self.percept["wrist"][2] > t_req + 0.3, timeout,
                  "/pac/perception/wrist/box_pose (gripper camera)")
        b_w, q_w, _ = self.percept["wrist"]
        return b_w, q_w

    def grip(self, b, yaw, top, size, q, at_pick):
        """Approach the box top, press, vacuum on, wait GRIPPED, attach it in the planning scene,
        lift APPROACH_M straight up and check that the box followed."""
        self.apply_scene(objects=[box_object(BOX_ID, b, size, q)])
        self.line((b[0], b[1], top + APPROACH_M), yaw, "lower to approach height")
        self.line((b[0], b[1], top + CHECKED_GAP_M), yaw, "descend to box (collision-checked)", speed=CONTACT_SPEED)
        self.line((b[0], b[1], top - PRESS_M), yaw, "press cup on box", avoid_collisions=False, speed=CONTACT_SPEED)
        gate = os.environ.get("SPAWN_GATE") if at_pick else None
        if gate and not os.path.exists(gate):
            # The cycle runner creates the next box at the inlet now: the gripper's DetachableJoint
            # attaches every new box_NN on appearance (released right after), which must happen
            # while the arm stands still.
            print(">>> [MoveIt 4/7] 로봇 정지(압착) 중: 다음 박스를 컨베이어 입구에 생성 대기...", flush=True)
            end = time.monotonic() + 30.0
            while time.monotonic() < end and not os.path.exists(gate):
                self.spin_for(0.1)
            if not os.path.exists(gate):
                raise Fatal("next box was not created at the inlet within 30 s (SPAWN_GATE)")
        print(">>> [MoveIt 4/7] 진공 ON, 흡착 확인 후 박스를 로봇에 부착(attach) 중...", flush=True)
        self.set_vacuum(True)
        self.wait(lambda: self.suction == "GRIPPED", 5, "suction GRIPPED (vacuum switch)")
        aco = AttachedCollisionObject(link_name=TCP, touch_links=TOUCH_LINKS)
        aco.object.id = BOX_ID
        aco.object.header.frame_id = FRAME
        aco.object.operation = CollisionObject.ADD
        self.apply_scene(attached=[aco])
        z_before = self.fresh_box()[0][2]
        # As at real palletizing infeeds, the box rests against the end stop with the conveyor
        # stopped (no back pressure) and is lifted straight up. Only for this vertical lift is
        # box<->pick_stopper contact allowed; every other collision stays checked.
        if at_pick:
            self.set_allowed_collision(BOX_ID, "pick_stopper", True)
        try:
            self.line((b[0], b[1], top + APPROACH_M), yaw, "lift box straight up")
        finally:
            if at_pick:
                self.set_allowed_collision(BOX_ID, "pick_stopper", False)
        # Box-follow check. In this simulation the box pose stream stands in for the vacuum
        # level sensor of a real cup (a box that stays behind = vacuum lost). Wait for the stream
        # to catch up (under CPU load the first sample after the lift still lagged).
        end = time.monotonic() + 5.0
        while time.monotonic() < end and self.fresh_box()[0][2] - z_before < APPROACH_M - 0.03:
            self.spin_for(0.2)
        rise = self.fresh_box()[0][2] - z_before
        if rise < APPROACH_M - 0.03:
            raise Failure(f"box did not follow the suction cup (rise {rise:.3f} m)")
        return rise

    def back_off(self, b, yaw, top):
        """Recovery after a failed grip: vacuum off, release the box in the scene, retreat
        straight up from the box and go back to READY (box free again, cameras unobstructed)."""
        self.set_vacuum(False)
        try:
            self.wait(lambda: self.suction == "OFF", 3, "suction OFF")
        except Failure:
            pass
        self.apply_scene(attached=[AttachedCollisionObject(link_name=TCP, object=box_object(BOX_ID, b, (0.1, 0.1, 0.1), op=CollisionObject.REMOVE))], check=False)
        self.set_allowed_collision(BOX_ID, "suction_cup_link", True)
        try:
            z_now = self.tcp_z() or 0.0
            self.line((b[0], b[1], max(z_now, top + APPROACH_M + 0.05)), yaw, "retreat up from the box (recovery)",
                      avoid_collisions=False)
        except Failure as exc:
            print(f"    ! retreat: {exc}", flush=True)
        finally:
            self.set_allowed_collision(BOX_ID, "suction_cup_link", False)
        self.apply_scene(objects=[box_object(BOX_ID, b, (0.1, 0.1, 0.1), op=CollisionObject.REMOVE)], check=False)
        self.go_ready("back to ready pose (recovery)")

    def run(self, box=BOX, slot=None, placed=(), pose_source="gazebo", slot_yaw=None, size=BOX_SIZE,
            placed_sizes=None, pick_from="conveyor", pick_pose=None, place_on="pallet", mass=None, gripper="pad"):
        """Pick `box` (size x, y, z [m]) at PICK and place its center at `slot` (world xyz; default
        pallet center). `placed`: names of boxes already on the pallet (obstacles; must not be
        disturbed); `placed_sizes`: their sizes (default: this box's size).
        Recovery: a failed pick (camera, vacuum, box not following, aborted motion) is retried
        after backing off and re-perceiving (PICK_ATTEMPTS); a placement the gripper camera finds
        off target is re-gripped on the pallet and placed again (REPLACE_ATTEMPTS); a box dropped
        in transfer or not confirmed flat on the pallet stops the cell (operator)."""
        size = tuple(size)
        placed_sizes = placed_sizes or {}
        place = np.array(slot if slot is not None else (PLACE_XY[0], PLACE_XY[1], PALLET_TOP_Z + size[2] / 2))
        # Test faults: PAC_FAULT=kind[:box_NN][,...] (suction_once is handled by the suction node).
        # Gripper payload check (rated payload of the selected end effector).
        cap = GRIPPERS[gripper]
        if mass is not None and mass > cap:
            ext = [f"{g} {c:.0f} kg" for g, c in GRIPPERS.items() if c >= mass]
            raise Fatal(f"box {mass:.1f} kg exceeds the {gripper} gripper payload {cap:.0f} kg: "
                        f"extend the gripper ({', '.join(ext) or 'none rated high enough'})")
        faults = {f.partition(":")[0] for f in os.environ.get("PAC_FAULT", "").split(",")
                  if f and f.partition(":")[2] in ("", box)}
        print(f">>> [MoveIt 1/7] MoveIt·흡착 그리퍼·박스({box}) 상태 확인 중...", flush=True)
        for ac, n in ((self.move_ac, "/move_action"), (self.exec_ac, "/execute_trajectory")):
            if not ac.wait_for_server(timeout_sec=15.0):
                raise Fatal(f"{n} not available (start hdr50_moveit_v44.launch.py)")
        for cli in (self.scene_cli, self.cart_cli, self.ik_cli, self.get_scene_cli):
            if not cli.wait_for_service(timeout_sec=15.0):
                raise Fatal(f"{cli.srv_name} not available")
        self.wait(lambda: self.box is not None and self.suction is not None, 10,
                  f"/model/{box}/pose and /pac/suction/state (is run_suction_gripper.sh running?)")
        if self.suction != "OFF":
            raise Fatal(f"suction must start OFF (is {self.suction})")
        pick_target = json.dumps({"box": box, "size": list(size)})
        for _ in range(3):
            self.target_pub.publish(String(data=box))
            self.percept_target_pub.publish(String(data=pick_target))
            self.spin_for(0.05)

        print(f">>> [MoveIt 2/7] 작업셀 장애물·박스·기적재 박스 {len(placed)}개를 planning scene에 등록 중...", flush=True)
        snap = gz_world_poses()
        before = {n: gz_model_pose(n, snap) for n in placed}
        if any(v is None for v in before.values()):
            raise Fatal(f"placed box missing in Gazebo: {[n for n, v in before.items() if v is None]}")
        psz = {n: tuple(placed_sizes.get(n, size)) for n in before}
        # Forget boxes that left the cell (pallet change): placed_* objects not in this cycle's list.
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        known = [o.id for o in self.call(self.get_scene_cli, req).scene.world.collision_objects]
        stale = [n for n in known if n.startswith("placed_") and n[len("placed_"):] not in before]
        if stale:
            self.apply_scene(objects=[box_object(n, place, (0.1, 0.1, 0.1), op=CollisionObject.REMOVE) for n in stale],
                             check=False)
        self.apply_scene(attached=[AttachedCollisionObject(link_name=TCP, object=box_object(BOX_ID, place, size, op=CollisionObject.REMOVE))], check=False)
        self.apply_scene(objects=[box_object(BOX_ID, place, size, op=CollisionObject.REMOVE)], check=False)
        self.apply_scene(objects=[box_object(n, c, s) for n, c, s in SCENE]
                         + [box_object(f"placed_{n}", p_[0], psz[n], p_[1]) for n, p_ in before.items()])

        # ---- pick with retries ---------------------------------------------------------------
        notes = []
        for attempt in range(1, PICK_ATTEMPTS + 1):
            b = None
            try:
                truth_b, truth_q, _ = self.fresh_box()
                b, q = truth_b.copy(), truth_q
                if pick_from == "buffer":
                    # Box parked on the buffer table: start from where it was put, refine with the
                    # gripper camera (the pole CCTV does not see the buffer).
                    b = np.array(pick_pose[:3], float)
                    q = (0.0, 0.0, math.sin(pick_pose[3] / 2), math.cos(pick_pose[3] / 2))
                elif pose_source == "camera":
                    # Coarse pose from the fixed pole CCTV (robot at READY, view unobstructed);
                    # Gazebo ground truth is only used to report the errors.
                    for _ in range(3):
                        self.percept_target_pub.publish(String(data=pick_target))
                        self.spin_for(0.05)
                    t_req = time.monotonic()
                    self.percept.pop("far", None)
                    self.wait(lambda: "far" in self.percept and self.percept["far"][2] > t_req + 0.3, 15,
                              "/pac/perception/far/box_pose (is box_perception_v44.py running and the box visible?)")
                    b, q, _ = self.percept["far"]
                    err = np.linalg.norm((b - truth_b)[:2])
                    notes.append(f"far CCTV error {err * 1000:.1f} mm")
                    print(f"    [camera] far CCTV coarse pose error vs ground truth {err * 1000:.1f} mm", flush=True)
                # Box resting against the pick stopper: front face at x = -0.845 (any SKU length).
                if pick_from == "conveyor" and not (abs(b[0] + size[0] / 2 + 0.845) < 0.06 and abs(b[1] - 1.20) < 0.15):
                    raise Fatal(f"box is not at PICK (x={b[0]:.3f}, y={b[1]:.3f}); run V4.3 first")
                yaw = yaw_of(q)
                top = b[2] + size[2] / 2
                if attempt == 1:
                    print(f"    box at ({b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}) m, yaw {math.degrees(yaw):.1f} deg; "
                          f"target ({place[0]:.3f}, {place[1]:.3f}, {place[2]:.3f}) m", flush=True)
                    print(">>> [MoveIt 3/7] 박스 위 접근 위치로 이동 중 (IK + Pilz PTP, 충돌 검사)...", flush=True)
                self.apply_scene(objects=[box_object(BOX_ID, b, size, q)])
                if pose_source == "camera":
                    b_w, q_w = self.wrist_look(box, size, b, yaw, top,
                                               base_z=BUFFER_TOP_Z if pick_from == "buffer" else None)
                    shift = np.linalg.norm((b_w - b)[:2])
                    if shift > 0.08:
                        raise Failure(f"gripper camera disagrees with the CCTV by {shift * 1000:.0f} mm")
                    b, q = np.array([b_w[0], b_w[1], b[2]]), q_w
                    yaw = yaw_of(q)
                    err = np.linalg.norm((b - truth_b)[:2])
                    notes.append(f"gripper camera error {err * 1000:.1f} mm (refined by {shift * 1000:.0f} mm)")
                    print(f"    [camera] gripper camera refined pose error vs ground truth {err * 1000:.1f} mm", flush=True)
                rise = self.grip(b, yaw, top, size, q, at_pick=(pick_from == "conveyor"))
                break
            except Fatal:
                raise
            except Failure as exc:
                if attempt == PICK_ATTEMPTS or b is None:
                    raise Failure(f"pick failed after {attempt} attempt(s): {exc}")
                print(f">>> [복구 {attempt}/{PICK_ATTEMPTS - 1}] 집기 실패: {exc} -> 진공 해제·위로 후퇴·대기 자세 복귀 후 "
                      f"박스를 다시 인식해서 재시도", flush=True)
                self.recoveries.append(f"pick retry {attempt}: {exc}")
                self.back_off(b, yaw_of(q), b[2] + size[2] / 2)
        if pick_from == "conveyor":
            # Marker for the cycle runner: PICK is empty now, the next box may be conveyed in.
            print(">>> [MoveIt 5/7] 박스 들어올림 확인: PICK 비움 (다음 박스 투입 가능)", flush=True)

        # Target box yaw (AHEAD allows 0 / 90 deg). The box is rigid on the cup, so the TCP
        # turns by the same angle; boxes are symmetric, so take the smaller turn (mod 180 deg).
        place_yaw = yaw if slot_yaw is None else slot_yaw
        place_yaw = yaw + ((place_yaw - yaw + math.pi / 2) % math.pi - math.pi / 2)
        # Carried box bottom must clear, by STACK_CLEAR_M, the boxes under the straight PICK->slot
        # corridor (not the whole pallet: a 1.6 m stack would put the far row out of reach).
        half = math.hypot(size[0], size[1]) / 2 + 0.05
        corridor = [p_[0][2] + psz[n][2] / 2 for n, p_ in before.items()
                    if seg_rect_dist(b[:2], place[:2], p_[0][:2], psz[n], yaw_of(p_[1])) <= half]
        stack_top = max([PALLET_TOP_Z] + corridor)
        transfer_z = max(top + APPROACH_M, stack_top + STACK_CLEAR_M + size[2])
        via_ptp = pick_from == "buffer" or place_on == "buffer"
        if via_ptp:
            # Buffer table: joint-space move (the straight line would pass over the robot base).
            transfer_z = max(top + APPROACH_M, place[2] + size[2] / 2 + APPROACH_M)
        print(f">>> [MoveIt 5/7] 수직으로 들어올린 뒤 수평 유지하며 이송 중 (이송 높이 TCP z={transfer_z:.2f} m, 충돌 검사)...", flush=True)
        if transfer_z > top + APPROACH_M + 1e-3:
            self.line((b[0], b[1], transfer_z), yaw, "lift to transfer height")
        if abs(place_yaw - yaw) > 1e-3 and not via_ptp:
            # Turn the box in place above PICK first: combining a 90 deg turn with the transfer line
            # was only 61 % feasible as a Cartesian path (box_23, 2026-10-09).
            # The box footprint is symmetric, so place_yaw +- 180 deg is the same placement: try the
            # other turn direction too (a -90 deg turn ran j6 past -pi; box_23, 2026-10-09).
            other = place_yaw - math.copysign(math.pi, place_yaw - yaw)
            last = None
            for cand in (place_yaw, other):
                turn = f"turn box {math.degrees(cand - yaw):+.0f} deg above PICK"
                try:
                    # slow: a 90 deg j6 turn at 0.2 lagged past the 0.2 rad path tolerance (box_23)
                    self.line((b[0], b[1], transfer_z), cand, turn, speed=CONTACT_SPEED)
                    place_yaw = cand
                    last = None
                    break
                except Failure as exc:
                    print(f"    ! {turn}: {exc}", flush=True)
                    last = exc
            if last is not None:
                raise last
        if via_ptp:
            # The footprint is symmetric: of place_yaw and place_yaw + 180 deg take the IK goal that
            # turns the wrist least (a 5.1 rad wrist turn left j6 near its limit and the following
            # vertical lowering was only 64 % feasible; box_11 to the buffer, 2026-10-09).
            goals = []
            for cand in (place_yaw, place_yaw + math.pi, place_yaw - math.pi):
                try:
                    g = self.ik((place[0], place[1], transfer_z), cand, "move the box above the target (PTP)")
                except Failure:
                    continue
                goals.append((sum(abs(g[j] - self.joints.get(j, g[j])) for j in ("j4", "j6")), cand, g))
            if not goals:
                raise Failure("move the box above the target (PTP): no IK")
            _, place_yaw, g = min(goals, key=lambda t: t[0])
            self.plan_joints(g, "move the box above the target (PTP)")
        else:
            self.line((place[0], place[1], transfer_z), place_yaw, "level transfer above slot")
        self.check_carried(size, "transfer")

        # ---- place, camera check, re-place if off target ---------------------------------------
        place_top = place[2] + size[2] / 2
        target = place.copy()
        if "place_offset" in faults:
            # Test fault: release 40 mm off the planned slot (e.g. the box slipped on the cup).
            target = place + np.array([0.0, 0.04 if place[1] > PLACE_XY[1] else -0.04, 0.0])
            print("    ! TEST FAULT place_offset: releasing 40 mm off the planned slot", flush=True)
        cam_note = ""
        for rp in range(REPLACE_ATTEMPTS + 1):
            print(">>> [MoveIt 6/7] 목표 위치로 내려놓고 진공 OFF(해제) 중 (주변 박스와 충돌 검사)...", flush=True)
            # Fast to just above the slot, then the last SLOW_LOWER_M slowly: at 0.2 a 13.8 kg box
            # swung into its neighbour (pushed it 6.9 mm, 2026-10-09).
            self.line((target[0], target[1], place_top + PLACE_GAP_M + SLOW_LOWER_M), place_yaw, "lower above slot")
            self.check_carried(size, "lowering")
            self.line((target[0], target[1], place_top + PLACE_GAP_M), place_yaw, "lower into slot (collision-checked)",
                      speed=CONTACT_SPEED)
            self.set_vacuum(False)
            self.wait(lambda: self.suction == "OFF", 5, "suction OFF")
            self.spin_for(1.0)
            # Detaching returns the box to the world at its attached pose; the cup still touches
            # it, so allow cup<->box only for the vertical retreat, other collisions stay checked.
            self.apply_scene(attached=[AttachedCollisionObject(link_name=TCP, object=box_object(BOX_ID, target, size, op=CollisionObject.REMOVE))])
            self.set_allowed_collision(BOX_ID, "suction_cup_link", True)
            try:
                self.line((target[0], target[1], transfer_z), place_yaw, "retreat up (collision-checked)")
            finally:
                self.set_allowed_collision(BOX_ID, "suction_cup_link", False)
            if pose_source != "camera":
                break
            # Gripper camera check of the placed box (RGB: top face on the plane slot top; a tilted
            # or fallen box does not match the SKU footprint there and is not confirmed).
            print(">>> [MoveIt 6/7] 그리퍼 카메라로 놓인 박스 위치 확인 중...", flush=True)
            try:
                # From the retreat pose straight above the slot (no extra move: a PTP down to the
                # inspection height aborted twice on box_01, 2026-10-09).
                c_b, c_q = self.wrist_look(box, size, place, place_yaw, place_top, base_z=place[2] - size[2] / 2,
                                           timeout=10.0, move=False)
            except Failure:
                raise Fatal("placed box not confirmed by the gripper camera (tilted, fallen or moved): "
                            "cell stopped, operator check needed")
            c_err = math.hypot(c_b[0] - place[0], c_b[1] - place[1])
            c_yaw = math.degrees((yaw_of(c_q) - place_yaw + math.pi / 2) % math.pi - math.pi / 2)
            cam_note = f"camera check xy {c_err * 1000:.1f} mm, yaw {c_yaw:+.1f} deg"
            print(f"    [camera] 놓인 박스: 목표 대비 xy {c_err * 1000:.1f} mm, yaw {c_yaw:+.1f} deg", flush=True)
            self.placed_measured = (c_b, yaw_of(c_q))
            if c_err <= PLACE_TOL_XY_M and abs(c_yaw) <= PLACE_YAW_DEG:
                break
            if rp == REPLACE_ATTEMPTS:
                raise Fatal(f"placement still off target after {REPLACE_ATTEMPTS} re-place attempt(s) ({cam_note})")
            print(f">>> [복구] 놓인 위치가 목표에서 {c_err * 1000:.0f} mm 벗어남 -> 팔레트 위에서 다시 집어서 다시 놓기", flush=True)
            self.recoveries.append(f"re-place: camera {c_err * 1000:.0f} mm off")
            c_top = place_top
            try:
                self.grip(np.array([c_b[0], c_b[1], place[2]]), yaw_of(c_q), c_top, size, c_q, at_pick=False)
            except Failure as exc:
                raise Fatal(f"re-grip on the pallet failed: {exc}")
            place_yaw = yaw_of(c_q) + ((place_yaw - yaw_of(c_q) + math.pi / 2) % math.pi - math.pi / 2)
            self.line((c_b[0], c_b[1], transfer_z), yaw_of(c_q), "lift for re-place")
            self.line((place[0], place[1], transfer_z), place_yaw, "move above the planned slot (re-place)")
            target = place.copy()
        pb, pq, _ = self.fresh_box()
        self.apply_scene(objects=[box_object(BOX_ID, target, size, op=CollisionObject.REMOVE)], check=False)
        self.apply_scene(objects=[box_object(f"placed_{box}", pb, size, pq)])

        print(">>> [MoveIt 7/7] PICK 위 대기 자세로 복귀 후 적재 결과·기적재 박스 밀림 확인 중...", flush=True)
        self.go_ready("return to ready pose above PICK")
        # Independent check against Gazebo ground truth (simulation only; the cell itself relies
        # on the gripper camera check above).
        pb, pq, _ = self.fresh_box()
        err_xy = math.hypot(pb[0] - place[0], pb[1] - place[1])
        err_z = pb[2] - place[2]
        tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(pq[0], pq[1]))))
        yaw_err = math.degrees((yaw_of(pq) - place_yaw + math.pi / 2) % math.pi - math.pi / 2)
        snap = gz_world_poses()
        moved = {n: float(np.linalg.norm(gz_model_pose(n, snap)[0] - p_[0])) for n, p_ in before.items()}
        worst = max(moved.values(), default=0.0)
        result = (f"{box} at ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f}) m; xy error {err_xy * 1000:.1f} mm, "
                  f"z error {err_z * 1000:+.1f} mm, yaw error {yaw_err:+.1f} deg, tilt {tilt:.2f} deg; lifted {rise * 1000:.0f} mm; "
                  f"transfer TCP z {transfer_z:.2f} m; placed boxes {len(before)}, max disturbance "
                  f"{worst * 1000:.1f} mm; recoveries {len(self.recoveries)}"
                  + (f" ({'; '.join(self.recoveries)})" if self.recoveries else "")
                  + f"; planners: {', '.join(self.planners_used)}; "
                  f"pick pose: {pose_source}" + (f" ({', '.join(notes)}; {cam_note})" if pose_source == "camera" else ""))
        if err_xy > PLACE_TOL_XY_M or abs(err_z) > PLACE_TOL_Z_M or tilt > PLACE_TILT_DEG or abs(yaw_err) > PLACE_YAW_DEG:
            raise Fatal("placed box outside tolerance (xy 30 mm, z 20 mm, yaw 3 deg, tilt 5 deg): " + result)
        if worst > DISTURB_TOL_M:
            raise Fatal(f"an already placed box moved {worst * 1000:.1f} mm (> {DISTURB_TOL_M * 1000:.0f} mm): " + result)
        return result

    def check_carried(self, size, phase):
        """Box still on the cup (simulation: pose stream as the vacuum sensor). A dropped box is
        not recovered automatically: vacuum off, back to READY, operator."""
        b, _, _ = self.fresh_box()
        tcp_z = self.tcp_z()
        if tcp_z is not None and abs((tcp_z - size[2] / 2) - b[2]) > 0.05:
            self.set_vacuum(False)
            try:
                self.go_ready("back to ready pose after drop")
            except Failure:
                pass
            raise Fatal(f"box dropped during {phase} (box z {b[2]:.2f} m, expected {tcp_z - size[2] / 2:.2f} m): "
                        "cell stopped, operator needed")

    def tcp_z(self):
        try:
            tf = self.tf_buffer.lookup_transform(FRAME, TCP, rclpy.time.Time())
            return tf.transform.translation.z
        except Exception:
            return None


def _raise(signum, frame):
    raise KeyboardInterrupt


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--box", default=BOX, help="Gazebo box model name (box_NN for stacking)")
    ap.add_argument("--slot", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="target box center in world frame [m] (default: pallet center, layer 1)")
    ap.add_argument("--placed", nargs="*", default=[], help="names of boxes already on the pallet")
    ap.add_argument("--size", type=float, nargs=3, default=list(BOX_SIZE), metavar=("X", "Y", "Z"),
                    help="box size [m] (generator SKU)")
    ap.add_argument("--placed-state", help="planner state JSON (placed box ids and sizes)")
    ap.add_argument("--slot-yaw", type=float, default=None,
                    help="target box yaw [rad] (default: keep the picked yaw)")
    ap.add_argument("--result-json", help="write the placed pose measured by the gripper camera here")
    ap.add_argument("--pick-from", choices=("conveyor", "buffer"), default="conveyor")
    ap.add_argument("--pick-pose", type=float, nargs=4, metavar=("X", "Y", "Z", "YAW"),
                    help="box centre / yaw on the buffer table (--pick-from buffer)")
    ap.add_argument("--place-on", choices=("pallet", "buffer"), default="pallet")
    ap.add_argument("--mass", type=float, help="measured box mass [kg] (gripper payload check)")
    ap.add_argument("--gripper", choices=tuple(GRIPPERS), default=os.environ.get("PAC_GRIPPER", "pad"))
    ap.add_argument("--extra-placed-json", help="JSON {name: [sx, sy, sz]} of other boxes to keep clear "
                    "(boxes on the buffer table)")
    ap.add_argument("--pose-source", choices=("gazebo", "camera"), default="gazebo",
                    help="pick pose from Gazebo ground truth or the top-view CCTV perception")
    args = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    placed, placed_sizes = list(args.placed), {}
    if args.placed_state and os.path.exists(args.placed_state):   # absent before the first commit
        for pb in json.load(open(args.placed_state)).get("placed", []):
            placed_sizes[pb["box_id"]] = (pb["size"]["x"], pb["size"]["y"], pb["size"]["z"])
            if pb["box_id"] not in placed:
                placed.append(pb["box_id"])
    if args.extra_placed_json and os.path.exists(args.extra_placed_json):
        for name, sz in json.load(open(args.extra_placed_json)).items():
            if name != args.box and name not in placed:
                placed.append(name)
                placed_sizes[name] = tuple(sz)
    node = MoveItPickPlace(args.box)
    try:
        result = node.run(args.box, args.slot, placed, args.pose_source, args.slot_yaw, args.size, placed_sizes,
                          args.pick_from, args.pick_pose, args.place_on, args.mass, args.gripper)
        if args.result_json and node.placed_measured is not None:
            c_b, c_yaw = node.placed_measured
            json.dump({"box": args.box, "x": float(c_b[0]), "y": float(c_b[1]), "z": float(c_b[2]), "yaw": float(c_yaw),
                       "source": "gripper camera (RGB, top face on the slot plane)",
                       "recoveries": node.recoveries}, open(args.result_json, "w"))
        print(f"\nMOVEIT PICK&PLACE PASS: {result}", flush=True)
        return 0
    except Failure as e:
        print(f">>> [중단] MoveIt 작업 실패, 정지\n\nMOVEIT PICK&PLACE FAIL: {e}", flush=True)
        return 1
    except KeyboardInterrupt:
        if node.active is not None:
            rclpy.spin_until_future_complete(node, node.active.cancel_goal_async(), timeout_sec=2)
        print("\nStopped by user; MoveIt motion cancelled (suction state unchanged).", flush=True)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
