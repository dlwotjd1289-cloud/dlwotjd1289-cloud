#!/usr/bin/env python3
"""V4.4 MoveIt2 pick & place: weighed box at PICK -> pallet, with the suction cup.

Requires: hdr50_workcell_v4_4_pick.launch.py, hdr50_moveit_v44.launch.py,
run_suction_gripper.sh, and a bridge for /model/v43_scale_box_5kg/pose.
Planning: OMPL (collision-aware, with the workcell planning scene) for free moves
without payload; move_group Cartesian paths (collision-checked) for approach,
level transfer with the box, and retreat.
Box pose is Gazebo ground truth (commissioning), not Top-view perception.
"""
from __future__ import annotations

import math
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from shape_msgs.msg import SolidPrimitive
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
BREAKAWAY_M = 0.01                # unchecked move away from the stopper contact (-x)
PLACE_GAP_M = 0.005               # box bottom above pallet when released
VEL_SCALE = 0.2                   # joint_limits.yaml: keep <= 0.2
PLACE_TOL_XY_M, PLACE_TOL_Z_M, PLACE_TILT_DEG = 0.03, 0.02, 5.0

# Workcell planning scene (world frame), from ahead_workcell_v4_2_physical_scale.sdf
# collision geometry; boxes as (name, center xyz, size xyz).
SCENE = [
    ("floor", (-1.0, -0.5, -0.05), (9.0, 7.0, 0.1)),
    ("pedestal", (0.0, 0.0, 0.195), (0.90, 0.90, 0.39)),           # top kept 1 cm below base_link
    ("conveyor", (-2.95, 1.20, 0.445), (4.18, 0.80, 0.89)),        # supports + rollers, top 0.89
    ("pick_stopper", (-0.82, 1.20, 0.96), (0.05, 0.66, 0.18)),
    ("pallet", (0.0, 1.20, 0.075), (1.10, 1.10, 0.15)),
    ("buffer_rack", (1.45, -0.55, 0.95), (0.92, 0.58, 1.90)),
    ("camera_pole", (-0.60, 1.95, 1.17), (0.24, 0.24, 2.34)),
]


class Failure(RuntimeError):
    pass


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
    def __init__(self):
        super().__init__("pac_moveit_pick_place_v44",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.box = None
        self.suction = None
        self.create_subscription(PoseStamped, f"/model/{BOX}/pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(String, "/pac/suction/state", lambda m: setattr(self, "suction", m.data), 10)
        self.vacuum_pub = self.create_publisher(Bool, "/pac/suction/vacuum", 10)
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.active = None

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

    def call(self, client, req, timeout=10.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise Failure(f"service {client.srv_name} did not answer")
        return fut.result()

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
        self.spin_for(0.5)  # let position-controlled joints settle

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

    def set_vacuum(self, on):
        for _ in range(3):
            self.vacuum_pub.publish(Bool(data=on))
            self.spin_for(0.05)

    # ---- motion ------------------------------------------------------------
    def plan_pose(self, xyz, yaw, what):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = TCP
        bv = BoundingVolume()
        bv.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
        p.orientation.w = 1.0
        bv.primitive_poses = [p]
        pc.constraint_region = bv
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = TCP
        oc.orientation = tool_down_quat(yaw)
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        # Keep the wrist away from its +-2*pi limits: a goal found at j4 = -6.28 rad left
        # no room for the following Cartesian descent (stopped at 95-98 %).
        c.joint_constraints = [JointConstraint(joint_name=j, position=0.0, tolerance_above=math.pi,
                                               tolerance_below=math.pi, weight=1.0) for j in ("j4", "j6")]
        self.plan_and_execute(c, what)

    def plan_joints(self, joints, what):
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=v, tolerance_above=0.005,
                                               tolerance_below=0.005, weight=1.0) for n, v in joints.items()]
        self.plan_and_execute(c, what)

    def plan_and_execute(self, constraints, what):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = GROUP
        r.pipeline_id = "ompl"
        r.num_planning_attempts = 5
        r.allowed_planning_time = 10.0
        # Gazebo's position-controlled joints lag on long, fast OMPL moves (large j6 turns
        # aborted with PATH_TOLERANCE_VIOLATED at 0.2): free moves run at half speed.
        r.max_velocity_scaling_factor = VEL_SCALE * 0.5
        r.max_acceleration_scaling_factor = VEL_SCALE * 0.5
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        g.planning_options.plan_only = False
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.run_action(self.move_ac, g, 120.0, what)

    def line(self, xyz, yaw, what, avoid_collisions=True):
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
        req.max_velocity_scaling_factor = VEL_SCALE * 0.5
        req.max_acceleration_scaling_factor = VEL_SCALE * 0.5
        res = self.call(self.cart_cli, req)
        if res.fraction < 0.999:
            raise Failure(f"{what}: Cartesian path only {res.fraction * 100:.0f}% feasible")
        g = ExecuteTrajectory.Goal(trajectory=res.solution)
        self.run_action(self.exec_ac, g, 60.0, what)

    # ---- cycle -------------------------------------------------------------
    def run(self):
        print(">>> [MoveIt 1/7] MoveIt·흡착 그리퍼·박스 상태 확인 중...", flush=True)
        for ac, n in ((self.move_ac, "/move_action"), (self.exec_ac, "/execute_trajectory")):
            if not ac.wait_for_server(timeout_sec=15.0):
                raise Failure(f"{n} not available (start hdr50_moveit_v44.launch.py)")
        for cli in (self.scene_cli, self.cart_cli):
            if not cli.wait_for_service(timeout_sec=15.0):
                raise Failure(f"{cli.srv_name} not available")
        self.wait(lambda: self.box is not None and self.suction is not None, 10,
                  "box pose and /pac/suction/state (is run_suction_gripper.sh running?)")
        if self.suction != "OFF":
            raise Failure(f"suction must start OFF (is {self.suction})")
        b, q, _ = self.fresh_box()
        if not (b[0] >= -1.10 and abs(b[1] - 1.20) < 0.05):
            raise Failure(f"box is not at PICK (x={b[0]:.3f}, y={b[1]:.3f}); run V4.3 first")
        yaw = yaw_of(q)
        top = b[2] + BOX_SIZE[2] / 2
        print(f"    box at ({b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}) m, yaw {math.degrees(yaw):.1f} deg", flush=True)

        print(">>> [MoveIt 2/7] 작업셀 장애물·박스를 planning scene에 등록 중...", flush=True)
        # Clear a box left from a previous run (absent objects are not an error here).
        self.apply_scene(attached=[AttachedCollisionObject(link_name=TCP, object=box_object(BOX_ID, b, BOX_SIZE, op=CollisionObject.REMOVE))], check=False)
        self.apply_scene(objects=[box_object(BOX_ID, b, BOX_SIZE, op=CollisionObject.REMOVE)], check=False)
        self.apply_scene(objects=[box_object(n, c, s) for n, c, s in SCENE] + [box_object(BOX_ID, b, BOX_SIZE, q)])

        print(">>> [MoveIt 3/7] 박스 위 접근 위치로 경로 계획·이동 중 (OMPL, 충돌 회피)...", flush=True)
        self.plan_pose((b[0], b[1], top + APPROACH_M), yaw, "move above box")
        self.line((b[0], b[1], top + CHECKED_GAP_M), yaw, "descend to box (collision-checked)")
        self.line((b[0], b[1], top - PRESS_M), yaw, "press cup on box", avoid_collisions=False)

        print(">>> [MoveIt 4/7] 진공 ON, 흡착 확인 후 박스를 로봇에 부착(attach) 중...", flush=True)
        self.set_vacuum(True)
        self.wait(lambda: self.suction == "GRIPPED", 5, "suction GRIPPED")
        aco = AttachedCollisionObject(link_name=TCP, touch_links=TOUCH_LINKS)
        aco.object.id = BOX_ID
        aco.object.header.frame_id = FRAME
        aco.object.operation = CollisionObject.ADD
        self.apply_scene(attached=[aco])

        print(">>> [MoveIt 5/7] 박스 들어올린 뒤 수평 유지하며 팔레트 위로 이송 중 (충돌 검사)...", flush=True)
        z_before = self.fresh_box()[0][2]
        # The box rests against the pick stopper (contact depth ~0), so the start state is
        # "in collision": first move it a little away from the stopper (-x) and off the
        # rollers (unchecked, tiny), then lift with collision checking.
        self.line((b[0] - BREAKAWAY_M, b[1], top - PRESS_M + 0.005), yaw,
                  "break away from stopper", avoid_collisions=False)
        self.line((b[0], b[1], top + APPROACH_M), yaw, "lift box")
        rise = self.fresh_box()[0][2] - z_before
        if rise < APPROACH_M - 0.03:
            raise Failure(f"box did not follow the suction cup (rise {rise:.3f} m)")
        place_center_z = PALLET_TOP_Z + BOX_SIZE[2] / 2
        place_top = place_center_z + BOX_SIZE[2] / 2
        # Transfer as a level, collision-checked Cartesian move at lift height (box bottom
        # above the stopper top): an OMPL transfer once swung the wrist and tilted the 5 kg
        # box ~15 deg, and Gazebo's position controller aborted (path tolerance 0.2 rad).
        lift_z = top + APPROACH_M
        self.line((PLACE_XY[0], PLACE_XY[1], lift_z), yaw, "transfer to pallet (level)")
        self.line((PLACE_XY[0], PLACE_XY[1], place_top + APPROACH_M), yaw, "lower above pallet")

        print(">>> [MoveIt 6/7] 팔레트에 내려놓고 진공 OFF(해제) 중...", flush=True)
        self.line((PLACE_XY[0], PLACE_XY[1], place_top + PLACE_GAP_M), yaw, "descend to pallet (collision-checked)")
        self.set_vacuum(False)
        self.wait(lambda: self.suction == "OFF", 5, "suction OFF")
        self.spin_for(1.0)
        # Released box leaves the robot; re-add it later at its measured pose.
        self.apply_scene(attached=[AttachedCollisionObject(link_name=TCP, object=box_object(BOX_ID, b, BOX_SIZE, op=CollisionObject.REMOVE))])
        self.apply_scene(objects=[box_object(BOX_ID, b, BOX_SIZE, op=CollisionObject.REMOVE)])
        self.line((PLACE_XY[0], PLACE_XY[1], place_top + APPROACH_M), yaw, "retreat from box", avoid_collisions=False)
        pb, pq, _ = self.fresh_box()
        self.apply_scene(objects=[box_object(BOX_ID, pb, BOX_SIZE, pq)])

        print(">>> [MoveIt 7/7] 홈 자세로 복귀 후 적재 결과 확인 중...", flush=True)
        self.plan_joints(HOME, "return home")
        pb, pq, _ = self.fresh_box()
        err_xy = math.hypot(pb[0] - PLACE_XY[0], pb[1] - PLACE_XY[1])
        err_z = pb[2] - place_center_z
        tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(pq[0], pq[1]))))
        result = (f"box on pallet at ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f}) m; xy error {err_xy * 1000:.1f} mm, "
                  f"z error {err_z * 1000:+.1f} mm, tilt {tilt:.2f} deg; lifted {rise * 1000:.0f} mm")
        if err_xy > PLACE_TOL_XY_M or abs(err_z) > PLACE_TOL_Z_M or tilt > PLACE_TILT_DEG:
            raise Failure("placed box outside tolerance (xy 30 mm, z 20 mm, tilt 5 deg): " + result)
        return result


def _raise(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    node = MoveItPickPlace()
    try:
        print(f"\nMOVEIT PICK&PLACE PASS: {node.run()}", flush=True)
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
