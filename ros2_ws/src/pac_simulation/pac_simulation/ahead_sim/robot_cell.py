"""HDR50-22 suction robot cell inside the AHEAD live PyBullet world (V4.4 extension).

Adds to the original pallet simulator (backed up in ~/AHEAD/backup_ahead_live_sim_original_20261009):
  - the HDR50-22 robot (same URDF as Gazebo/MoveIt: pedestal z, suction cup, suction_tcp),
  - the conveyor end / PICK station with the pick stopper,
  - a placement executor: a box spec (same JSON as /api/place) is delivered to PICK, picked with
    the suction cup, carried above the stack and *released* at the target; Bullet decides how it
    settles. Nothing is teleported onto the pallet.

Frame: simulator frame = Gazebo workcell world - (0, 1.20, 0.15): pallet centre at the origin,
deck top z = 0 (as before). Robot base_link (Gazebo (0, 0, 0.40)) -> (0, -1.20, 0.25).
Joints use POSITION_CONTROL along interpolated targets (the robot is a dynamic body); IK/FK run
in a separate DIRECT client so planning never disturbs the live physics.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import pybullet as p

from .models import BoxSpec

GZ_TO_SIM = (0.0, -1.20, -0.15)
ROBOT_URDF = Path(__file__).resolve().parents[5] / "models" / "pybullet" / "hdr50_22_suction.urdf"
JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")
VEL_LIMIT = (3.054, 3.054, 3.054, 4.363, 4.363, 6.109)
HOME = (0.0, 1.5707, 0.0, 0.0, 0.0, 0.0)

# Workcell geometry (Gazebo V4.4 world, converted to the simulator frame).
ROLLER_TOP_Z = 0.895 + GZ_TO_SIM[2]                 # 0.745
STOPPER_FACE_X = -0.845
INFEED = {"x": (-1.90, -0.85), "y": (-0.34, 0.34), "top": ROLLER_TOP_Z}
STOPPER = {"center": (-0.82, 0.0, 0.96 + GZ_TO_SIM[2]), "size": (0.05, 0.66, 0.18)}
PEDESTAL = {"center": (0.0, -1.20, (0.40 + GZ_TO_SIM[2] + GZ_TO_SIM[2] - 0.02) / 2), "size": (0.90, 0.90, 0.40 + 0.02)}
READY_TCP = (-0.95, 0.95 + GZ_TO_SIM[1], 1.55 + GZ_TO_SIM[2])
# Fixed pole CCTV (camera_1_cctv_base in the Gazebo world): position, rpy, horizontal FOV, aspect.
CCTV = {"pos": (-0.6, 1.78 + GZ_TO_SIM[1], 2.4 + GZ_TO_SIM[2]), "rpy": (0.0, 1.178996, -1.317721),
        "hfov": 1.40, "aspect": 1280 / 720, "pole_xy": (-0.6, 1.95 + GZ_TO_SIM[1])}

SPEED = 0.25          # fraction of joint velocity limits (free moves)
CART_SPEED_M_S = 0.35  # straight lines with the box
SLOW_M_S = 0.08        # contact / last part of lowering
APPROACH_M = 0.20
STACK_CLEAR_M = 0.10
PLACE_GAP_M = 0.005
SLOW_LOWER_M = 0.06
SETTLE_S = 1.0


def tool_down_quat(yaw: float) -> Tuple[float, float, float, float]:
    """TCP +x pointing down, TCP +y along world yaw (same convention as the MoveIt executor)."""
    x = (0.0, 0.0, -1.0)
    y = (math.cos(yaw), math.sin(yaw), 0.0)
    z = (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0])
    R = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j][i] + R[i][j]) / s
    q[k] = (R[k][i] + R[i][k]) / s
    return (q[0], q[1], q[2], (R[k][j] - R[j][k]) / s)


def parse_visuals(urdf_path: Path) -> Dict[str, List[Dict[str, Any]]]:
    """URDF link -> visuals (mesh path or primitive, origin, colour) for the web viewer."""
    root = ET.parse(urdf_path).getroot()
    materials = {m.get("name"): m.find("color").get("rgba") for m in root.findall("material") if m.find("color") is not None}
    out: Dict[str, List[Dict[str, Any]]] = {}
    for link in root.findall("link"):
        items = []
        for v in link.findall("visual"):
            o = v.find("origin")
            xyz = [float(a) for a in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(a) for a in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            g = v.find("geometry")[0]
            m = v.find("material")
            rgba = None
            if m is not None:
                c = m.find("color")
                rgba = c.get("rgba") if c is not None else materials.get(m.get("name"))
            item = {"xyz": xyz, "quat": list(p.getQuaternionFromEuler(rpy)),
                    "rgba": [float(a) for a in rgba.split()] if rgba else [0.85, 0.85, 0.82, 1.0]}
            if g.tag == "mesh":
                item.update({"type": "mesh", "file": Path(g.get("filename")).name,
                             "path": g.get("filename"), "scale": [float(a) for a in g.get("scale", "1 1 1").split()]})
            elif g.tag == "cylinder":
                item.update({"type": "cylinder", "radius": float(g.get("radius")), "length": float(g.get("length"))})
            elif g.tag == "box":
                item.update({"type": "box", "size": [float(a) for a in g.get("size").split()]})
            else:
                continue
            items.append(item)
        if items:
            out[link.get("name")] = items
    return out


@dataclass
class Segment:
    name: str
    points: List[Tuple[float, ...]]        # joint targets
    duration_s: float
    on_end: Optional[str] = None           # "attach" / "release"


@dataclass
class PlaceTask:
    spec: BoxSpec
    phase: str = "QUEUED"
    segments: Deque[Segment] = field(default_factory=deque)
    message: str = ""


class RobotCell:
    def __init__(self, world) -> None:
        self.world = world
        self.cid = world.client_id
        self.ik_cid = p.connect(p.DIRECT)
        self.visuals = parse_visuals(ROBOT_URDF)
        self.queue: Deque[PlaceTask] = deque()
        self.task: Optional[PlaceTask] = None
        self.log: Deque[str] = deque(maxlen=12)
        self.in_transit: set = set()
        self.completed: List[Dict[str, Any]] = []
        self.robot = None
        self.ik_robot = p.loadURDF(str(ROBOT_URDF), basePosition=GZ_TO_SIM, useFixedBase=True,
                                   physicsClientId=self.ik_cid)
        self.ik_joints = self._joint_indices(self.ik_robot, self.ik_cid)
        self.reload()

    # ---------------------------------------------------------------- setup
    def _joint_indices(self, body, cid):
        idx, links = {}, {}
        for i in range(p.getNumJoints(body, physicsClientId=cid)):
            info = p.getJointInfo(body, i, physicsClientId=cid)
            idx[info[1].decode()] = i
            links[info[12].decode()] = i
        self.link_index = links
        return [idx[j] for j in JOINTS]

    def reload(self) -> None:
        """(Re)create robot + workcell after BulletPalletWorld.reset() wiped the simulation."""
        cid = self.cid
        self.robot = p.loadURDF(str(ROBOT_URDF), basePosition=GZ_TO_SIM, useFixedBase=True,
                                flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=cid)
        self.joints = self._joint_indices(self.robot, cid)
        self.tcp = self.link_index["suction_tcp"]
        self.cup = self.link_index["suction_cup_link"]
        self.q = list(self.ik(READY_TCP, 0.0, HOME) or HOME)
        for j, q in zip(self.joints, self.q):
            p.resetJointState(self.robot, j, q, physicsClientId=cid)
        self._command(self.q)
        self.static_ids = []
        for name, center, size, rgba in (
            ("INFEED", ((INFEED["x"][0] + INFEED["x"][1]) / 2, 0.0, (INFEED["top"] - 0.17) / 2),
             (INFEED["x"][1] - INFEED["x"][0], INFEED["y"][1] - INFEED["y"][0], INFEED["top"] + 0.17),
             [0.16, 0.18, 0.20, 1]),
            ("PICK_STOPPER", STOPPER["center"], STOPPER["size"], [0.91, 0.56, 0.12, 1]),
            ("ROBOT_PEDESTAL", PEDESTAL["center"], PEDESTAL["size"], [0.36, 0.39, 0.44, 1]),
        ):
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s / 2 for s in size], physicsClientId=cid)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s / 2 for s in size], rgbaColor=rgba,
                                      physicsClientId=cid)
            body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=vis,
                                     basePosition=center, physicsClientId=cid)
            p.changeDynamics(body, -1, lateralFriction=0.9, physicsClientId=cid)
            self.world._body_to_name[body] = name
            self.static_ids.append(body)
        self.world._body_to_name[self.robot] = "ROBOT"
        if self.world.floor_id is not None:   # plain floor in software renders (no checker texture)
            p.changeVisualShape(self.world.floor_id, -1, rgbaColor=[0.55, 0.57, 0.60, 1.0], textureUniqueId=-1,
                                physicsClientId=cid)
        self.constraint = None
        self.held = None
        self.queue.clear()
        self.task = None
        self.in_transit.clear()
        self.completed.clear()
        self.log.append("robot cell ready (HDR50-22 at READY above PICK)")

    def _command(self, q) -> None:
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, targetPositions=list(q),
                                    forces=[2.0e5] * 6, positionGains=[0.6] * 6, physicsClientId=self.cid)

    # ---------------------------------------------------------------- kinematics (shadow client)
    def fk(self, q) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        for j, v in zip(self.ik_joints, q):
            p.resetJointState(self.ik_robot, j, v, physicsClientId=self.ik_cid)
        s = p.getLinkState(self.ik_robot, self.link_index["suction_tcp"], computeForwardKinematics=True,
                           physicsClientId=self.ik_cid)
        return s[4], s[5]

    def ik(self, pos, yaw, seed) -> Optional[Tuple[float, ...]]:
        """Damped least-squares IK on the shadow robot (PyBullet Jacobian). pybullet's
        calculateInverseKinematics did not converge for tool-down targets near PICK."""
        import numpy as np
        orn = tool_down_quat(yaw)
        Rt = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        lo = np.array([-3.142, -1.134, -1.396, -math.pi, -2.182, -math.pi])
        hi = np.array([3.142, 2.967, 3.141, math.pi, 2.182, math.pi])
        q = np.clip(np.array(seed, dtype=float), lo, hi)
        tcp = self.link_index["suction_tcp"]
        for _ in range(200):
            fp, fo = self.fk(q)
            R = np.array(p.getMatrixFromQuaternion(fo)).reshape(3, 3)
            ep = np.array(pos) - np.array(fp)
            E = Rt @ R.T
            ang = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1) / 2)))
            v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
            eo = 0.5 * v if ang < 1e-6 else v * (ang / (2 * math.sin(ang))) if ang < math.pi - 1e-3 else v
            if np.linalg.norm(ep) < 2e-4 and ang < 2e-3:
                return tuple(float(x) for x in q)
            jl, ja = p.calculateJacobian(self.ik_robot, tcp, [0, 0, 0], list(q), [0.0] * 6, [0.0] * 6,
                                         physicsClientId=self.ik_cid)
            J = np.vstack([np.array(jl), np.array(ja)])
            e = np.concatenate([ep, eo])
            dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(6), e)
            q = np.clip(q + dq, lo, hi)
        return None

    def line(self, q0, p1, yaw, speed_m_s, name, on_end=None) -> Segment:
        p0, o0 = self.fk(q0)
        R0 = p.getMatrixFromQuaternion(o0)
        yaw0 = math.atan2(R0[4], R0[1])               # TCP +y axis (tool down) in the xy plane
        dyaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi
        dist = math.dist(p0, p1)
        # enough steps for the translation (1 cm) AND the rotation (3 deg): a pure 90 deg turn
        # in 2 steps jumped j6 by 0.8 rad and was rejected.
        n = max(2, int(dist / 0.01) + 1, int(abs(dyaw) / math.radians(3)) + 1)
        pts, q = [], tuple(q0)
        for k in range(1, n + 1):
            t = k / n
            target = tuple(a + (b - a) * t for a, b in zip(p0, p1))
            sol = self.ik(target, yaw0 + dyaw * t, q)
            if sol is None or max(abs(a - b) for a, b in zip(sol, q)) > 0.3:
                raise ValueError(f"{name}: no continuous IK at {tuple(round(v, 3) for v in target)}")
            pts.append(sol)
            q = sol
        return Segment(name, pts, max(0.4, dist / speed_m_s, self._joint_time(q0, pts)), on_end)

    def joint_move(self, q0, q1, name) -> Segment:
        n = 30
        pts = [tuple(a + (b - a) * k / n for a, b in zip(q0, q1)) for k in range(1, n + 1)]
        return Segment(name, pts, max(0.6, self._joint_time(q0, pts)))

    @staticmethod
    def _joint_time(q0, pts) -> float:
        prev, total = q0, 0.0
        for q in pts:
            total += max(abs(a - b) / (v * SPEED) for a, b, v in zip(q, prev, VEL_LIMIT))
            prev = q
        return total

    # ---------------------------------------------------------------- tasks
    def request_place(self, spec: BoxSpec) -> Dict[str, Any]:
        if spec.box_id in self.world.boxes or any(t.spec.box_id == spec.box_id for t in self.queue):
            raise ValueError(f"duplicate box id: {spec.box_id}")
        self.queue.append(PlaceTask(spec))
        return {"queued": spec.box_id, "queue_length": len(self.queue) + (1 if self.task else 0)}

    def stack_top(self) -> float:
        tops = [0.0]
        for bid, rec in self.world.boxes.items():
            if bid in self.in_transit:
                continue
            lo, hi = p.getAABB(rec.body_id, physicsClientId=self.cid)
            if abs(hi[0]) < 0.8 and abs(hi[1]) < 0.8:
                tops.append(hi[2])
        return max(tops)

    def _start(self, task: PlaceTask) -> None:
        spec = task.spec
        sx, sy, sz = spec.size_m
        pick = (STOPPER_FACE_X - sx / 2, 0.0, INFEED["top"] + sz / 2 + 0.002)
        body = self.world.add_box_at(spec, pick, 0.0)
        p.changeVisualShape(body, -1, rgbaColor=[0.79, 0.55, 0.32, 1.0], physicsClientId=self.cid)  # cardboard
        self.in_transit.add(spec.box_id)
        self.held = (spec.box_id, body)
        top = pick[2] + sz / 2
        tx, ty, tz = spec.target_position_m
        place_top = tz + sz / 2
        transfer_z = max(top + APPROACH_M, self.stack_top() + STACK_CLEAR_M + sz)
        yaw0, yaw1 = 0.0, spec.yaw_rad
        # symmetric box: pick the TCP turn that keeps j6 inside +-pi
        q = tuple(self.q)
        above = self.ik((pick[0], pick[1], top + APPROACH_M), yaw0, q)
        if above is None:
            raise ValueError("PICK approach unreachable")
        segs = [self.joint_move(q, above, "move above PICK")]
        q = above
        for seg in (self.line(q, (pick[0], pick[1], top - 0.003), yaw0, SLOW_M_S, "lower onto box", "attach"),):
            segs.append(seg)
            q = seg.points[-1]
        seg = self.line(q, (pick[0], pick[1], transfer_z), yaw0, CART_SPEED_M_S, "lift")
        segs.append(seg)
        q = seg.points[-1]
        if abs(yaw1 - yaw0) > 1e-3:
            for cand in (yaw1, yaw1 - math.copysign(math.pi, yaw1)):
                try:
                    seg = self.line(q, (pick[0], pick[1], transfer_z), cand, CART_SPEED_M_S, "turn box")
                    yaw1 = cand
                    break
                except ValueError:
                    seg = None
            if seg is None:
                raise ValueError("cannot turn the box at PICK")
            segs.append(seg)
            q = seg.points[-1]
        for name, target, speed, end in (
            ("transfer", (tx, ty, transfer_z), CART_SPEED_M_S, None),
            ("lower above slot", (tx, ty, place_top + PLACE_GAP_M + SLOW_LOWER_M), CART_SPEED_M_S, None),
            ("lower into slot", (tx, ty, place_top + PLACE_GAP_M), SLOW_M_S, "release"),
            ("retreat", (tx, ty, transfer_z), CART_SPEED_M_S, None),
        ):
            seg = self.line(q, target, yaw1, speed, name, end)
            segs.append(seg)
            q = seg.points[-1]
        ready = self.ik(READY_TCP, 0.0, q) or self.q
        segs.append(self.joint_move(q, ready, "back to READY"))
        task.segments = deque(segs)
        task.phase = "RUNNING"
        self._seg, self._t = None, 0.0

    def _attach(self) -> None:
        box_id, body = self.held
        tcp = p.getLinkState(self.robot, self.tcp, computeForwardKinematics=True, physicsClientId=self.cid)
        bpos, born = p.getBasePositionAndOrientation(body, physicsClientId=self.cid)
        inv_p, inv_o = p.invertTransform(tcp[4], tcp[5])
        rel_p, rel_o = p.multiplyTransforms(inv_p, inv_o, bpos, born)
        self.constraint = p.createConstraint(self.robot, self.tcp, body, -1, p.JOINT_FIXED, [0, 0, 0],
                                             rel_p, [0, 0, 0], rel_o, [0, 0, 0, 1], physicsClientId=self.cid)
        p.changeConstraint(self.constraint, maxForce=5.0e4, physicsClientId=self.cid)
        for link in (self.cup, self.tcp, self.link_index.get("wrist_camera_link", self.cup)):
            p.setCollisionFilterPair(self.robot, body, link, -1, 0, physicsClientId=self.cid)
        self.log.append(f"{box_id}: suction attached")

    def _release(self) -> None:
        box_id, body = self.held
        if self.constraint is not None:
            p.removeConstraint(self.constraint, physicsClientId=self.cid)
            self.constraint = None
        self.release_t = self.world.simulation_time_s
        self.log.append(f"{box_id}: released (vacuum off)")

    def update(self, dt: float) -> None:
        """Advance the executor by one physics step (called before stepSimulation)."""
        if self.task is None:
            if not self.queue:
                return
            self.task = self.queue.popleft()
            try:
                self._start(self.task)
                self.log.append(f"{self.task.spec.box_id}: pick & place started")
            except ValueError as exc:
                self.log.append(f"{self.task.spec.box_id}: FAILED to plan ({exc})")
                self._abort()
                return
        task = self.task
        if self._seg is None:
            if not task.segments:
                # settle, then hand the box over to the pallet metrics
                if self.world.simulation_time_s - getattr(self, "release_t", 0.0) >= SETTLE_S:
                    self._finish()
                return
            self._seg, self._t, self._q0 = task.segments.popleft(), 0.0, tuple(self.q)
            task.phase = self._seg.name
        seg = self._seg
        self._t += dt
        f = min(1.0, self._t / seg.duration_s)
        s = f * f * (3 - 2 * f)                      # smooth start/stop
        idx = s * len(seg.points)
        i = min(len(seg.points) - 1, int(idx))
        a = seg.points[i - 1] if i > 0 else self._q0
        b = seg.points[i]
        w = idx - i if i < len(seg.points) else 1.0
        self.q = [x + (y - x) * min(1.0, w) for x, y in zip(a, b)]
        self._command(self.q)
        if f >= 1.0:
            # wait until the joints arrived before contact actions
            actual = [p.getJointState(self.robot, j, physicsClientId=self.cid)[0] for j in self.joints]
            if max(abs(x - y) for x, y in zip(actual, seg.points[-1])) > 0.01 and self._t < seg.duration_s + 2.0:
                return
            if seg.on_end == "attach":
                self._attach()
            elif seg.on_end == "release":
                self._release()
            self._seg = None

    def _finish(self) -> None:
        box_id, body = self.held
        pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=self.cid)
        tgt = self.task.spec.target_position_m
        err = math.dist(pos[:2], tgt[:2])
        rpy = p.getEulerFromQuaternion(quat)
        self.completed.append({"id": box_id, "xy_error_mm": round(err * 1000, 1),
                               "z_error_mm": round((pos[2] - tgt[2]) * 1000, 1),
                               "tilt_deg": round(math.degrees(max(abs(rpy[0]), abs(rpy[1]))), 2)})
        self.log.append(f"{box_id}: placed by robot, xy error {err * 1000:.1f} mm")
        for link in range(-1, p.getNumJoints(self.robot, physicsClientId=self.cid)):
            p.setCollisionFilterPair(self.robot, body, link, -1, 1, physicsClientId=self.cid)
        self.in_transit.discard(box_id)
        self.held = None
        self.task = None

    def _abort(self) -> None:
        if self.held:
            box_id, body = self.held
            if self.constraint is not None:
                p.removeConstraint(self.constraint, physicsClientId=self.cid)
                self.constraint = None
            self.in_transit.discard(box_id)
            if self.task is not None and self.task.phase in ("QUEUED", "RUNNING"):
                # never picked: take it off the PICK station again
                p.removeBody(body, physicsClientId=self.cid)
                self.world.boxes.pop(box_id, None)
                self.world._body_to_name.pop(body, None)
        self.held = None
        self.task = None

    # ---------------------------------------------------------------- viewer
    def snapshot(self) -> Dict[str, Any]:
        links = []
        for name, items in self.visuals.items():
            if name == "world":
                pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cid)
            elif name in self.link_index:
                s = p.getLinkState(self.robot, self.link_index[name], computeForwardKinematics=True,
                                   physicsClientId=self.cid)
                pos, orn = s[4], s[5]
            else:
                continue
            links.append({"name": name, "pos": [round(v, 5) for v in pos], "quat": [round(v, 6) for v in orn]})
        return {
            "links": links,
            "state": self.task.phase if self.task else ("QUEUED" if self.queue else "READY"),
            "current_box": self.task.spec.box_id if self.task else None,
            "queue": [t.spec.box_id for t in self.queue],
            "in_transit": sorted(self.in_transit),
            "completed": self.completed[-30:],
            "log": list(self.log),
        }

    def static_scene(self) -> Dict[str, Any]:
        return {"visuals": {k: [{kk: vv for kk, vv in v.items() if kk != "path"} for v in items]
                            for k, items in self.visuals.items()},
                "infeed": INFEED, "stopper": STOPPER, "pedestal": PEDESTAL, "ready_tcp": READY_TCP,
                "cctv": {**CCTV, "quat": list(p.getQuaternionFromEuler(CCTV["rpy"]))}}

    def mesh_path(self, file_name: str) -> Optional[str]:
        for items in self.visuals.values():
            for v in items:
                if v.get("type") == "mesh" and v["file"] == file_name:
                    return v["path"]
        return None
