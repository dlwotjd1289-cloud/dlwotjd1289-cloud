"""Vertical TCP descent guard and evidence, for the V4.6 Gazebo executor.

This is a simulation commissioning guard, not a certified safety controller.
No gain/tolerance changes, no collision exceptions, no automatic motion retries.
Kinematics comes from move_group's active URDF and is cross-checked with its FK.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

import numpy as np

BOX_FACE_MARGIN_M = 0.002  # each object: +4 mm total x and y, unchanged z
PLAN_EDGE_M = 0.0005
ALIGN_EDGE_M = 0.0005
EXEC_EDGE_M = 0.0015  # +0.5 mm initial alignment budget <= 2 mm moving-box face margin
DESCENT_SPEED = 0.05


def rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_from_matrix(r):
    # Eigenvector form avoids the trace=-1 singularity of tool-down rotations.
    xx, xy, xz = r[0]
    yx, yy, yz = r[1]
    zx, zy, zz = r[2]
    k = np.array([[xx-yy-zz, xy+yx, xz+zx, zy-yz],
                  [xy+yx, yy-xx-zz, yz+zy, xz-zx],
                  [xz+zx, yz+zy, zz-xx-yy, yx-xy],
                  [zy-yz, xz-zx, yx-xy, xx+yy+zz]]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


class URDFKinematics:
    """Active fixed/revolute/prismatic chain; reject unknown/mimic joints."""
    def __init__(self, xml, frame="world", tip="suction_tcp"):
        joints = ET.fromstring(xml).findall("joint")
        by_child = {j.find("child").get("link"): j for j in joints}
        chain, seen = [], set()
        while tip != frame:
            if tip in seen or tip not in by_child:
                raise ValueError(f"URDF has no unambiguous chain {frame} -> {tip}")
            seen.add(tip)
            j = by_child[tip]
            typ = j.get("type")
            if typ not in ("fixed", "revolute", "continuous", "prismatic") or j.find("mimic") is not None:
                raise ValueError(f"unsupported joint {j.get('name')}: {typ}")
            o = j.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            r, p, y = rpy
            t = np.eye(4)
            t[:3, :3] = rotation((0, 0, 1), y) @ rotation((0, 1, 0), p) @ rotation((1, 0, 0), r)
            t[:3, 3] = xyz
            ax = j.find("axis")
            axis = np.array([float(x) for x in (ax.get("xyz", "1 0 0") if ax is not None else "1 0 0").split()])
            if np.linalg.norm(axis) < 1e-8:
                raise ValueError("zero joint axis")
            chain.append((j.get("name"), typ, t, axis / np.linalg.norm(axis)))
            tip = j.find("parent").get("link")
        self.chain = list(reversed(chain))
        self.names = [n for n, typ, _, _ in self.chain if typ != "fixed"]

    def fk(self, positions):
        out = np.eye(4)
        for name, typ, origin, axis in self.chain:
            out = out @ origin
            if typ == "fixed":
                continue
            val = float(positions[name])
            if not math.isfinite(val):
                raise ValueError(f"nonfinite joint {name}")
            joint = np.eye(4)
            if typ == "prismatic":
                joint[:3, 3] = axis * val
            else:
                joint[:3, :3] = rotation(axis, val)
            out = out @ joint
        return out


def error_metrics(pose, reference, radius):
    xy = float(np.linalg.norm(pose[:2, 3] - reference[:2, 3]))
    angle = math.acos(float(np.clip((np.trace(reference[:3, :3].T @ pose[:3, :3]) - 1) / 2, -1, 1)))
    tilt = math.acos(float(np.clip(-pose[2, 0], -1, 1)))  # TCP +x points down
    return {"xy_m": xy, "orientation_deg": math.degrees(angle),
            "tool_tilt_deg": math.degrees(tilt),
            "edge_bound_m": xy + 2 * radius * math.sin(angle / 2)}


def classify(plan_edge, samples, complete):
    if plan_edge > PLAN_EDGE_M:
        return "PLAN_GEOMETRY: MoveIt returned a nonvertical path; execution blocked"
    if not complete or len(samples) < 2:
        return "INCONCLUSIVE: insufficient or interrupted controller evidence"
    if max(s["desired"]["edge_bound_m"] for s in samples) > EXEC_EDGE_M:
        return "CONTROLLER_REFERENCE: desired path departs from audited MoveIt waypoints"
    if max(s["actual"]["edge_bound_m"] for s in samples) > EXEC_EDGE_M:
        return "EXECUTION_TRACKING: desired is vertical, actual joints deviate (control/physics side)"
    return "WITHIN_LIMITS: sampled TCP path within limits; box slip/physics needs box-pose evidence"


class VerticalDescent:
    def __init__(self, node, failure_type):
        from control_msgs.msg import JointTrajectoryControllerState
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.Failure = node, failure_type
        self.latest = None
        self.model = None
        self.active_trace = None
        self.failure = None
        self.radius = 0.5
        self.reference = None
        self.previous_stamp = None
        self.watchdog = node.create_timer(0.1, self.check_stream)
        self.subs = [node.create_subscription(JointTrajectoryControllerState, topic,
                    self.on_state, qos_profile_sensor_data) for topic in
                    ("/joint_trajectory_controller/controller_state", "/joint_trajectory_controller/state")]

    def check_stream(self):
        if self.active_trace is not None and (self.latest is None or time.monotonic()-self.latest[0] > 0.3):
            self.active_trace["stream_stale"] = True
            self.stop("controller feedback stale during descent")

    def on_state(self, msg):
        desired = getattr(msg, "reference", None)
        actual = getattr(msg, "feedback", None)
        if desired is None or not len(desired.positions):
            desired = getattr(msg, "desired", None)
        if actual is None or not len(actual.positions):
            actual = getattr(msg, "actual", None)
        if desired is None or actual is None:
            return
        names = list(msg.joint_names)
        if len(desired.positions) != len(names) or len(actual.positions) != len(names):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest = (time.monotonic(), stamp, names, list(desired.positions), list(actual.positions))
        self.record_alignment_feedback()
        if self.active_trace is None or stamp == self.previous_stamp:
            return
        self.previous_stamp = stamp
        try:
            dp = self.model.fk(dict(zip(names, desired.positions)))
            ap = self.model.fk(dict(zip(names, actual.positions)))
            dm = error_metrics(dp, self.reference, self.radius)
            am = error_metrics(ap, self.reference, self.radius)
            sample = {"stamp": stamp, "joint_names": names,
                      "desired_joints": list(desired.positions), "actual_joints": list(actual.positions),
                      "desired_tcp": dp[:3, 3].tolist(), "actual_tcp": ap[:3, 3].tolist(),
                      "desired": dm, "actual": am,
                      "tracking": error_metrics(ap, dp, self.radius)}
            if self.node.box is not None:
                xyz, quat, received = self.node.box
                sample["gazebo_box"] = {"xyz": xyz.tolist(), "quat": list(quat),
                                         "age_wall_s": time.monotonic() - received}
            self.active_trace["samples"].append(sample)
            if am["edge_bound_m"] > EXEC_EDGE_M or dm["edge_bound_m"] > EXEC_EDGE_M:
                self.stop(f"vertical edge deviation: desired {dm['edge_bound_m']*1000:.2f}, "
                          f"actual {am['edge_bound_m']*1000:.2f} mm (limit {EXEC_EDGE_M*1000:.1f} mm)")
        except Exception as exc:
            self.stop(f"vertical telemetry invalid: {exc}")

    def stop(self, reason):
        if self.failure is None:
            self.failure = reason
            print(f"    [vertical] STOP: {reason}", flush=True)
            if self.node.active is not None:
                self.node.active.cancel_goal_async()

    def initialize(self):
        if self.model is not None:
            return
        from rcl_interfaces.srv import GetParameters
        from moveit_msgs.srv import GetPositionFK
        cli = self.node.create_client(GetParameters, "/move_group/get_parameters")
        fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        try:
            for c in (cli, fk_cli):
                if not c.wait_for_service(timeout_sec=5):
                    raise self.Failure(f"vertical guard requires {c.srv_name}")
            values = self.node.call(cli, GetParameters.Request(names=["robot_description"])).values
            xml = values[0].string_value if values else ""
            model = URDFKinematics(xml)
            self.node.wait(lambda: self.latest is not None and time.monotonic()-self.latest[0] < 0.25,
                           5, "fresh trajectory controller state")
            _, _, names, _, actual = self.latest
            current = dict(zip(names, actual))
            local = model.fk(current)
            req = GetPositionFK.Request()
            req.header.frame_id = "world"
            req.fk_link_names = ["suction_tcp"]
            req.robot_state.is_diff = True
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = actual
            res = self.node.call(fk_cli, req)
            if res.error_code.val != 1 or len(res.pose_stamped) != 1:
                raise self.Failure("MoveIt FK cross-check failed")
            p = res.pose_stamped[0].pose
            remote = np.eye(4)
            remote[:3, 3] = (p.position.x, p.position.y, p.position.z)
            remote[:3, :3] = quaternion_matrix((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
            err = error_metrics(local, remote, 1.0)
            if err["edge_bound_m"] > 1e-5 or abs(local[2, 3]-remote[2, 3]) > 1e-5:
                raise self.Failure("active URDF and MoveIt FK disagree; descent blocked")
            self.model = model
        except (ValueError, KeyError, ET.ParseError) as exc:
            raise self.Failure(f"vertical model unavailable: {exc}") from exc
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_client(fk_cli)

    def current(self):
        self.node.spin_for(0.05)
        if self.latest is None or time.monotonic() - self.latest[0] > 0.25:
            raise self.Failure("vertical controller state stale; descent blocked")
        _, _, names, _, actual = self.latest
        return self.model.fk(dict(zip(names, actual)))

    def prepare(self, xy, yaw, size, what):
        try:
            return self._prepare(xy, yaw, size, what)
        except RuntimeError as exc:
            # Executor's Fatal bypasses generic pickup recovery/back-off.
            raise self.Failure(str(exc)) from exc

    def alignment_sample(self):
        """Read only the already received controller sample; no spin or motion."""
        if self.latest is None:
            return None
        received, stamp, names, desired, actual = self.latest
        dp = self.model.fk(dict(zip(names, desired)))
        ap = self.model.fk(dict(zip(names, actual)))
        target = self.alignment_target
        def metrics(pose, ref):
            m = error_metrics(pose, ref, self.radius)
            m['orientation_edge_m'] = m['edge_bound_m'] - m['xy_m']
            m['delta_xyz_m'] = (pose[:3, 3] - ref[:3, 3]).tolist()
            return m
        return {'stamp': stamp, 'age_wall_s': time.monotonic() - received, 'target_tcp': target.tolist(),
                'joint_names': list(names), 'desired_joints': list(desired), 'actual_joints': list(actual),
                'desired_tcp': dp.tolist(), 'actual_tcp': ap.tolist(),
                'desired_to_target': metrics(dp, target), 'actual_to_target': metrics(ap, target),
                'actual_to_desired': metrics(ap, dp)}

    def record_alignment_feedback(self):
        trace = getattr(self, 'alignment_trace', None)
        if trace is None or self.latest[1] == getattr(self, 'alignment_stamp', None):
            return
        self.alignment_stamp = self.latest[1]
        try:
            sample = self.alignment_sample()
            sample['attempt'] = trace['current_attempt']
            if len(trace['samples']) < 5000:
                trace['samples'].append(sample)
            else:
                trace['dropped_samples'] += 1
        except Exception as exc:
            trace['recording_errors'].append(str(exc))

    def record_alignment_plan(self, trajectory, what):
        """Store the returned plan before execution. This does not accept/reject it."""
        trace = getattr(self, 'alignment_trace', None)
        if trace is None:
            return
        try:
            jt = trajectory.joint_trajectory
            points = []
            for pt in jt.points:
                pose = self.model.fk(dict(zip(jt.joint_names, pt.positions)))
                m = error_metrics(pose, self.alignment_target, self.radius)
                m['orientation_edge_m'] = m['edge_bound_m'] - m['xy_m']
                points.append({'positions': list(pt.positions), 'velocities': list(pt.velocities),
                               'accelerations': list(pt.accelerations),
                               't': pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                               'tcp': pose.tolist(), 'to_target': m})
            trace['plans'].append({'step': what, 'attempt': trace['current_attempt'],
                                   'target_tcp': self.alignment_target.tolist(),
                                   'joint_names': list(jt.joint_names), 'points': points})
        except Exception as exc:
            trace['recording_errors'].append('plan: ' + str(exc))

    def finish_alignment_record(self, status, error=None):
        trace = self.alignment_trace
        self.alignment_trace = None
        if trace is None:
            return
        try:
            trace['status'] = status
            trace['error'] = error
            trace['finished_wall_ns'] = time.time_ns()
            trace['final_sample'] = self.alignment_sample()
            final = trace['final_sample']
            if final:
                trace['evidence_flags'] = {
                    'actual_target_edge_exceeds_alignment_limit': final['actual_to_target']['edge_bound_m'] > ALIGN_EDGE_M,
                    'controller_reference_target_edge_exceeds_alignment_limit': final['desired_to_target']['edge_bound_m'] > ALIGN_EDGE_M,
                    'actual_reference_edge_exceeds_alignment_limit': final['actual_to_desired']['edge_bound_m'] > ALIGN_EDGE_M,
                    'last_sample_stale': final['age_wall_s'] > 0.25,
                }
            if trace['plans'] and trace['plans'][-1]['points']:
                trace['last_plan_endpoint_to_target'] = trace['plans'][-1]['points'][-1]['to_target']
            trace['interpretation'] = (
                'Evidence only: compare requested target, returned plan endpoint, controller reference and actual. '
                'Do not identify a root cause from action SUCCESS alone. Samples during correction are moving states. '
                'edge_bound = XY error + orientation corner-displacement bound; this is not a measured collision. '
                'Final sample can be stale after an exception; use age_wall_s and pre_attempts.')
            root = Path(os.environ.get('PAC_DIAGNOSTIC_DIR', str(Path(__file__).resolve().parents[1] / 'logs/vertical_v46')))
            root.mkdir(parents=True, exist_ok=True)
            safe = re.sub(r'[^A-Za-z0-9_-]+', '_', trace['step'])
            file = root / f"alignment_{trace['started_wall_ns']}_{safe}.json"
            file.write_text(json.dumps(trace, indent=2, allow_nan=False))
            print(f'    [alignment] {status}; evidence: {file}', flush=True)
        except Exception as exc:
            # Diagnostic I/O must not replace the original rejection/exception.
            print(f'    [alignment] evidence write failed: {exc}', flush=True)

    def _prepare(self, xy, yaw, size, what):
        """Original alignment motion and 0.5 mm gate; add evidence only."""
        from geometry_msgs.msg import Quaternion
        self.initialize()
        self.radius = float(np.linalg.norm(size))
        desired = np.eye(4)
        desired[:3, :3] = np.column_stack(((0, 0, -1), (math.cos(yaw), math.sin(yaw), 0),
                                         (math.sin(yaw), -math.cos(yaw), 0)))
        initial = self.model.fk(dict(zip(self.latest[2], self.latest[4])))
        desired[:3, 3] = (float(xy[0]), float(xy[1]), float(initial[2, 3]))
        self.alignment_target = desired
        self.alignment_trace = {'schema_version': 1, 'kind': 'pre_descent_alignment', 'step': what,
            'started_wall_ns': time.time_ns(), 'box_size_m': [float(s) for s in size],
            'requested_xy_m': [float(x) for x in xy], 'requested_yaw_rad': float(yaw),
            'radius_m': self.radius, 'align_limit_m': ALIGN_EDGE_M, 'speed': DESCENT_SPEED,
            'pre_attempts': [], 'plans': [], 'samples': [], 'recording_errors': [],
            'dropped_samples': 0, 'current_attempt': 0}
        self.alignment_stamp = None
        status, failure = 'INTERRUPTED', None
        try:
            for attempt in range(3):
                self.alignment_trace['current_attempt'] = attempt
                now = self.current()
                desired[:3, 3] = (float(xy[0]), float(xy[1]), float(now[2, 3]))
                measured = error_metrics(now, desired, self.radius)
                sample = self.alignment_sample()
                self.alignment_trace['pre_attempts'].append({
                    'attempt': attempt, 'target_tcp': desired.tolist(), 'sample': sample,
                    'gate_error': measured})
                print(f"    [alignment] {what} attempt {attempt}: XY {measured['xy_m']*1000:.3f} mm, "
                      f"orientation {measured['orientation_deg']:.4f} deg, "
                      f"orientation-edge {(measured['edge_bound_m']-measured['xy_m'])*1000:.3f} mm, "
                      f"total {measured['edge_bound_m']*1000:.3f} mm / limit {ALIGN_EDGE_M*1000:.3f} mm", flush=True)
                if measured['edge_bound_m'] <= ALIGN_EDGE_M:
                    self.reference = now.copy()
                    q = quaternion_from_matrix(now[:3, :3])
                    self.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
                    print(f"    [vertical] aligned at z={now[2,3]:.4f}; XY/full orientation locked", flush=True)
                    status = 'PASS'
                    return
                if attempt < 2:
                    self.node.line(desired[:3, 3], yaw, what + ' (align above slot)', speed=DESCENT_SPEED)
            # Bounded passive settling only: the R3 trace shows the controller reference
            # reached its target while actual joints were still converging. No new move,
            # tolerance relaxation or collision override is allowed here.
            settle_sec = 5.0
            settle_deadline = time.monotonic() + settle_sec
            stable = 0
            last_stamp = None
            settle = {'max_wall_s': settle_sec, 'required_fresh_samples': 3,
                      'samples': [], 'result': 'TIMEOUT'}
            self.alignment_trace['passive_settle'] = settle
            print(f'    [alignment] waiting up to {settle_sec:.1f}s for natural controller settling '
                  f'(limit {ALIGN_EDGE_M*1000:.3f} mm; no extra motion)', flush=True)
            while time.monotonic() < settle_deadline:
                now = self.current()  # fails closed if the controller state becomes stale
                stamp = self.latest[1]
                if last_stamp is not None and stamp <= last_stamp:
                    self.node.spin_for(0.05)
                    continue
                last_stamp = stamp
                desired[:3, 3] = (float(xy[0]), float(xy[1]), float(now[2, 3]))
                measured = error_metrics(now, desired, self.radius)
                stable = stable + 1 if measured['edge_bound_m'] <= ALIGN_EDGE_M else 0
                settle['samples'].append({'stamp': float(stamp), 'metrics': measured,
                                          'consecutive_within_limit': stable})
                if stable >= settle['required_fresh_samples']:
                    self.reference = now.copy()
                    q = quaternion_from_matrix(now[:3, :3])
                    self.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                                  z=float(q[2]), w=float(q[3]))
                    settle['result'] = 'PASS'
                    print(f"    [alignment] passive settling PASS: edge {measured['edge_bound_m']*1000:.3f} mm, "
                          f"3 fresh samples; XY/full orientation locked", flush=True)
                    status = 'PASS'
                    return
                self.node.spin_for(0.05)
            status = 'REJECTED'
            raise self.Failure('vertical pre-alignment exceeds 0.5 mm edge budget '
                               'after 3 alignment attempts + 5s passive settling; no descent')
        except BaseException as exc:
            failure = str(exc) or type(exc).__name__
            if status != 'REJECTED':
                status = 'ERROR'
            raise
        finally:
            self.finish_alignment_record(status, failure)


    def descend(self, z, yaw, what, avoid_collisions=True):
        if self.reference is None:
            raise self.Failure("vertical axis was not prepared")
        now = self.current()
        if abs(float(z)-now[2, 3]) < 0.0005:
            return
        if float(z) > now[2, 3]:
            raise self.Failure(f"{what}: target is above current TCP; descent blocked")
        try:
            self.node.line((self.reference[0, 3], self.reference[1, 3], float(z)), yaw, what,
                           avoid_collisions=avoid_collisions, speed=DESCENT_SPEED,
                           orientation=self.orientation, vertical_guard=self)
        except RuntimeError as exc:
            raise self.Failure(str(exc)) from exc

    def audit(self, trajectory, what):
        jt = trajectory.joint_trajectory
        self.failure = None
        self.previous_stamp = None
        poses = [self.model.fk(dict(zip(jt.joint_names, pt.positions))) for pt in jt.points]
        errors = [error_metrics(p, self.reference, self.radius) for p in poses]
        peak = max((e["edge_bound_m"] for e in errors), default=float("inf"))
        self.active_trace = {"step": what, "axis_tcp": self.reference.tolist(),
                             "radius_m": self.radius, "plan_max_edge_m": peak,
                             "limits_m": {"plan": PLAN_EDGE_M, "execution": EXEC_EDGE_M},
                             "samples": [], "planned": [
                                 {"positions": list(pt.positions), "velocities": list(pt.velocities),
                                  "accelerations": list(pt.accelerations),
                                  "t": pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9,
                                  "tcp": p[:3, 3].tolist(), "error": e}
                                 for pt, p, e in zip(jt.points, poses, errors)],
                             "joint_names": list(jt.joint_names)}
        # Check the actual start too: a previous successful action may still be settling.
        now = self.current()
        bad_start = error_metrics(now, self.reference, self.radius)["edge_bound_m"] > PLAN_EDGE_M
        upward = any(b[2, 3] > a[2, 3]+0.0001 for a, b in zip(poses, poses[1:]))
        if not poses or peak > PLAN_EDGE_M or bad_start or upward or self.failure:
            self.finish(False)
            raise self.Failure(f"nonvertical Cartesian plan/start (max edge {peak*1000:.2f} mm); not executed")
        print(f"    [vertical] planned edge bound {peak*1000:.3f} mm", flush=True)

    def settle_endpoint(self, target_z, max_wall_s=5.0, required_samples=3):
        """Read-only goal check after the trajectory action reports success.

        A position-controlled joint may still be following the final command.
        Never issue a new motion or relax the 1 mm Z / 1.5 mm lateral budget.
        Require distinct, fresh controller stamps and a settled goal reference.
        """
        trace = self.active_trace
        evidence = {"limit_z_m": 0.001, "limit_edge_m": EXEC_EDGE_M,
                    "max_wall_s": max_wall_s, "required_fresh_samples": required_samples,
                    "samples": [], "result": "PENDING"}
        trace["endpoint_settle"] = evidence
        deadline = time.monotonic() + max_wall_s
        started = time.monotonic()
        previous_stamp = self.latest[1] if self.latest is not None else None
        consecutive = 0
        while time.monotonic() < deadline and self.failure is None:
            # Spins the existing ROS callbacks, allowing the joints to settle.
            # There is no new robot target and no unverified trajectory here.
            try:
                self.node.spin_for(0.05)
                sample = self.latest
                if sample is None or sample[1] == previous_stamp:
                    continue
                received, stamp, names, desired, actual = sample
                previous_stamp = stamp
                age = time.monotonic() - received
                if age > 0.25:
                    self.stop("descent endpoint controller feedback stale")
                    break
                pd = self.model.fk(dict(zip(names, desired)))
                pa = self.model.fk(dict(zip(names, actual)))
                desired_dz = float(pd[2, 3] - target_z)
                actual_dz = float(pa[2, 3] - target_z)
                edge = error_metrics(pa, self.reference, self.radius)["edge_bound_m"]
                if not all(math.isfinite(v) for v in (desired_dz, actual_dz, edge)):
                    self.stop("descent endpoint has nonfinite controller values")
                    break
                entry = {"stamp": stamp, "age_wall_s": age,
                         "desired_z_m": float(pd[2, 3]), "actual_z_m": float(pa[2, 3]),
                         "target_z_m": float(target_z), "desired_dz_m": desired_dz,
                         "actual_dz_m": actual_dz, "edge_bound_m": edge}
                evidence["samples"].append(entry)
                if actual_dz < -0.001:
                    self.stop(f"descent endpoint overshoot {actual_dz * 1000:.2f} mm; do not contact/release")
                    break
                if edge > EXEC_EDGE_M:
                    self.stop(f"descent endpoint lateral/orientation edge {edge * 1000:.2f} mm exceeds limit")
                    break
                inside = (abs(actual_dz) <= 0.001 and abs(desired_dz) <= 0.001
                          and edge <= EXEC_EDGE_M)
                consecutive = consecutive + 1 if inside else 0
                entry["consecutive_within_limit"] = consecutive
                if consecutive >= required_samples:
                    evidence["result"] = "PASS"
                    evidence["elapsed_wall_s"] = round(time.monotonic() - started, 4)
                    trace["end_z_error_m"] = abs(actual_dz)
                    print(f"    [vertical] endpoint settle PASS: Z {actual_dz * 1000:+.3f} mm / "
                          f"limit 1.000 mm, {required_samples} fresh samples", flush=True)
                    return True
            except (KeyError, ValueError, TypeError, IndexError) as exc:
                self.stop(f"descent endpoint feedback invalid: {exc}")
                break
        evidence["elapsed_wall_s"] = round(time.monotonic() - started, 4)
        evidence["result"] = "FAIL"
        if self.failure is None:
            last = evidence["samples"][-1] if evidence["samples"] else None
            suffix = (f"last actual Z error {last['actual_dz_m'] * 1000:+.3f} mm"
                      if last is not None else "no fresh controller sample")
            self.stop(f"descent endpoint not settled within {max_wall_s:.1f} s: {suffix}; do not contact/release")
        return False

    def finish(self, completed):
        trace = self.active_trace
        if trace is None:
            return
        trace["completed"] = bool(completed)
        if completed and self.failure is None and trace.get("planned"):
            # Run while active_trace is still set: lateral watchdog and controller
            # samples continue to be recorded during passive endpoint settling.
            self.settle_endpoint(trace["planned"][-1]["tcp"][2])
        self.active_trace = None
        if completed and self.latest is not None and trace.get("planned"):
            end = self.model.fk(dict(zip(self.latest[2], self.latest[4])))
            trace["end_z_error_m"] = abs(float(end[2, 3]) - trace["planned"][-1]["tcp"][2])
            if trace["end_z_error_m"] > 0.001 and self.failure is None:
                self.failure = "descent end height differs by more than 1 mm; do not release"
        elif completed and self.failure is None:
            self.failure = "descent endpoint controller evidence absent; do not contact/release"
        trace["guard_failure"] = self.failure
        # The old WITHIN_LIMITS covers XY/tilt only; never present a failed
        # endpoint-Z check as a successful full descent.
        lateral = classify(trace["plan_max_edge_m"], trace["samples"],
                           (completed or bool(self.failure)) and not trace.get("stream_stale"))
        trace["lateral_assessment"] = lateral
        trace["assessment"] = ("ENDPOINT_Z_FAIL: " + self.failure if completed and self.failure
                               else lateral)
        root = Path(os.environ.get("PAC_DIAGNOSTIC_DIR", str(Path(__file__).resolve().parents[1] / "logs/vertical_v46")))
        root.mkdir(parents=True, exist_ok=True)
        safe = re.sub(r"[^A-Za-z0-9_-]+", "_", trace["step"])
        file = root / f"vertical_{time.time_ns()}_{safe}.json"
        file.write_text(json.dumps(trace, indent=2))
        print(f"    [vertical] {trace['assessment']}\n    [vertical] evidence: {file}", flush=True)
        if completed and (self.failure or len(trace["samples"]) < 2):
            raise self.Failure(self.failure or "insufficient controller samples; placement not confirmed")
