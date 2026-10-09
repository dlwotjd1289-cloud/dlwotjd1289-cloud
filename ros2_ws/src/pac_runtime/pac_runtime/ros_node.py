"""ROS 2 node around ``RuntimeCore`` (JSON in ``std_msgs/String``).

Topics (all JSON):
  in  /pac/observation       {"box_id", "label_sku" | null, "weight_kg", "size_m": [x, y, z],
                              "confidence", "visual_damage", "stamp"}
  in  /pac/execution_result  {"state_version", "ok", "attempts", "other_grasp",
                              "measured_pose": [x, y, z, yaw] (pallet frame, min corner),
                              "issues": [..] | null, "repack_poses": {box_id: [x, y, z, yaw]}}
  in  /pac/conveyor_idle     {"idle_s"}
  in  /pac/conveyor_preview  {"boxes": [observation, ...]}: the boxes the conveyor camera sees
                              after the current one, in arrival order (look-ahead window;
                              each item has the /pac/observation fields)
  out /pac/command           ``command_to_dict`` (action, box, slot, target pose as min corner
                              and as box centre, stage-6 joints / gripper yaw / cycle time)
  out /pac/status            state version, pallet, mode, counts

Parameters: ``order_file`` (see ``order.py``), ``candidates_config``,
``highlevel_config``, ``lookahead_config``, ``runtime_config``,
``robot_config``, ``ranker`` (layer | dblf | donghan), ``ranker_model_path``,
``ranker_config`` and ``ranker_seed``. Stage 4 always decides with the
look-ahead search over the boxes on ``/pac/conveyor_preview``.

Run (after ``colcon build``)::

    ros2 run pac_runtime runtime_node --ros-args -p order_file:=/path/order.json

One command is outstanding at a time; the next one is published after its
result arrives (or after a new observation when the cell was idle).
"""

import json

from pac_common import Pose3D

from .core import ExecutionReport
from .perception import RawObservation
from pac_common import Size3D


def command_to_dict(cmd, box=None):
    out = {"action": cmd.action, "state_version": cmd.state_version, "box_id": cmd.box_id, "slot": cmd.slot,
           "reason": cmd.reason, "decided_by": cmd.decided_by}
    if cmd.candidate is not None:
        p = cmd.candidate.target_pose
        out["target_min_corner"] = [p.x, p.y, p.z, p.yaw]
        out["candidate_id"] = cmd.candidate.candidate_id
        if box is not None:
            from pac_candidates.geometry import rotated_dims

            dx, dy, dz = rotated_dims(box.size, p.yaw)
            out["target_center"] = [p.x + dx / 2, p.y + dy / 2, p.z + dz / 2, p.yaw]
    if cmd.robot:
        out["robot"] = {k: cmd.robot[k] for k in ("gripper_yaw_rad", "q_place", "q_approach", "q_path",
                                                  "cycle_time_s", "arm_clearance_m") if k in cmd.robot}
    if cmd.repack:
        out["repack"] = [{"box_id": b, "target_min_corner": [c.target_pose.x, c.target_pose.y, c.target_pose.z,
                                                             c.target_pose.yaw],
                          "robot": {k: det[k] for k in ("gripper_yaw_rad", "q_place", "q_approach", "cycle_time_s")
                                    if k in det}} for b, c, det in cmd.repack]
        # the current box is planned again after the moves (next command)
        out.pop("target_min_corner", None)
        out.pop("target_center", None)
        out.pop("candidate_id", None)
    return out


def observation_from_dict(d):
    return RawObservation(d["box_id"], d.get("label_sku"), float(d["weight_kg"]), Size3D(*d["size_m"]),
                          float(d.get("confidence", 1.0)), bool(d.get("visual_damage", False)),
                          d.get("view", "top"), float(d.get("stamp", 0.0)))


def report_from_dict(d):
    pose = lambda v: Pose3D("pallet", v[0], v[1], v[2], yaw=v[3]) if v is not None else None  # noqa: E731
    return ExecutionReport(ok=bool(d.get("ok", True)), attempts=int(d.get("attempts", 1)),
                           other_grasp=bool(d.get("other_grasp", False)), measured_pose=pose(d.get("measured_pose")),
                           issues=tuple(d["issues"]) if d.get("issues") is not None else None,
                           repack_poses={k: pose(v) for k, v in (d.get("repack_poses") or {}).items()})


def make_runtime_ranker(name="layer", model_path="", seed=7, config_path=""):
    """Build the stage-5 ranker without changing the PPO contract.

    ``layer`` (default) orders by the flat-layer / side-contact score, the
    same rule the look-ahead search uses. ``dblf`` is the old
    deepest-bottom-left order.  ``donghan`` only replaces
    the low-level ordering inside :class:`RobotAwarePlacer`; stage 4 still uses
    its declared proxy + DBLF value-provider contract.  An empty model path is
    intentional: no learned model is assumed compatible with the live EMS
    backend until its saved backend fingerprint has been checked.
    """
    selected = str(name).strip().lower()
    if selected in ("", "layer"):
        from .placer import layer_ranker

        return layer_ranker()
    if selected == "dblf":
        from .placer import dblf_order

        rank = lambda valid, box, state, backend: dblf_order(valid)  # noqa: E731
        rank.name = "dblf"
        return rank
    if selected == "donghan":
        from .placer import donghan_ranker
        from pac_planning.config import load_config

        config = load_config(config_path) if config_path else None
        return donghan_ranker(model_path or None, planner_config=config, seed=int(seed))
    raise ValueError(f"unknown runtime ranker: {name!r} (expected layer, dblf or donghan)")


class CoreBridge:
    """ROS-free message handling (unit-tested); the node only moves strings."""

    def __init__(self, core):
        self.core = core
        self.pending = None
        # Observations / idle reports that arrive while a command is being
        # executed (the next box reaches the pick point during a placement).
        # Applying them at once would change the state under the outstanding
        # command (STALE_RESULT); they are applied right after its result.
        self.deferred = []

    def _next(self):
        if self.pending is None and self.core.has_work():
            cmd = self.core.next_command()
            while cmd.action == "WAIT" and cmd.reason == "REPACK_NOT_EXECUTABLE":
                cmd = self.core.next_command()  # the repack counter rises: terminates
            if cmd.action != "WAIT":
                self.pending = cmd
                box = self.core.sm.tracked.get(cmd.box_id)
                return command_to_dict(cmd, box)
        return None

    def on_observation(self, text):
        """Returns (verdict, command); verdict None: duplicate or deferred
        (``self.deferred`` non-empty) while a command is outstanding."""
        obs = observation_from_dict(json.loads(text))
        if self.pending is not None:
            self.deferred.append(("obs", obs))
            return None, None
        verdict = self.core.on_observation(obs)  # None: duplicate
        out = self._next()
        return verdict, out

    def on_preview(self, text):
        """Look-ahead window update. It is search input only (no state
        change), so it is applied at once, even with a command outstanding."""
        boxes = json.loads(text).get("boxes", [])
        self.core.on_preview([observation_from_dict(b) for b in boxes])

    def on_result(self, text):
        d = json.loads(text)
        if self.pending is None:
            raise ValueError("no outstanding command")
        if int(d.get("state_version", self.pending.state_version)) != self.pending.state_version:
            raise ValueError("result for another command")
        completed_action = self.pending.action
        level = self.core.on_result(self.pending, report_from_dict(d))
        self.pending = None
        deferred, self.deferred = self.deferred, []
        for kind, item in deferred:
            if kind == "obs":
                self.core.on_observation(item)
            else:
                self.core.on_conveyor_idle(item)
        # These commands release the conveyor box.  The plant publishes the
        # next observation immediately afterwards, so planning a buffered-box
        # command here would use the pre-arrival state and become stale as soon
        # as that observation lands.  Let on_observation/on_idle plan it from
        # the authoritative new snapshot instead (unless it already arrived).
        if not deferred and completed_action in ("PLACE_CURRENT", "BUFFER_CURRENT", "REJECT_NG"):
            return level, None
        return level, self._next()

    def on_idle(self, text):
        idle_s = float(json.loads(text)["idle_s"])
        if self.pending is not None:
            self.deferred.append(("idle", idle_s))
            return {}, None
        out = self.core.on_conveyor_idle(idle_s)
        return dict(out), self._next()

    def status(self):
        sm = self.core.sm
        ranker = self.core.placer.ranker
        out = {"state_version": sm.version, "pallet_id": sm.pallet_id, "placed": len(sm.placed),
               "buffer": sm.buffer_slots(), "mode": self.core.supervisor.mode.value,
               "inspection": len(self.core.inspection), "counts": dict(self.core.counts),
               "policy": "lookahead", "visible_boxes": len(self.core.preview),
               "ranker": getattr(ranker, "name", "custom"),
               "deferred": len(self.deferred)}
        if ranker is not None and hasattr(ranker, "provenance"):
            out["ranker_provenance"] = ranker.provenance()
        return out


def main(args=None):  # pragma: no cover - needs ROS 2
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from pac_candidates import load_candidate_config
    from pac_highlevel import load_highlevel_config, load_policy
    from pac_robot_check import RobotFeasibility, load_robot_check_config

    from .config import load_runtime_config
    from .core import RuntimeCore
    from .order import load_order

    class RuntimeNode(Node):
        def __init__(self):
            super().__init__("pac_runtime")
            p = {n: self.declare_parameter(n, d).value for n, d in (
                ("order_file", ""), ("candidates_config", ""), ("highlevel_config", ""),
                ("lookahead_config", ""), ("runtime_config", ""), ("robot_config", ""),
                ("ranker", "layer"), ("ranker_model_path", ""), ("ranker_config", ""), ("ranker_seed", 7))}
            cand = load_candidate_config(p["candidates_config"])
            hl = load_highlevel_config(p["highlevel_config"])
            policy = load_policy("lookahead", p["lookahead_config"] or None, config=hl)
            ranker = make_runtime_ranker(p["ranker"], p["ranker_model_path"], p["ranker_seed"], p["ranker_config"])
            core = RuntimeCore(load_order(p["order_file"], cand), cand, hl, load_runtime_config(p["runtime_config"]),
                               RobotFeasibility(load_robot_check_config(p["robot_config"])), policy, ranker=ranker)
            self.bridge = CoreBridge(core)
            self.cmd_pub = self.create_publisher(String, "/pac/command", 10)
            self.status_pub = self.create_publisher(String, "/pac/status", 10)
            self.create_subscription(String, "/pac/observation", self._obs, 10)
            self.create_subscription(String, "/pac/execution_result", self._result, 10)
            self.create_subscription(String, "/pac/conveyor_idle", self._idle, 10)
            self.create_subscription(String, "/pac/conveyor_preview", self._preview, 10)

        def _publish(self, cmd):
            if cmd is not None:
                self.cmd_pub.publish(String(data=json.dumps(cmd)))
            self.status_pub.publish(String(data=json.dumps(self.bridge.status())))

        def _obs(self, msg):
            try:
                verdict, cmd = self.bridge.on_observation(msg.data)
            except (ValueError, KeyError, TypeError) as error:
                self.get_logger().error(f"observation rejected: {error}")
                return
            if verdict is None and self.bridge.pending is not None:
                self.get_logger().info("observation -> deferred until the current command's result")
            else:
                self.get_logger().info("observation -> " + (verdict.kind.value if verdict else "duplicate, ignored"))
            self._publish(cmd)

        def _preview(self, msg):
            try:
                self.bridge.on_preview(msg.data)
            except (ValueError, KeyError, TypeError) as error:
                self.get_logger().error(f"conveyor_preview rejected: {error}")

        def _result(self, msg):
            try:
                level, cmd = self.bridge.on_result(msg.data)
            except (ValueError, KeyError, TypeError) as error:
                # STALE_RESULT / result for another command: the command stays
                # outstanding; never commit a pose that the state cannot explain.
                self.get_logger().error(f"result rejected, command still outstanding: {error}")
                self._publish(None)
                return
            self.get_logger().info(f"result -> {level}")
            self._publish(cmd)

        def _idle(self, msg):
            try:
                missing, cmd = self.bridge.on_idle(msg.data)
            except (ValueError, KeyError, TypeError) as error:
                self.get_logger().error(f"conveyor_idle rejected: {error}")
                return
            if missing:
                self.get_logger().warn(f"MISSING confirmed: {missing}")
            self._publish(cmd)

    rclpy.init(args=args)
    node = RuntimeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
