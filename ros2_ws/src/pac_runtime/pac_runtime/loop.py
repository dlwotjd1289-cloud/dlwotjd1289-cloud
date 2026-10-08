"""The 1 -> 8 runtime loop on a virtual cell (one box at a time).

1 perception -> 2 State Validator (NG / inspection branch) -> 3 Supervisor
-> 4 HighLevelDecider -> 5 candidates + hard mask + ranking -> 6 robot
feasibility (inside the placer: only executable candidates reach stage 4)
-> 7 execution + post check (escalation L0-L4) -> 8 State Manager (measured
state) -> next box.

The plant is simulated (``FieldBox`` truth, ``ExecutorSim``); everything the
decision stages see comes from the State Manager snapshot, exactly as with a
real cell.
"""

from collections import Counter
from dataclasses import dataclass, field
import random

from pac_common import PlacementCandidate, PalletState, Pose3D, SystemState
from pac_candidates.geometry import rotated_dims
from pac_highlevel import ActionType, HighLevelDecider

from .executor import ExecutorSim, TrueBox
from .perception import PerceptionSim
from .placer import RobotAwarePlacer
from .state_manager import StateManager
from .state_validator import StateValidator
from .supervisor import Supervisor


@dataclass
class CellSpec:
    """Everything known before the run (order list, pallet, catalog)."""
    stream: tuple                    # FieldBox in true arrival order (missing boxes absent)
    expected_by_sku: dict            # order list: SKU -> count (includes boxes that never come)
    pallet_size: object
    catalog: dict
    weight_ranges: dict = field(default_factory=dict)
    pallet_max_weight_kg: float = 1000.0
    pallet_prefix: str = "PALLET"


class RuntimeLoop:
    def __init__(self, cell, cand_config, hl_config, rt_config, robot, policy, ranker=None, log_events=True):
        self.cell = cell
        self.cand_config = cand_config
        self.hl = hl_config
        self.cfg = rt_config
        self.robot = robot
        self.policy = policy
        self.ranker = ranker
        self.log_events = log_events

    # ------------------------------------------------------------------
    def run(self):
        cell, cfg, hl = self.cell, self.cfg, self.hl
        rng = random.Random(cfg.seed)
        perception = PerceptionSim(cfg.perception, rng)
        validator = StateValidator(cell.catalog, cfg.validator, cell.weight_ranges)
        supervisor = Supervisor(cfg.supervisor, cell.expected_by_sku)
        sm = StateManager(cell.pallet_size, cell.catalog, cell.expected_by_sku,
                          pallet_max_weight_kg=cell.pallet_max_weight_kg, pallet_prefix=cell.pallet_prefix,
                          buffer_slots=hl.buffer.slots)
        executor = ExecutorSim(cfg.execution, cfg.verify, rng)
        placer = RobotAwarePlacer(self.robot, cfg.robot_checks_per_option, self.ranker)
        decider = HighLevelDecider(sm.context(), self.cand_config, hl, policy=self.policy, placer=placer)
        travel = hl.buffer.travel_times()

        truth, true_stack, pallets = {}, [], []
        inspection, events = [], []
        counts = Counter()
        anomalies = Counter()
        idx, repacks_for_current, missing_done = 0, 0, False

        def log(kind, **kw):
            if self.log_events:
                events.append({"t": round(sm.t, 2), "v": sm.version, "event": kind, **kw})

        def finish_pallet():
            if true_stack:
                vol = sum(tb.size.x * tb.size.y * tb.size.z for tb in true_stack)
                pallets.append({"pallet_id": sm.pallet_id, "boxes": len(true_stack),
                                "fill_true": vol / (cell.pallet_size.x * cell.pallet_size.y * cell.pallet_size.z),
                                "layout": [{"box_id": tb.box_id, "size": [tb.size.x, tb.size.y, tb.size.z],
                                            "pose": [tb.pose.x, tb.pose.y, tb.pose.z, tb.pose.yaw]}
                                           for tb in true_stack]})

        while True:
            # 1 + 2: next box from the conveyor
            if sm.current_id() is None and idx < len(cell.stream) and supervisor.can_pick():
                fb = cell.stream[idx]
                idx += 1
                supervisor.on_arrival(fb.truth.sku_id)
                obs = perception.observe(fb, sm.t)
                verdict = validator.validate(obs, perception, fb)
                anomalies[verdict.kind.value] += 1
                if verdict.route == "INSPECTION":
                    if obs.label_sku is not None or verdict.used_base_view:
                        sku = verdict.box.sku_id if verdict.box else (obs.label_sku or None)
                        if sku in sm.remaining:
                            sm.discard_expected(sku)
                    inspection.append({"box_id": fb.truth.box_id, "reason": verdict.kind.value, "stage": 2})
                    log("INSPECTION", box=fb.truth.box_id, reason=verdict.kind.value)
                    continue
                sm.arrive(verdict.box, uncertain=verdict.uncertain, no_load=verdict.no_load_on_top)
                truth[verdict.box.box_id] = fb
                repacks_for_current = 0
                log("ARRIVE", box=verdict.box.box_id, sku=verdict.box.sku_id, anomaly=verdict.kind.value)
            current = sm.current_id()
            if current is None and not sm.buffer_slots():
                if idx >= len(cell.stream):
                    break
                continue
            # 3: stream over -> confirm MISSING once (order list known)
            if idx >= len(cell.stream) and not missing_done:
                missing_done = True
                sm.t += cfg.supervisor.missing_timeout_s
                out = supervisor.confirm_missing(sm.t, cfg.supervisor.missing_timeout_s)
                sm.confirm_missing(out)
                if out:
                    log("MISSING", by_sku=dict(out))
            # 4 (+5, 6 inside the placer)
            decider.context = sm.context()
            state = sm.snapshot()
            d = decider.decide(state, current_box_id=current, buffer_slots=sm.buffer_slots(),
                               buffer_age=sm.buffer_age(), repack_attempts=repacks_for_current)
            sm.decisions += 1
            a = d.action.type if d.action is not None else None
            counts[a.value if a else "NONE"] += 1
            if a is None:
                break
            if a == ActionType.REJECT_NG:
                sm.reject(d.box.box_id)
                inspection.append({"box_id": d.box.box_id, "reason": d.reason, "stage": 4})
                log("NG", box=d.box.box_id, reason=d.reason)
            elif a == ActionType.BUFFER_CURRENT:
                sm.to_buffer(d.box.box_id, d.slot)
                sm.t += travel[d.slot]
                log("BUFFER", box=d.box.box_id, slot=d.slot)
            elif a == ActionType.PALLET_CLOSE:
                finish_pallet()
                log("CLOSE", pallet=sm.pallet_id, reason=d.reason, boxes=len(true_stack))
                sm.close_pallet()
                true_stack = []
                sm.t += supervisor.pallet_change(sm.t)
            elif a == ActionType.PARTIAL_REPACK:
                repacks_for_current += 1
                ok = self._repack(d, sm, executor, true_stack, truth, supervisor, counts, log)
                if not ok:
                    counts["repack_aborted_stage6"] += 1
            else:  # PLACE_CURRENT / RETRIEVE_BUFFER
                self._place(d, a, state, sm, executor, true_stack, truth, supervisor, travel, counts,
                            inspection, log)
        finish_pallet()
        return {
            "boxes_in_stream": len(cell.stream),
            "expected": sum(cell.expected_by_sku.values()),
            "placed": sum(p["boxes"] for p in pallets),
            "pallets": len(pallets),
            "fill_true_mean": (sum(p["fill_true"] for p in pallets) / len(pallets)) if pallets else 0.0,
            "time_s": round(sm.t, 1),
            "inspection": inspection,
            "missing": dict(supervisor.missing),
            "decisions": dict(counts),
            "anomalies": dict(anomalies),
            "stage6": dict(placer.stats),
            "supervisor_time_s": dict(supervisor.time_in),
            "supervisor_log": supervisor.log,
            "pallet_list": pallets,
            "events": events,
        }

    # ------------------------------------------------------------------
    def _place(self, d, a, state, sm, executor, true_stack, truth, supervisor, travel, counts, inspection, log):
        box, cand = d.box, d.candidate
        verdict6 = self.robot.validate_robot_motion(box, cand, state)
        if not verdict6.success:  # the placer only hands out executable candidates
            raise RuntimeError(f"stage 6 rejected the chosen candidate: {verdict6.codes}")
        ok, attempts, other = executor.grip()
        sm.t += self.cfg.execution.retry_time_s * (attempts - 1)
        if attempts > 1:
            counts["grip_retries"] += attempts - 1
        if not ok:
            counts["L3"] += 1
            sm.reject(box.box_id)
            inspection.append({"box_id": box.box_id, "reason": "GRIP_FAIL", "stage": 7})
            log("L3_GRIP_FAIL", box=box.box_id)
            return
        if other:
            counts["L3_other_grasp"] += 1
        fb = truth[box.box_id]
        ex = executor.place(box, fb.truth.size, cand, true_stack, sm.pallet_size)
        sm.t += verdict6.details["cycle_time_s"]
        if a == ActionType.RETRIEVE_BUFFER:
            sm.t += travel[d.slot]
        true_pose, measured = ex.true_pose, ex.measured_pose
        if ex.level == "L4":
            sm.t += supervisor.hold(sm.t, "L4: " + ",".join(ex.issues))
            # operator puts the box where it was planned
            p = cand.target_pose
            mdx, mdy, _ = rotated_dims(box.size, p.yaw)
            tdx, tdy, _ = rotated_dims(fb.truth.size, p.yaw)
            true_pose = Pose3D("pallet", p.x + (mdx - tdx) / 2, p.y + (mdy - tdy) / 2,
                               executor.settle_z((p.x, p.y), (p.x + tdx, p.y + tdy), true_stack), yaw=p.yaw)
            measured = p
        counts[ex.level] += 1
        true_stack.append(TrueBox(box.box_id, fb.truth.size, true_pose))
        sm.place(box.box_id, measured)
        log("PLACE", box=box.box_id, action=a.value, level=ex.level, dxy_mm=round(ex.dxy_m * 1000, 1),
            dz_mm=round(ex.dz_m * 1000, 1), issues=list(ex.issues),
            gripper_yaw=verdict6.details.get("gripper_yaw_rad"), cycle_s=verdict6.details["cycle_time_s"],
            pose=[round(v, 4) for v in (measured.x, measured.y, measured.z, measured.yaw)])

    def _repack(self, d, sm, executor, true_stack, truth, supervisor, counts, log):
        """Execute the repack moves one by one; each move passes stage 6."""
        started = sm.t
        for box_id, pose in d.repack_moves:
            state = sm.snapshot()
            placed = {p.box_id: p for p in state.pallet.boxes}
            pb = placed[box_id]
            without = SystemState(state.state_version, state.stamp_sec,
                                  PalletState(state.pallet.pallet_id, state.pallet.size,
                                              tuple(p for p in state.pallet.boxes if p.box_id != box_id)),
                                  state.inventory)
            cand = PlacementCandidate(f"repack-{box_id}", box_id, pose, state.state_version)
            v6 = self.robot.validate_robot_motion(pb, cand, without)
            if not v6.success:
                log("REPACK_ABORT", box=box_id, codes=[c.value for c in v6.codes])
                return False
            others = [tb for tb in true_stack if tb.box_id != box_id]
            tb_old = next(tb for tb in true_stack if tb.box_id == box_id)
            ex = executor.place(pb, tb_old.size, cand, others, sm.pallet_size)
            true_stack[:] = others + [TrueBox(box_id, tb_old.size, ex.true_pose)]
            sm.move_placed(box_id, ex.measured_pose)
            sm.t += v6.details["cycle_time_s"]
            counts["repack_moves"] += 1
            log("REPACK_MOVE", box=box_id, level=ex.level)
        supervisor.repacking(started, sm.t - started)
        return True
