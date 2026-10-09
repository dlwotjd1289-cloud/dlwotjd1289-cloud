"""Stage 8: the single writer of ACTUAL SystemState (common standard section 12).

Other modules hand in observations, plans and execution results; only this
class changes the plant state, and every change increments state_version so
plans made on an older snapshot are detected (STALE_PLAN / STALE_RESULT).

Commit rule for executed placements (team decision 2026-10-08):
the measured pose is compared with the planned target (both lower-corner
poses in frame "pallet"). Within ``CommitTolerance`` the PLANNED pose is
committed, so neighbouring boxes stay exactly non-overlapping for the
planner (contact solvers and sensors leave ~0.01 mm penetrations that the
planner's 1e-8 m geometry checks reject). Outside the tolerance the box is
not committed at the planned pose:

* ``commit_execution`` (single-shot executors) marks the box FAILED and
  returns SENSOR_UNCERTAIN; the caller re-perceives and calls it again.
* ``place`` (runtime loop, stage 7 already re-measured the box from the top
  view) records the re-measured pose after ``reconcile``.

Inventory (section 11): ``tracked_boxes`` holds every box that got an ID and
is still in the cell (pending, buffered, or PLACED on the current pallet);
``remaining_by_sku`` holds boxes of the order list not seen yet. Boxes of a
closed pallet leave ``tracked_boxes`` with the pallet.

Two ways to create one:

    StateManager(initial_state)                         # snapshot based (V4.5 scripts)
    StateManager.for_order(pallet_size, catalog, expected_by_sku,
                           pallet_max_weight_kg=1000, buffer_slots=4)   # runtime loop
"""

from collections import Counter
from dataclasses import dataclass, replace
import math

from .frames import rotated_dims
from .models import (
    BoxState,
    BoxStatus,
    ExecutionResult,
    InventoryState,
    PalletState,
    PlacedBox,
    PlacementCandidate,
    Pose3D,
    RejectCode,
    SystemState,
    validate_box,
)
from .planning import PlanningContext


@dataclass(frozen=True)
class CommitTolerance:
    xy_m: float = 0.005
    z_m: float = 0.003
    yaw_rad: float = math.radians(1.0)


@dataclass(frozen=True)
class CommitOutcome:
    state: SystemState
    placed: bool
    codes: tuple[RejectCode, ...] = ()
    reason: str = ""


def _yaw_error(a, b):
    """Box footprints are symmetric under 180 deg."""
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def pose_error(measured: Pose3D, planned: Pose3D):
    """(xy, |z|, yaw) error between two pallet-frame corner poses."""
    if measured.frame_id != planned.frame_id:
        raise ValueError("Poses must share frame_id")
    return (
        math.hypot(measured.x - planned.x, measured.y - planned.y),
        abs(measured.z - planned.z),
        _yaw_error(measured.yaw, planned.yaw),
    )


def within_tolerance(measured: Pose3D, planned: Pose3D, tolerance: CommitTolerance = CommitTolerance()):
    xy, z, yaw = pose_error(measured, planned)
    return xy <= tolerance.xy_m and z <= tolerance.z_m and yaw <= tolerance.yaw_rad


PENDING = (BoxStatus.MEASURED, BoxStatus.ON_CONVEYOR, BoxStatus.READY_FOR_PICK, BoxStatus.BUFFERED,
           BoxStatus.PICKING, BoxStatus.IN_TRANSIT)


class StateManager:
    """Single writer for ACTUAL SystemState."""

    def __init__(self, initial_state: SystemState, tolerance: CommitTolerance = CommitTolerance(), *,
                 catalog=None, pallet_max_weight_kg=None, buffer_slots=0, pallet_prefix=None):
        self.tolerance = tolerance
        self.version = initial_state.state_version
        self.t = initial_state.stamp_sec
        self.pallet_size = initial_state.pallet.size
        self.prefix = pallet_prefix
        self._first_pallet_id = initial_state.pallet.pallet_id
        self.pallet_index = 0
        self.placed = list(initial_state.pallet.boxes)
        self.closed = []
        self.tracked = dict(initial_state.inventory.tracked_boxes)
        self.remaining = Counter({k: v for k, v in initial_state.inventory.remaining_by_sku.items() if v > 0})
        self.catalog = dict(catalog or {})
        self.max_load = None if pallet_max_weight_kg is None else float(pallet_max_weight_kg)
        self.slots = [None] * buffer_slots
        self.buffered_at = {}
        self.decisions = 0
        self.uncertain = set()
        self.no_load = set()
        self._pending: dict[str, tuple[PlacementCandidate, tuple]] = {}

    @classmethod
    def for_order(cls, pallet_size, catalog, expected_by_sku, *, pallet_max_weight_kg, pallet_prefix="PALLET",
                  buffer_slots=4, tolerance: CommitTolerance = CommitTolerance()):
        """Empty pallet, nothing seen yet, ``expected_by_sku`` still to come."""
        initial = SystemState(0, 0.0, PalletState(f"{pallet_prefix}-01", pallet_size, ()),
                              InventoryState({}, dict(expected_by_sku)))
        return cls(initial, tolerance, catalog=catalog, pallet_max_weight_kg=pallet_max_weight_kg,
                   buffer_slots=buffer_slots, pallet_prefix=pallet_prefix)

    # -- reads ----------------------------------------------------------
    @property
    def pallet_id(self):
        if self.prefix is not None:
            return f"{self.prefix}-{self.pallet_index + 1:02d}"
        if self.pallet_index == 0:
            return self._first_pallet_id
        return f"{self._first_pallet_id}-{self.pallet_index + 1:02d}"

    def snapshot(self) -> SystemState:
        return SystemState(
            self.version, self.t,
            PalletState(self.pallet_id, self.pallet_size, tuple(self.placed)),
            InventoryState(dict(self.tracked), dict(+self.remaining)),
        )

    def pending_ids(self):
        """Tracked boxes not on the pallet (current, buffered, in hand)."""
        return {k for k, b in self.tracked.items() if b.status in PENDING}

    def context(self):
        if self.max_load is None:
            raise ValueError("context() needs pallet_max_weight_kg")
        return PlanningContext(
            catalog=self.catalog, pallet_max_weight_kg=self.max_load,
            capacity_overrides_n={b: 0.0 for b in self.no_load},
            buffer_capacity=len(self.slots),
            uncertain_box_ids=tuple(sorted(self.uncertain & self.pending_ids())),
        )

    def buffer_slots(self):
        return {i: b for i, b in enumerate(self.slots) if b is not None}

    def buffer_age(self):
        return {b: self.decisions - self.buffered_at[b] for b in self.slots if b is not None}

    def current_id(self):
        pending = [k for k, b in self.tracked.items() if b.status == BoxStatus.READY_FOR_PICK]
        return pending[0] if pending else None

    def cog_and_weight(self):
        total = sum(p.weight_kg for p in self.placed)
        if total <= 0:
            return None, 0.0
        cx = cy = 0.0
        for p in self.placed:
            dx, dy, _ = rotated_dims(p.size, p.pose.yaw)
            cx += p.weight_kg * (p.pose.x + dx / 2)
            cy += p.weight_kg * (p.pose.y + dy / 2)
        return (cx / total, cy / total), total

    # -- writes ---------------------------------------------------------
    def _bump(self, stamp_sec=None):
        self.version += 1
        if stamp_sec is not None:
            self.t = stamp_sec
        return self.snapshot()

    def _take_from_stock(self, sku_id):
        if self.remaining.get(sku_id, 0) <= 0:
            raise ValueError(f"No remaining stock for {sku_id}")
        self.remaining[sku_id] -= 1
        if self.remaining[sku_id] <= 0:
            del self.remaining[sku_id]

    def commit_observation(self, observed_box: BoxState, stamp_sec: float,
                           from_stock: bool = True) -> SystemState:
        """Track a newly observed box (or update a tracked one).

        A box that first gets an ID leaves anonymous stock: remaining_by_sku
        is decremented here, never again by planners (from_stock=False for
        boxes that were not counted in remaining_by_sku).
        """
        validate_box(observed_box)
        old = self.tracked.get(observed_box.box_id)
        if old is not None and old.status == BoxStatus.PLACED:
            raise ValueError("Cannot re-observe a PLACED box")
        if old is None and from_stock:
            self._take_from_stock(observed_box.sku_id)
        self.tracked[observed_box.box_id] = observed_box
        return self._bump(stamp_sec)

    def arrive(self, box, uncertain=False, no_load=False):
        """Runtime stage 2 -> 8: a validated box is ready for pick."""
        self._take_from_stock(box.sku_id)
        self.tracked[box.box_id] = replace(box, status=BoxStatus.READY_FOR_PICK)
        if uncertain:
            self.uncertain.add(box.box_id)
        if no_load:
            self.no_load.add(box.box_id)
        self._bump()

    def discard_expected(self, box_sku):
        """An arrival routed to inspection still consumes its order-list entry."""
        self._take_from_stock(box_sku)
        self._bump()

    def confirm_missing(self, by_sku):
        for sku, n in by_sku.items():
            self.remaining[sku] -= n
            if self.remaining[sku] <= 0:
                del self.remaining[sku]
        if by_sku:
            self._bump()

    def to_buffer(self, box_id, slot):
        if self.slots[slot] is not None:
            raise ValueError(f"buffer slot {slot} occupied")
        self.slots[slot] = box_id
        self.buffered_at[box_id] = self.decisions
        self.tracked[box_id] = replace(self.tracked[box_id], status=BoxStatus.BUFFERED)
        self._bump()

    def _free_slot_of(self, box_id):
        for i, b in enumerate(self.slots):
            if b == box_id:
                self.slots[i] = None

    def register_plan(self, candidate: PlacementCandidate) -> None:
        """Remember the candidate sent to the executor (one per box)."""
        if candidate.base_state_version != self.version:
            raise ValueError("STALE_PLAN: candidate built on another version")
        box = self.tracked.get(candidate.box_id)
        if box is None or box.status == BoxStatus.PLACED:
            raise ValueError("Box is not tracked or already placed")
        if candidate.box_id in self._pending:
            raise ValueError("Box already has a pending execution")
        self._pending[candidate.box_id] = (candidate, tuple(self.placed))

    def _put_on_pallet(self, box_id, pose, stamp_sec=None):
        box = self.tracked[box_id]
        self._free_slot_of(box_id)
        self.placed.append(PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg, pose))
        self.tracked[box_id] = replace(box, pose=pose, status=BoxStatus.PLACED,
                                       stamp_sec=box.stamp_sec if stamp_sec is None else stamp_sec)
        return box

    def commit_execution(self, result: ExecutionResult) -> CommitOutcome:
        """Single-shot executors: ExecutionResult -> CommitOutcome (commit rule above)."""
        pending = self._pending.pop(result.box_id, None)
        if pending is None:
            raise ValueError("No pending plan for this box (duplicate result?)")
        candidate, pallet_at_plan = pending
        if candidate.candidate_id != result.candidate_id:
            raise ValueError("Execution result is for another candidate")
        box = self.tracked[result.box_id]

        def fail(codes, reason):
            self.tracked[result.box_id] = replace(box, status=BoxStatus.FAILED, stamp_sec=result.stamp_sec)
            return CommitOutcome(self._bump(result.stamp_sec), False, tuple(codes), reason)

        if not result.success or result.actual_pose is None:
            return fail(result.codes or (RejectCode.EXECUTION_FAIL,), "execution reported failure")
        if tuple(self.placed) != pallet_at_plan:
            return fail((RejectCode.STALE_PLAN,), "pallet changed between plan and execution")
        xy, z, yaw = pose_error(result.actual_pose, candidate.target_pose)
        tol = self.tolerance
        if xy > tol.xy_m or z > tol.z_m or yaw > tol.yaw_rad:
            return fail(
                (RejectCode.SENSOR_UNCERTAIN,),
                f"measured pose differs from plan (xy {xy*1000:.1f} mm, "
                f"z {z*1000:.1f} mm, yaw {math.degrees(yaw):.2f} deg); "
                "re-perceive before planning",
            )
        self._put_on_pallet(result.box_id, candidate.target_pose, result.stamp_sec)
        return CommitOutcome(self._bump(result.stamp_sec), True)

    def place(self, box_id, measured_pose, planned_pose=None):
        """Runtime stage 7 -> 8: record a placed box.

        With ``planned_pose`` the commit rule applies: within the tolerance the
        planned pose is stored. Otherwise ``measured_pose`` is stored (the
        caller re-measured it and ran ``reconcile``). Returns the BoxState.
        """
        pose = measured_pose
        if planned_pose is not None and within_tolerance(measured_pose, planned_pose, self.tolerance):
            pose = planned_pose
        box = self._put_on_pallet(box_id, pose)
        self._bump()
        return box

    def move_placed(self, box_id, measured_pose):
        self.placed = [replace(p, pose=measured_pose) if p.box_id == box_id else p for p in self.placed]
        if box_id in self.tracked:
            self.tracked[box_id] = replace(self.tracked[box_id], pose=measured_pose)
        self._bump()

    def reject(self, box_id):
        box = self.tracked.pop(box_id)
        self._free_slot_of(box_id)
        self._bump()
        return box

    def close_pallet(self):
        self.closed.append((self.pallet_id, tuple(self.placed)))
        for p in self.placed:
            self.tracked.pop(p.box_id, None)
        self.placed = []
        self.pallet_index += 1
        self._bump()

    def reconcile(self, size, pose, tol=0.002, ignore=None, z_tol=0.015):
        """Make a measured pose consistent with the stored state.

        Measurement noise can leave the measured box a millimetre inside a
        neighbour (e.g. the lower box was measured slightly taller than it
        is) or outside the pallet edge. Downstream planners require a
        consistent state (no overlaps, inside the pallet), so overlaps and
        protrusions are resolved: z is lifted onto the stored top of the box
        below (up to ``z_tol``, the height error of a low-confidence
        measurement), x / y are pushed out along the smaller penetration (up
        to ``tol``). Larger conflicts are left for the post check (L4).
        Returns (pose, shift in m).
        """
        dx, dy, dz = rotated_dims(size, pose.yaw)
        x, y, z = pose.x, pose.y, pose.z
        for _ in range(4):
            moved = False
            for p in self.placed:
                if p.box_id == ignore:
                    continue
                px, py, pz = rotated_dims(p.size, p.pose.yaw)
                ox = min(x + dx, p.pose.x + px) - max(x, p.pose.x)
                oy = min(y + dy, p.pose.y + py) - max(y, p.pose.y)
                oz = min(z + dz, p.pose.z + pz) - max(z, p.pose.z)
                if ox <= 1e-9 or oy <= 1e-9 or oz <= 1e-9:
                    continue
                if oz <= z_tol and p.pose.z < z and min(ox, oy) > tol:  # resting on it: lift onto its stored top
                    z = p.pose.z + pz
                elif min(ox, oy) <= tol:                 # side contact: push out
                    if ox <= oy:
                        x += -ox if x < p.pose.x else ox
                    else:
                        y += -oy if y < p.pose.y else oy
                else:
                    continue
                moved = True
            X, Y = self.pallet_size.x, self.pallet_size.y
            nx, ny = min(max(x, 0.0), X - dx) if dx <= X else x, min(max(y, 0.0), Y - dy) if dy <= Y else y
            if abs(nx - x) <= tol and abs(ny - y) <= tol and (nx, ny) != (x, y):
                x, y, moved = nx, ny, True
            if not moved:
                break
        shift = ((x - pose.x) ** 2 + (y - pose.y) ** 2 + (z - pose.z) ** 2) ** 0.5
        return replace(pose, x=x, y=y, z=z), shift


__all__ = ["CommitOutcome", "CommitTolerance", "StateManager", "pose_error", "within_tolerance"]
