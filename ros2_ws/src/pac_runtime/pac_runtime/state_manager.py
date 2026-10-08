"""Stage 8: State Manager. Single owner of the plant state; every change
bumps ``state_version`` so stale plans are detected (STALE_PLAN).

The pallet is stored as measured after execution (stage 7), not as planned.
``remaining_by_sku`` holds boxes not yet seen (order list minus arrivals and
confirmed MISSING); the current and buffered boxes live in
``tracked_boxes`` (statuses READY_FOR_PICK / BUFFERED).
"""

from collections import Counter
from dataclasses import replace

from pac_common import BoxStatus, InventoryState, PalletState, PlacedBox, PlanningContext, SystemState


class StateManager:
    def __init__(self, pallet_size, catalog, expected_by_sku, *, pallet_max_weight_kg, pallet_prefix="PALLET",
                 buffer_slots=4):
        self.pallet_size = pallet_size
        self.catalog = dict(catalog)
        self.max_load = float(pallet_max_weight_kg)
        self.prefix = pallet_prefix
        self.pallet_index = 0
        self.version = 0
        self.t = 0.0
        self.placed = []
        self.closed = []
        self.remaining = Counter(expected_by_sku)
        self.tracked = {}
        self.slots = [None] * buffer_slots
        self.buffered_at = {}
        self.decisions = 0
        self.uncertain = set()
        self.no_load = set()

    # -- reads ----------------------------------------------------------
    @property
    def pallet_id(self):
        return f"{self.prefix}-{self.pallet_index + 1:02d}"

    def snapshot(self):
        return SystemState(
            self.version, self.t,
            PalletState(self.pallet_id, self.pallet_size, tuple(self.placed)),
            InventoryState(dict(self.tracked), dict(+self.remaining)),
        )

    def context(self):
        return PlanningContext(
            catalog=self.catalog, pallet_max_weight_kg=self.max_load,
            capacity_overrides_n={b: 0.0 for b in self.no_load},
            buffer_capacity=len(self.slots),
            uncertain_box_ids=tuple(sorted(self.uncertain & set(self.tracked))),
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
        from pac_candidates.geometry import rotated_dims

        cx = cy = 0.0
        for p in self.placed:
            dx, dy, _ = rotated_dims(p.size, p.pose.yaw)
            cx += p.weight_kg * (p.pose.x + dx / 2)
            cy += p.weight_kg * (p.pose.y + dy / 2)
        return (cx / total, cy / total), total

    # -- writes ---------------------------------------------------------
    def _bump(self):
        self.version += 1

    def arrive(self, box, uncertain=False, no_load=False):
        self.remaining[box.sku_id] -= 1
        if self.remaining[box.sku_id] <= 0:
            del self.remaining[box.sku_id]
        self.tracked[box.box_id] = replace(box, status=BoxStatus.READY_FOR_PICK)
        if uncertain:
            self.uncertain.add(box.box_id)
        if no_load:
            self.no_load.add(box.box_id)
        self._bump()

    def discard_expected(self, box_sku):
        """An arrival routed to inspection still consumes its order-list entry."""
        self.remaining[box_sku] -= 1
        if self.remaining[box_sku] <= 0:
            del self.remaining[box_sku]
        self._bump()

    def to_buffer(self, box_id, slot):
        if self.slots[slot] is not None:
            raise ValueError(f"buffer slot {slot} occupied")
        self.slots[slot] = box_id
        self.buffered_at[box_id] = self.decisions
        self.tracked[box_id] = replace(self.tracked[box_id], status=BoxStatus.BUFFERED)
        self._bump()

    def place(self, box_id, measured_pose):
        """Record the box at its measured pose (stage 7 result)."""
        box = self.tracked.pop(box_id)
        for i, b in enumerate(self.slots):
            if b == box_id:
                self.slots[i] = None
        self.placed.append(PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg, measured_pose))
        self._bump()
        return box

    def move_placed(self, box_id, measured_pose):
        self.placed = [replace(p, pose=measured_pose) if p.box_id == box_id else p for p in self.placed]
        self._bump()

    def reject(self, box_id):
        box = self.tracked.pop(box_id)
        for i, b in enumerate(self.slots):
            if b == box_id:
                self.slots[i] = None
        self._bump()
        return box

    def close_pallet(self):
        self.closed.append((self.pallet_id, tuple(self.placed)))
        self.placed = []
        self.pallet_index += 1
        self._bump()

    def confirm_missing(self, by_sku):
        for sku, n in by_sku.items():
            self.remaining[sku] -= n
            if self.remaining[sku] <= 0:
                del self.remaining[sku]
        if by_sku:
            self._bump()
