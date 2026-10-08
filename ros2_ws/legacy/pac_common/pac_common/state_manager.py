from dataclasses import replace
from .models import BoxState, BoxStatus, ExecutionResult, PlacedBox, SystemState
from .validation import validate_box

class StateManager:
    """Single writer for ACTUAL SystemState."""
    def __init__(self, initial_state: SystemState): self._state = initial_state
    @property
    def snapshot(self) -> SystemState: return self._state
    def commit_observation(self, observed_box: BoxState, stamp_sec: float) -> SystemState:
        validate_box(observed_box)
        tracked = dict(self._state.inventory.tracked_boxes); tracked[observed_box.box_id] = observed_box
        inv = replace(self._state.inventory, tracked_boxes=tracked)
        self._state = replace(self._state, state_version=self._state.state_version+1, stamp_sec=stamp_sec, inventory=inv)
        return self._state
    def commit_execution(self, result: ExecutionResult) -> SystemState:
        tracked = dict(self._state.inventory.tracked_boxes)
        if result.box_id not in tracked: raise KeyError(result.box_id)
        box = tracked[result.box_id]
        if not result.success or result.actual_pose is None:
            tracked[result.box_id] = replace(box, status=BoxStatus.FAILED, stamp_sec=result.stamp_sec)
            self._state = replace(self._state, state_version=self._state.state_version+1, stamp_sec=result.stamp_sec,
                                  inventory=replace(self._state.inventory, tracked_boxes=tracked))
            return self._state
        placed = PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg, result.actual_pose)
        tracked[result.box_id] = replace(box, pose=result.actual_pose, status=BoxStatus.PLACED, stamp_sec=result.stamp_sec)
        self._state = replace(self._state, state_version=self._state.state_version+1, stamp_sec=result.stamp_sec,
                              pallet=replace(self._state.pallet, boxes=self._state.pallet.boxes+(placed,)),
                              inventory=replace(self._state.inventory, tracked_boxes=tracked))
        return self._state
