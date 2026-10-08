from dataclasses import dataclass, field
from enum import Enum
from typing import Any

class BoxStatus(str, Enum):
    UNKNOWN='UNKNOWN'; DETECTED='DETECTED'; MEASURED='MEASURED'
    ON_CONVEYOR='ON_CONVEYOR'; READY_FOR_PICK='READY_FOR_PICK'; PICKING='PICKING'
    IN_TRANSIT='IN_TRANSIT'; PLACED='PLACED'; BUFFERED='BUFFERED'
    REJECTED='REJECTED'; FAILED='FAILED'

class RejectCode(str, Enum):
    OUT_OF_BOUND='OUT_OF_BOUND'; HEIGHT_LIMIT='HEIGHT_LIMIT'; BOX_COLLISION='BOX_COLLISION'
    LOW_SUPPORT='LOW_SUPPORT'; LOAD_VIOLATION='LOAD_VIOLATION'; COG_VIOLATION='COG_VIOLATION'
    PAYLOAD_EXCEEDED='PAYLOAD_EXCEEDED'; IK_FAIL='IK_FAIL'; ROBOT_COLLISION='ROBOT_COLLISION'
    APPROACH_FAIL='APPROACH_FAIL'; RETREAT_FAIL='RETREAT_FAIL'; TIMEOUT='TIMEOUT'
    INVALID_STATE='INVALID_STATE'; STALE_PLAN='STALE_PLAN'; SENSOR_UNCERTAIN='SENSOR_UNCERTAIN'
    TRACKING_LOST='TRACKING_LOST'; SIZE_MISMATCH='SIZE_MISMATCH'; WEIGHT_MISMATCH='WEIGHT_MISMATCH'
    EXECUTION_FAIL='EXECUTION_FAIL'; UNKNOWN_ERROR='UNKNOWN_ERROR'

@dataclass(frozen=True)
class Size3D:
    x: float; y: float; z: float

@dataclass(frozen=True)
class Pose3D:
    frame_id: str; x: float; y: float; z: float
    roll: float=0.0; pitch: float=0.0; yaw: float=0.0

@dataclass(frozen=True)
class BoxState:
    box_id: str; sku_id: str; size: Size3D; weight_kg: float; pose: Pose3D
    allowed_yaws_rad: tuple[float, ...]; status: BoxStatus; confidence: float
    stamp_sec: float; source: str

@dataclass(frozen=True)
class PlacedBox:
    box_id: str; sku_id: str; size: Size3D; weight_kg: float; pose: Pose3D

@dataclass(frozen=True)
class PalletState:
    pallet_id: str; size: Size3D; boxes: tuple[PlacedBox, ...]

@dataclass(frozen=True)
class InventoryState:
    tracked_boxes: dict[str, BoxState]
    remaining_by_sku: dict[str, int]

@dataclass(frozen=True)
class SystemState:
    state_version: int; stamp_sec: float; pallet: PalletState; inventory: InventoryState

@dataclass(frozen=True)
class PlacementCandidate:
    candidate_id: str; box_id: str; target_pose: Pose3D; base_state_version: int
    score: float|None=None; score_detail: dict[str,float]=field(default_factory=dict)

@dataclass(frozen=True)
class ValidationResult:
    success: bool; codes: tuple[RejectCode,...]=(); details: dict[str,Any]=field(default_factory=dict)

@dataclass(frozen=True)
class ExecutionResult:
    success: bool; box_id: str; candidate_id: str; actual_pose: Pose3D|None
    codes: tuple[RejectCode,...]; stamp_sec: float
