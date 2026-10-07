from dataclasses import dataclass
from typing import Protocol
from pac_common.models import BoxState, PlacementCandidate, SystemState, ValidationResult
@dataclass(frozen=True)
class RobotCapability:
    robot_model: str; dof: int; nominal_payload_kg: float|None; nominal_reach_m: float|None
    tool_mass_kg: float|None=None; tool_com_m: tuple[float,float,float]|None=None
class RobotFeasibilityAdapter(Protocol):
    def validate_robot_motion(self, box: BoxState, candidate: PlacementCandidate,
                              state: SystemState, robot_state: object) -> ValidationResult: ...
