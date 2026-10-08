from dataclasses import dataclass

from pac_common.models import (
    BoxState,
    PlacementCandidate,
    RejectCode,
    SystemState,
    ValidationResult,
)

from .capability import RobotCapability
from .palletizer_command import PalletizerCommand


@dataclass(frozen=True)
class PalletizerEmulationPolicy:
    lock_roll_rad: float = 0.0
    lock_pitch_rad: float = 0.0
    vertical_approach: bool = True
    vertical_retreat: bool = True


class Hdr50_22SimAdapter:
    """Simulation proxy adapter for the final HDP160-31 target.

    HDR50-22 is a 6-DOF robot. This adapter deliberately exposes only the
    palletizer command space x/y/z/yaw to AHEAD. The MoveIt backend is injected
    later, so this module remains testable without ROS.
    """

    capability = RobotCapability(
        robot_model="hdr50_22",
        dof=6,
        nominal_payload_kg=50.0,
        nominal_reach_m=2.239,
        tool_mass_kg=None,
        tool_com_m=None,
    )

    def __init__(self, backend=None, policy: PalletizerEmulationPolicy | None = None):
        self._backend = backend
        self.policy = policy or PalletizerEmulationPolicy()

    def command_from_candidate(self, candidate: PlacementCandidate) -> PalletizerCommand:
        pose = candidate.target_pose
        return PalletizerCommand(
            frame_id=pose.frame_id,
            x=pose.x,
            y=pose.y,
            z=pose.z,
            yaw=pose.yaw,
        )

    def validate_robot_motion(
        self,
        box: BoxState,
        candidate: PlacementCandidate,
        state: SystemState,
        robot_state: object,
    ) -> ValidationResult:
        if candidate.base_state_version != state.state_version:
            return ValidationResult(
                success=False,
                codes=(RejectCode.STALE_PLAN,),
                details={
                    "candidate_version": candidate.base_state_version,
                    "state_version": state.state_version,
                    "adapter": "hdr50_22_sim_proxy",
                },
            )

        if self._backend is None:
            return ValidationResult(
                success=False,
                codes=(RejectCode.INVALID_STATE,),
                details={
                    "reason": "hdr50_22_moveit_backend_not_loaded",
                    "adapter": "hdr50_22_sim_proxy",
                    "proxy_is_final_target": False,
                },
            )

        return self._backend.validate_robot_motion(
            box=box,
            candidate=candidate,
            state=state,
            robot_state=robot_state,
            palletizer_command=self.command_from_candidate(candidate),
            lock_roll_rad=self.policy.lock_roll_rad,
            lock_pitch_rad=self.policy.lock_pitch_rad,
            vertical_approach=self.policy.vertical_approach,
            vertical_retreat=self.policy.vertical_retreat,
        )
