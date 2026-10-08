from dataclasses import dataclass

from pac_common.models import (
    BoxState,
    PlacementCandidate,
    RejectCode,
    SystemState,
    ValidationResult,
)

from pac_common.frames import corner_to_center

from .capability import RobotCapability
from .palletizer_command import PalletizerCommand


@dataclass(frozen=True)
class PalletizerEmulationPolicy:
    lock_roll_rad: float = 0.0
    lock_pitch_rad: float = 0.0
    vertical_approach: bool = True
    vertical_retreat: bool = True


class Hdr50_22Adapter:
    """Robot feasibility adapter for the target robot HDR50-22 (team decision 2026-10-08).

    HDR50-22 is a 6-DOF robot. AHEAD deliberately commands it in the
    palletizer command space x/y/z/yaw (box kept upright, vertical approach and
    retreat). The MoveIt / Gazebo backend is injected, so this module remains
    testable without ROS; without a backend every candidate is rejected.
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

    def command_from_candidate(self, box: BoxState, candidate: PlacementCandidate) -> PalletizerCommand:
        """Box CENTRE in frame "pallet" (the candidate holds the lower AABB corner).

        pallet -> robot_base TF and the TCP / gripper offset are applied by the
        backend; this command is still a box pose, not a flange pose.
        """
        if box.box_id != candidate.box_id:
            raise ValueError("Candidate is for another box")
        pose = corner_to_center(box.size, candidate.target_pose)
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
                    "adapter": "hdr50_22",
                },
            )

        if self._backend is None:
            return ValidationResult(
                success=False,
                codes=(RejectCode.INVALID_STATE,),
                details={
                    "reason": "hdr50_22_moveit_backend_not_loaded",
                    "adapter": "hdr50_22",
                },
            )

        return self._backend.validate_robot_motion(
            box=box,
            candidate=candidate,
            state=state,
            robot_state=robot_state,
            palletizer_command=self.command_from_candidate(box, candidate),
            lock_roll_rad=self.policy.lock_roll_rad,
            lock_pitch_rad=self.policy.lock_pitch_rad,
            vertical_approach=self.policy.vertical_approach,
            vertical_retreat=self.policy.vertical_retreat,
        )


# Name used before HDR50-22 became the target (it was the HDP160-31 sim proxy).
Hdr50_22SimAdapter = Hdr50_22Adapter

