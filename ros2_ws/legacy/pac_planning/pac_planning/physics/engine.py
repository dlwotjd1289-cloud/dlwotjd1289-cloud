from dataclasses import dataclass

from pac_common.models import BoxState, PlacementCandidate, RejectCode, SystemState
from pac_common.validation import validate_candidate

from .center_of_mass import combined_com, com_xy_metrics
from .geometry import candidate_footprint, collides_with_existing, inside_pallet
from .load_balance import projected_quadrant_loads
from .support import evaluate_support
from .types import PalletPhysicsReport


@dataclass(frozen=True)
class PhysicsLimits:
    # Engineering parameter for V1; tune / validate rather than treating it as
    # an official competition threshold.
    min_support_ratio: float = 0.80
    geometric_tolerance_m: float = 1e-6
    support_z_tolerance_m: float = 1e-4

    def __post_init__(self):
        if not 0.0 <= self.min_support_ratio <= 1.0:
            raise ValueError("min_support_ratio must be in [0, 1]")
        if self.geometric_tolerance_m <= 0.0 or self.support_z_tolerance_m <= 0.0:
            raise ValueError("tolerances must be > 0")


class PalletPhysicsEngine:
    """Fast analytic candidate evaluator used before robot feasibility checks."""

    def __init__(self, limits: PhysicsLimits | None = None):
        self.limits = limits or PhysicsLimits()

    def evaluate(
        self,
        box: BoxState,
        candidate: PlacementCandidate,
        state: SystemState,
    ) -> PalletPhysicsReport:
        validate_candidate(candidate, state)
        if candidate.box_id != box.box_id:
            raise ValueError("candidate.box_id must match box.box_id")

        pose = candidate.target_pose
        pallet = state.pallet
        codes: list[RejectCode] = []

        boundary_ok = inside_pallet(
            candidate_poly=candidate_footprint(box.size, candidate),
            pallet=pallet,
            tol_m=self.limits.geometric_tolerance_m,
        )
        if not boundary_ok:
            codes.append(RejectCode.OUT_OF_BOUND)

        if collides_with_existing(
            box.size,
            pose,
            pallet,
            self.limits.geometric_tolerance_m,
        ):
            codes.append(RejectCode.BOX_COLLISION)

        support_ratio, contacts = evaluate_support(
            box.size,
            pose,
            pallet,
            self.limits.support_z_tolerance_m,
        )
        if support_ratio + self.limits.geometric_tolerance_m < self.limits.min_support_ratio:
            codes.append(RejectCode.LOW_SUPPORT)

        total_mass, com_x, com_y, com_z = combined_com(
            pallet,
            box.size,
            box.weight_kg,
            pose,
        )
        com_offset, com_inside = com_xy_metrics(pallet, com_x, com_y)
        if not com_inside:
            codes.append(RejectCode.COG_VIOLATION)

        quadrant_loads, load_balance_score = projected_quadrant_loads(
            pallet,
            box.size,
            box.weight_kg,
            pose,
        )

        # Preserve deterministic code order while avoiding duplicates.
        unique_codes = tuple(dict.fromkeys(codes))
        return PalletPhysicsReport(
            valid=not unique_codes,
            codes=unique_codes,
            support_ratio=support_ratio,
            support_contacts=contacts,
            total_mass_kg=total_mass,
            com_x_m=com_x,
            com_y_m=com_y,
            com_z_m=com_z,
            com_offset_m=com_offset,
            com_inside_pallet=com_inside,
            quadrant_loads_kg=quadrant_loads,
            load_balance_score=load_balance_score,
        )
