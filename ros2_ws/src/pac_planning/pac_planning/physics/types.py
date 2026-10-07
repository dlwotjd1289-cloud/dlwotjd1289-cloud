from dataclasses import dataclass

from pac_common.models import RejectCode


@dataclass(frozen=True)
class SupportContact:
    """Geometric support contributed by one lower body."""

    supporter_id: str
    area_m2: float
    load_share: float


@dataclass(frozen=True)
class PalletPhysicsReport:
    """Fast deterministic pallet-evaluation output used by AHEAD.

    V1 deliberately separates analytic planning metrics from Gazebo dynamics.
    `quadrant_loads_kg` is a projected mass-balance heuristic, not a contact-force
    measurement or a structural stress result.
    """

    valid: bool
    codes: tuple[RejectCode, ...]

    support_ratio: float
    support_contacts: tuple[SupportContact, ...]

    total_mass_kg: float
    com_x_m: float
    com_y_m: float
    com_z_m: float
    com_offset_m: float
    com_inside_pallet: bool

    quadrant_loads_kg: tuple[float, float, float, float]
    load_balance_score: float
