import math

from pac_common.models import (
    BoxState,
    BoxStatus,
    InventoryState,
    PalletState,
    PlacementCandidate,
    PlacedBox,
    Pose3D,
    RejectCode,
    Size3D,
    SystemState,
)
from pac_planning.physics import PalletPhysicsEngine, PhysicsLimits


def state_with(*boxes: PlacedBox) -> SystemState:
    return SystemState(
        state_version=7,
        stamp_sec=1.0,
        pallet=PalletState("P001", Size3D(1.2, 1.0, 0.15), tuple(boxes)),
        inventory=InventoryState({}, {}),
    )


def incoming(size: Size3D, weight: float = 10.0) -> BoxState:
    return BoxState(
        "BNEW",
        "SKU_A",
        size,
        weight,
        Pose3D("conveyor", 0.0, 0.0, 0.8),
        (0.0, math.pi / 2.0),
        BoxStatus.READY_FOR_PICK,
        1.0,
        1.0,
        "fixture",
    )


def candidate(x: float, y: float, z: float, yaw: float = 0.0) -> PlacementCandidate:
    return PlacementCandidate(
        "S0007-BNEW-C000",
        "BNEW",
        Pose3D("pallet", x, y, z, yaw=yaw),
        7,
    )


def test_first_layer_candidate_is_supported_and_computed():
    engine = PalletPhysicsEngine()
    box = incoming(Size3D(0.4, 0.3, 0.2), 12.0)
    report = engine.evaluate(box, candidate(0.0, 0.0, 0.1), state_with())

    assert report.valid
    assert math.isclose(report.support_ratio, 1.0)
    assert math.isclose(report.total_mass_kg, 12.0)
    assert math.isclose(report.com_x_m, 0.0, abs_tol=1e-12)
    assert math.isclose(report.com_y_m, 0.0, abs_tol=1e-12)
    assert math.isclose(report.com_z_m, 0.1, abs_tol=1e-12)
    assert math.isclose(report.load_balance_score, 1.0, abs_tol=1e-12)


def test_half_supported_upper_box_is_rejected_at_80_percent_threshold():
    lower = PlacedBox(
        "BLOW",
        "SKU_L",
        Size3D(0.4, 0.4, 0.2),
        10.0,
        Pose3D("pallet", 0.0, 0.0, 0.1),
    )
    box = incoming(Size3D(0.4, 0.4, 0.2), 5.0)
    engine = PalletPhysicsEngine(PhysicsLimits(min_support_ratio=0.80))

    # Upper footprint spans x=[0.0, 0.4], while lower spans [-0.2, 0.2].
    # Exactly half of the upper footprint is supported.
    report = engine.evaluate(box, candidate(0.2, 0.0, 0.3), state_with(lower))

    assert math.isclose(report.support_ratio, 0.5, rel_tol=1e-9)
    assert RejectCode.LOW_SUPPORT in report.codes
    assert not report.valid


def test_same_height_overlap_is_box_collision():
    existing = PlacedBox(
        "B001",
        "SKU_A",
        Size3D(0.4, 0.4, 0.2),
        10.0,
        Pose3D("pallet", 0.0, 0.0, 0.1),
    )
    box = incoming(Size3D(0.4, 0.4, 0.2), 8.0)
    report = PalletPhysicsEngine().evaluate(
        box,
        candidate(0.1, 0.0, 0.1),
        state_with(existing),
    )

    assert RejectCode.BOX_COLLISION in report.codes
    assert not report.valid


def test_out_of_bound_candidate_is_rejected():
    box = incoming(Size3D(0.4, 0.3, 0.2), 8.0)
    # Pallet x range is [-0.6, 0.6]; this box reaches x=0.75.
    report = PalletPhysicsEngine().evaluate(
        box,
        candidate(0.55, 0.0, 0.1),
        state_with(),
    )

    assert RejectCode.OUT_OF_BOUND in report.codes
    assert not report.valid


def test_four_quadrants_are_balanced():
    boxes = (
        PlacedBox("B1", "S", Size3D(0.2, 0.2, 0.2), 10.0, Pose3D("pallet", 0.3, 0.25, 0.1)),
        PlacedBox("B2", "S", Size3D(0.2, 0.2, 0.2), 10.0, Pose3D("pallet", -0.3, 0.25, 0.1)),
        PlacedBox("B3", "S", Size3D(0.2, 0.2, 0.2), 10.0, Pose3D("pallet", -0.3, -0.25, 0.1)),
    )
    box = incoming(Size3D(0.2, 0.2, 0.2), 10.0)
    report = PalletPhysicsEngine().evaluate(
        box,
        candidate(0.3, -0.25, 0.1),
        state_with(*boxes),
    )

    assert report.valid
    assert all(math.isclose(v, 10.0, rel_tol=1e-9) for v in report.quadrant_loads_kg)
    assert math.isclose(report.load_balance_score, 1.0, abs_tol=1e-12)
