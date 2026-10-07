#!/usr/bin/env python3
import math

from pac_common.models import (
    BoxState,
    BoxStatus,
    InventoryState,
    PalletState,
    PlacementCandidate,
    PlacedBox,
    Pose3D,
    Size3D,
    SystemState,
)

from pac_planning.physics import PalletPhysicsEngine


engine = PalletPhysicsEngine()


def make_state(version, placed):
    return SystemState(
        state_version=version,
        stamp_sec=float(version),
        pallet=PalletState(
            pallet_id="P001",
            size=Size3D(1.2, 1.0, 0.15),
            boxes=tuple(placed),
        ),
        inventory=InventoryState({}, {}),
    )


def make_box(box_id, mass_kg, size):
    return BoxState(
        box_id=box_id,
        sku_id="DEMO",
        size=size,
        weight_kg=mass_kg,
        pose=Pose3D("conveyor", 0.0, 0.0, 0.8),
        allowed_yaws_rad=(0.0, math.pi / 2.0),
        status=BoxStatus.READY_FOR_PICK,
        confidence=1.0,
        stamp_sec=0.0,
        source="demo_generator",
    )


def evaluate(box, xyz, state):
    candidate = PlacementCandidate(
        candidate_id=f"S{state.state_version:04d}-{box.box_id}",
        box_id=box.box_id,
        target_pose=Pose3D(
            "pallet",
            xyz[0],
            xyz[1],
            xyz[2],
            yaw=0.0,
        ),
        base_state_version=state.state_version,
    )

    report = engine.evaluate(box, candidate, state)

    print(f"\n===== {box.box_id} =====")
    print(f"target              : {xyz}")
    print(f"valid               : {report.valid}")
    print(f"reject codes        : {[c.value for c in report.codes]}")
    print(f"support ratio       : {report.support_ratio:.3f}")
    print(f"total mass [kg]     : {report.total_mass_kg:.2f}")
    print(
        "combined CoM [m]    : "
        f"({report.com_x_m:+.3f}, "
        f"{report.com_y_m:+.3f}, "
        f"{report.com_z_m:+.3f})"
    )
    print(f"CoM XY offset [m]   : {report.com_offset_m:.3f}")
    print(
        "quadrant loads [kg]: "
        + ", ".join(f"{v:.2f}" for v in report.quadrant_loads_kg)
    )
    print(f"load balance score  : {report.load_balance_score:.3f}")

    return candidate, report


placed = []
state = make_state(1, placed)

# 1층 4개
sequence = [
    ("B001", 12.0, (-0.30, +0.25, 0.10)),
    ("B002",  8.0, (+0.30, +0.25, 0.10)),
    ("B003", 10.0, (-0.30, -0.25, 0.10)),
    ("B004",  6.0, (+0.30, -0.25, 0.10)),
]

base_size = Size3D(0.40, 0.30, 0.20)

for box_id, mass, xyz in sequence:
    box = make_box(box_id, mass, base_size)
    candidate, report = evaluate(box, xyz, state)

    if report.valid:
        placed.append(
            PlacedBox(
                box_id=box.box_id,
                sku_id=box.sku_id,
                size=box.size,
                weight_kg=box.weight_kg,
                pose=candidate.target_pose,
            )
        )
        state = make_state(state.state_version + 1, placed)


# --------------------------------------------------
# 잘못된 2층 후보:
# B001과 B002 사이에 걸치지만 지지율이 약 2/3뿐
# --------------------------------------------------
bad_box = make_box(
    "B005",
    5.0,
    Size3D(0.60, 0.30, 0.20),
)

_, bad_report = evaluate(
    bad_box,
    (0.00, +0.25, 0.30),
    state,
)

print("\nExpected: B005 bridge candidate should be rejected")
print(f"Actual  : valid={bad_report.valid}")


# --------------------------------------------------
# 정상적인 2층 후보:
# B001 바로 위에 완전히 적재
# --------------------------------------------------
good_box = make_box(
    "B005",
    5.0,
    Size3D(0.40, 0.30, 0.20),
)

candidate, good_report = evaluate(
    good_box,
    (-0.30, +0.25, 0.30),
    state,
)

if good_report.valid:
    placed.append(
        PlacedBox(
            box_id=good_box.box_id,
            sku_id=good_box.sku_id,
            size=good_box.size,
            weight_kg=good_box.weight_kg,
            pose=candidate.target_pose,
        )
    )

print("\n===== FINAL =====")
print(f"placed boxes: {len(placed)}")
print("IDs:", [b.box_id for b in placed])
