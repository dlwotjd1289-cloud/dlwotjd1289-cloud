#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import math

import plotly.graph_objects as go


@dataclass(frozen=True)
class VisualBox:
    box_id: str
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float
    mass_kg: float
    yaw: float = 0.0


def cuboid_vertices(box: VisualBox):
    hx = box.sx / 2.0
    hy = box.sy / 2.0
    hz = box.sz / 2.0

    local = [
        (-hx, -hy, -hz),
        (+hx, -hy, -hz),
        (+hx, +hy, -hz),
        (-hx, +hy, -hz),
        (-hx, -hy, +hz),
        (+hx, -hy, +hz),
        (+hx, +hy, +hz),
        (-hx, +hy, +hz),
    ]

    c = math.cos(box.yaw)
    s = math.sin(box.yaw)

    world = []
    for lx, ly, lz in local:
        rx = c * lx - s * ly
        ry = s * lx + c * ly
        world.append(
            (
                box.x + rx,
                box.y + ry,
                box.z + lz,
            )
        )

    return world


def add_box(fig, box: VisualBox, opacity=0.90, name=None):
    v = cuboid_vertices(box)

    x = [p[0] for p in v]
    y = [p[1] for p in v]
    z = [p[2] for p in v]

    # cuboid 12 triangles
    i = [0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3]
    j = [1, 2, 5, 6, 1, 5, 2, 6, 3, 7, 0, 4]
    k = [2, 3, 6, 7, 5, 4, 6, 5, 7, 6, 4, 7]

    hover = (
        f"<b>{box.box_id}</b><br>"
        f"mass = {box.mass_kg:.1f} kg<br>"
        f"center = ({box.x:.2f}, {box.y:.2f}, {box.z:.2f}) m<br>"
        f"size = ({box.sx:.2f}, {box.sy:.2f}, {box.sz:.2f}) m<br>"
        f"yaw = {math.degrees(box.yaw):.0f}°"
        "<extra></extra>"
    )

    fig.add_trace(
        go.Mesh3d(
            x=x,
            y=y,
            z=z,
            i=i,
            j=j,
            k=k,
            opacity=opacity,
            flatshading=True,
            name=name or box.box_id,
            hovertemplate=hover,
        )
    )


def combined_com(boxes):
    total_mass = sum(b.mass_kg for b in boxes)

    return (
        sum(b.mass_kg * b.x for b in boxes) / total_mass,
        sum(b.mass_kg * b.y for b in boxes) / total_mass,
        sum(b.mass_kg * b.z for b in boxes) / total_mass,
    )


def main():
    fig = go.Figure()

    # --------------------------------------------------
    # Pallet
    # 좌표 규칙:
    # x/y origin = pallet top surface center
    # z = 0       = pallet top surface
    # --------------------------------------------------
    pallet = VisualBox(
        box_id="PALLET",
        x=0.0,
        y=0.0,
        z=-0.075,
        sx=1.20,
        sy=1.00,
        sz=0.15,
        mass_kg=0.0,
    )
    add_box(fig, pallet, opacity=0.35, name="Pallet")

    # Pallet Physics V1에서 테스트했던 배치
    boxes = [
        VisualBox("B001", -0.30, +0.25, 0.10, 0.40, 0.30, 0.20, 12.0),
        VisualBox("B002", +0.30, +0.25, 0.10, 0.40, 0.30, 0.20,  8.0),
        VisualBox("B003", -0.30, -0.25, 0.10, 0.40, 0.30, 0.20, 10.0),
        VisualBox("B004", +0.30, -0.25, 0.10, 0.40, 0.30, 0.20,  6.0),

        # 정상 2층 박스
        VisualBox("B005", -0.30, +0.25, 0.30, 0.40, 0.30, 0.20, 5.0),
    ]

    for box in boxes:
        add_box(fig, box)

    com_x, com_y, com_z = combined_com(boxes)

    fig.add_trace(
        go.Scatter3d(
            x=[com_x],
            y=[com_y],
            z=[com_z],
            mode="markers+text",
            marker=dict(size=8),
            text=["Combined CoM"],
            textposition="top center",
            name="Combined CoM",
            hovertemplate=(
                "<b>Combined CoM</b><br>"
                f"x = {com_x:+.3f} m<br>"
                f"y = {com_y:+.3f} m<br>"
                f"z = {com_z:+.3f} m"
                "<extra></extra>"
            ),
        )
    )

    fig.update_layout(
        title="PAC2026 - Pallet 3D Viewer V1",
        scene=dict(
            xaxis_title="Pallet X [m]",
            yaxis_title="Pallet Y [m]",
            zaxis_title="Z [m]",
            aspectmode="data",
            camera=dict(
                eye=dict(x=1.6, y=1.6, z=1.3)
            ),
        ),
        margin=dict(l=0, r=0, b=0, t=50),
    )

    output_dir = Path("artifacts")
    output_dir.mkdir(exist_ok=True)

    output_file = output_dir / "pallet_3d_viewer_v1.html"
    fig.write_html(
        output_file,
        include_plotlyjs=True,
        auto_open=False,
    )

    print("===== Pallet 3D Viewer V1 =====")
    print(f"boxes        : {len(boxes)}")
    print(f"total mass   : {sum(b.mass_kg for b in boxes):.2f} kg")
    print(
        "combined CoM : "
        f"({com_x:+.3f}, {com_y:+.3f}, {com_z:+.3f}) m"
    )
    print(f"output       : {output_file.resolve()}")


if __name__ == "__main__":
    main()
