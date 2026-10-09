"""Box geometry primitives shared by every module (common standard v0.3, 9.1 / 10.1).

PlacementCandidate.target_pose / PlacedBox.pose (frame "pallet") give the
lower x/y/z corner of the box AABB after yaw; z = 0 is the deck top. Robots
and vision work with box centres, so they convert here instead of guessing.
Only upright boxes with yaw a multiple of 90 deg are supported (section 9.1).

Single source: pac_candidates.geometry, pac_planning.geometry,
pac_robot_check and the Gazebo bridges import these instead of keeping
their own copies.
"""

from dataclasses import replace
import math

from .models import Pose3D, Size3D

HALF_PI = math.pi / 2.0
YAW_TOL = 1e-6
_YAW_TOL = YAW_TOL


def quarter_turns(yaw: float) -> int:
    """Number of quarter turns of ``yaw``; ValueError unless a multiple of 90 deg."""
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    quarter = round(yaw / HALF_PI)
    if abs(yaw - quarter * HALF_PI) > YAW_TOL:
        raise ValueError(f"Only axis-aligned yaws (k * pi/2) are supported, got {yaw}")
    return quarter


def rotated_dims(size: Size3D, yaw: float) -> tuple[float, float, float]:
    """AABB extents (dx, dy, dz) of an upright box turned by yaw about z."""
    return (size.y, size.x, size.z) if quarter_turns(yaw) % 2 else (size.x, size.y, size.z)


def _upright(pose: Pose3D):
    if abs(pose.roll) > _YAW_TOL or abs(pose.pitch) > _YAW_TOL:
        raise ValueError("Only upright boxes are supported")


def corner_to_center(size: Size3D, corner: Pose3D) -> Pose3D:
    _upright(corner)
    dx, dy, dz = rotated_dims(size, corner.yaw)
    return replace(corner, x=corner.x + dx / 2, y=corner.y + dy / 2, z=corner.z + dz / 2)


def center_to_corner(size: Size3D, center: Pose3D) -> Pose3D:
    _upright(center)
    dx, dy, dz = rotated_dims(size, center.yaw)
    return replace(center, x=center.x - dx / 2, y=center.y - dy / 2, z=center.z - dz / 2)
