"""Stage-6 settings (``config/taehyeon/robot_check.yaml``).

Values that come from an official source say so; everything else is a
stated assumption to be replaced when the real cell / gripper is known.
"""

from dataclasses import dataclass, field, fields, is_dataclass, replace
from pathlib import Path

import yaml


@dataclass(frozen=True)
class GripperConfig:
    # V4.4 suction cup (pac_bringup/urdf/hdr50_pedestal_gripper.urdf.xacro):
    # cup radius 0.04 m, suction face 0.06 m from the flange. The EOAT is not
    # selected yet; mass is an ASSUMPTION (conservative, the cup itself is 0.4 kg)
    footprint_m: tuple = (0.08, 0.08)
    tcp_offset_m: float = 0.06
    mass_kg: float = 15.0
    clearance_m: float = 0.01      # side margin of the gripper body during descent


@dataclass(frozen=True)
class CellConfig:
    # Team workcell (config/workcell.yaml, Gazebo V4.2-V4.5): robot base_link
    # at world (0, 0, 0.40) on the 0.40 m pedestal, pallet deck centre at
    # world (0, 1.20, 0.15). Robot base relative to the deck centre
    # (x, y, z, yaw), world-aligned axes. tests/test_config_consistency.py
    # checks these against workcell.yaml and the pedestal URDF.
    base_from_pallet_center: tuple = (0.0, -1.20, 0.25, 0.0)
    deck_height_m: float = 0.15
    # conveyor surface point under the picked box, relative to the robot base:
    # V4.3 pick stop (-1.08, 1.20) on the roller top 0.895 m
    pick_point_base_m: tuple = (-1.08, 1.20, 0.495)


@dataclass(frozen=True)
class MotionConfig:
    approach_clearance_m: float = 0.30   # vertical approach / retreat above the target
    path_step_m: float = 0.05            # IK / collision sampling along the vertical path
    max_joint_jump_rad: float = 0.35     # between consecutive path samples
    limit_margin_rad: float = 0.02
    speed_scale: float = 0.5             # fraction of URDF joint velocity limits
    linear_speed_mps: float = 0.25       # vertical approach / retreat speed
    grip_time_s: float = 0.5
    release_time_s: float = 0.5


@dataclass(frozen=True)
class CollisionConfig:
    # capsule radii around the link centre lines (ASSUMPTION, from the mesh
    # bounding sizes, to be replaced by mesh collision in MoveIt)
    upper_arm_radius_m: float = 0.16
    forearm_radius_m: float = 0.13
    wrist_radius_m: float = 0.10
    sample_step_m: float = 0.04


@dataclass(frozen=True)
class RobotCheckConfig:
    robot_model: str = "hdr50_22"
    rated_payload_kg: float = 50.0       # HDR50-22 (HH050) nominal payload
    gripper: GripperConfig = field(default_factory=GripperConfig)
    cell: CellConfig = field(default_factory=CellConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    collision: CollisionConfig = field(default_factory=CollisionConfig)


def _build(cls, data):
    kwargs = {}
    for f in fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        default = f.default_factory() if callable(f.default_factory) else f.default
        if is_dataclass(default):
            value = _build(type(default), value or {})
        elif isinstance(default, tuple):
            value = tuple(value)
        kwargs[f.name] = value
    unknown = set(data) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}")
    return cls(**kwargs)


def config_from_dict(data):
    cfg = _build(RobotCheckConfig, (data or {}).get("robot_check", data or {}))
    if cfg.robot_model != "hdr50_22":
        raise ValueError("only hdr50_22 kinematics are available")
    if cfg.rated_payload_kg <= 0 or cfg.gripper.mass_kg < 0:
        raise ValueError("payload / gripper mass must be positive")
    if cfg.motion.path_step_m <= 0 or cfg.collision.sample_step_m <= 0:
        raise ValueError("sampling steps must be positive")
    return cfg


def load_robot_check_config(path):
    return config_from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


__all__ = ["RobotCheckConfig", "GripperConfig", "CellConfig", "MotionConfig", "CollisionConfig",
           "config_from_dict", "load_robot_check_config", "replace"]
