"""Locate the team packages of this monorepo for scripts run without colcon.

Resolution order for each package:
1. explicit environment variable (e.g. ``PAC_COMMON_SRC``),
2. the monorepo location (``ros2_ws/src/<pkg>``, ``tools/...``).
"""

from pathlib import Path
import os
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]

_KEYS = {
    "pac_common": ("PAC_COMMON_SRC", "ros2_ws/src/pac_common"),
    "pac_planning": ("PAC_PLANNING_SRC", "ros2_ws/src/pac_planning"),
    "pac_simulation": ("PAC_SIMULATION_SRC", "ros2_ws/src/pac_simulation"),
    "generator": ("AHEAD_GENERATOR_ROOT", "tools/ahead_dataset_generator"),
    "pac_robot_check": ("PAC_ROBOT_CHECK_SRC", "ros2_ws/src/pac_robot_check"),
    "pac_runtime": ("PAC_RUNTIME_SRC", "ros2_ws/src/pac_runtime"),
}


def locate(name):
    """Return the directory for ``name`` or ``None`` when unavailable."""
    env_key, relative = _KEYS[name]
    if os.environ.get(env_key):
        path = Path(os.environ[env_key])
        return path if path.exists() else None
    direct = REPO_ROOT / relative
    return direct if direct.exists() else None


def own_source_dirs():
    return [
        REPO_ROOT / "ros2_ws" / "src" / "pac_candidates",
        REPO_ROOT / "ros2_ws" / "src" / "pac_highlevel",
        REPO_ROOT / "ros2_ws" / "src" / "pac_robot_check",
        REPO_ROOT / "ros2_ws" / "src" / "pac_runtime",
        REPO_ROOT / "tools" / "virtual_data",
    ]


def bootstrap(require_common=True):
    """Put own and teammates' packages on ``sys.path``."""
    paths = [str(p) for p in own_source_dirs()]
    common = locate("pac_common")
    if common is None and require_common:
        raise RuntimeError(
            "pac_common not found in ros2_ws/src; set PAC_COMMON_SRC."
        )
    if common is not None:
        paths.append(str(common))
    # The generator ships its own minimal ``pac_common`` copy; it is run as a
    # subprocess and never imported here, so it cannot shadow the team one.
    for path in reversed(paths):
        if path not in sys.path:
            sys.path.insert(0, path)
    return common


def add_optional(name):
    """Add an optional teammate package (planner/simulator); return path."""
    path = locate(name)
    if path is not None and str(path) not in sys.path:
        sys.path.append(str(path))
    return path
