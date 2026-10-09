"""V4.4 Gazebo cycle bridge: team stages 5/6 for plan, StateManager rule for commit."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "scripts" / "ahead_planner_bridge_v44.py"


def run(*args):
    out = subprocess.run([sys.executable, str(BRIDGE), *map(str, args)], capture_output=True, text=True, check=True)
    return out.stdout.strip()


@pytest.fixture
def cell(tmp_path):
    per = tmp_path / "per.json"
    per.write_text(json.dumps({"status": "OK", "length": 0.40, "width": 0.30, "x": -1.03, "y": 1.20, "yaw": 0.0}))
    return tmp_path, per


def test_plan_fills_the_far_corner_and_commit_keeps_the_plan(cell):
    tmp, per = cell
    state = tmp / "s.json"
    x, y, z, yaw = map(float, run("plan", "--state", state, "--box-id", "b1", "--mass", 5, "--perception", per,
                                  "--remaining", 2, "--out", tmp / "p1.json").split())
    assert y > 1.2 and x > 0.0             # deck corner farthest from the robot first
    plan = json.loads((tmp / "p1.json").read_text())
    assert plan["robot"]["q_place"] and plan["stack_limit_m"] == 1.5
    run("commit", "--state", state, "--box-id", "b1", "--mass", 5, "--world", x, y, z + 0.002, yaw,
        "--planned", tmp / "p1.json")
    placed = json.loads(state.read_text())["placed"]
    assert placed[0]["pose"]["x"] == pytest.approx(plan["target_corner_pallet"][0])
    assert placed[0]["size"]["x"] == pytest.approx(0.42)     # planner size (clearance) kept on plan


def test_commit_outside_tolerance_records_the_measured_true_box(cell):
    tmp, per = cell
    state = tmp / "s.json"
    x, y, z, yaw = map(float, run("plan", "--state", state, "--box-id", "b1", "--mass", 5, "--perception", per,
                                  "--out", tmp / "p1.json").split())
    run("commit", "--state", state, "--box-id", "b1", "--mass", 5, "--world", x - 0.02, y, z, yaw,
        "--planned", tmp / "p1.json")
    placed = json.loads(state.read_text())["placed"]
    assert placed[0]["size"]["x"] == pytest.approx(0.40)
    plan = json.loads((tmp / "p1.json").read_text())
    assert placed[0]["pose"]["x"] != pytest.approx(plan["target_corner_pallet"][0], abs=1e-3)
