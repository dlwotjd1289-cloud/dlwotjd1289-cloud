import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import hdr50_kinematics as K
import mission_bridge_v45 as MB
from pick_place_plan_v45 import plan_pick_place_yaw

BOX = (0.40, 0.30, 0.25)
PICK = (-1.06, 1.20, 1.02)
P = MB.GazeboPallet()


def corner(x, y, z=0.0, yaw=0.0):
    return {"frame_id": "pallet", "x": x, "y": y, "z": z, "roll": 0.0, "pitch": 0.0, "yaw": yaw}


class TestFrames(unittest.TestCase):
    def test_origin_corner_maps_to_pallet_corner(self):
        x, y, z, yaw = MB.candidate_to_world(corner(0.0, 0.0), BOX, P)
        self.assertAlmostEqual(x, -0.55 + 0.20)
        self.assertAlmostEqual(y, 1.20 - 0.55 + 0.15)
        self.assertAlmostEqual(z, 0.15 + 0.125)
        self.assertEqual(yaw, 0.0)

    def test_yaw_swaps_footprint(self):
        x, y, _, yaw = MB.candidate_to_world(corner(0.0, 0.0, yaw=math.pi / 2), BOX, P)
        self.assertAlmostEqual(x, -0.55 + 0.15)
        self.assertAlmostEqual(y, 1.20 - 0.55 + 0.20)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_round_trip(self):
        for c in (corner(0.002, 0.002), corner(0.35, 0.4, 0.25, math.pi / 2), corner(0.698, 0.0)):
            x, y, z, yaw = MB.candidate_to_world(c, BOX, P)
            back = MB.world_to_pallet_corner((x, y, z), yaw + 0.01, BOX, P)  # small measured yaw noise
            for k in ("x", "y", "z", "yaw"):
                self.assertAlmostEqual(back[k], c[k], places=9, msg=k)

    def test_rejects_wrong_frame_and_tilt(self):
        with self.assertRaises(ValueError):
            MB.candidate_to_world(dict(corner(0, 0), frame_id="world"), BOX, P)
        with self.assertRaises(ValueError):
            MB.candidate_to_world(dict(corner(0, 0), roll=0.1), BOX, P)
        with self.assertRaises(ValueError):
            MB.candidate_to_world(corner(0, 0, yaw=0.3), BOX, P)


class TestBridge(unittest.TestCase):
    def test_parse_scale_pass(self):
        self.assertEqual(MB.parse_scale_pass("...\nV4.3 PASS: 5.012 kg; empty-scale restored"), 5.012)
        with self.assertRaises(ValueError):
            MB.parse_scale_pass("V4.3 FAIL: pose timeout")

    def test_request_uses_measured_mass_and_nominal_catalog(self):
        state, ctx = MB.build_planning_request(box_id="B001", sku_id="S", size_m=BOX, measured_kg=5.04,
                                               pick_center_world=PICK, state_version=3, stamp_sec=1.0,
                                               nominal_kg=5.0, top_load_capacity_n=900.0,
                                               pallet_max_weight_kg=1000.0)
        box = state["inventory"]["tracked_boxes"]["B001"]
        self.assertEqual(box["weight_kg"], 5.04)
        self.assertEqual(box["status"], "READY_FOR_PICK")
        self.assertEqual(ctx["catalog"]["S"]["weight_kg"], 5.0)
        self.assertEqual(state["state_version"], 3)
        self.assertEqual(state["pallet"]["size"], {"x": 1.10, "y": 1.10, "z": 1.35})

    def test_yaw_delta_short_turn(self):
        self.assertAlmostEqual(MB.gripper_yaw_delta(math.pi / 2, 0.0), math.pi / 2)
        self.assertAlmostEqual(MB.gripper_yaw_delta(0.0, math.pi), 0.0)          # 180 deg symmetric
        self.assertAlmostEqual(MB.gripper_yaw_delta(math.pi / 2, math.pi), -math.pi / 2)
        self.assertAlmostEqual(MB.gripper_yaw_delta(0.1, -0.1), 0.2)


class TestYawPlan(unittest.TestCase):
    def test_turn_segment_rotates_tool_by_delta(self):
        place = MB.candidate_to_world(corner(0.0, 0.0, yaw=math.pi / 2), BOX, P)
        segs = plan_pick_place_yaw(K.HOME, PICK, place[:3], BOX, math.pi / 2)
        names = [s.name for s in segs]
        self.assertIn("turn", names)
        self.assertLess(names.index("transfer"), names.index("turn"))
        self.assertLess(names.index("turn"), names.index("descend_place"))
        release = next(s for s in segs if s.gripper_after == "detach").points[-1]
        p, R = K.fk(release)
        np.testing.assert_allclose(p[:2], place[:2], atol=2e-3)
        np.testing.assert_allclose(R[:, 1], [0.0, 1.0, 0.0], atol=1e-2)   # flange y along world yaw 90 deg

    def test_no_turn_matches_v44_segments(self):
        segs = plan_pick_place_yaw(K.HOME, PICK, (0.0, 1.2, 0.275), BOX, 0.0)
        self.assertNotIn("turn", [s.name for s in segs])

    def test_whole_first_layer_reachable(self):
        for x in np.linspace(-0.35, 0.35, 3):
            for y in np.linspace(0.80, 1.60, 3):
                for yaw in (0.0, math.pi / 2):
                    plan_pick_place_yaw(K.HOME, PICK, (x, y, 0.275), BOX, yaw)


if __name__ == "__main__":
    unittest.main()
