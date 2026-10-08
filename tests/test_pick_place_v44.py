import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import hdr50_kinematics as K
from pick_place_plan_v44 import PlanConfig, plan_pick_place

BOX = (0.40, 0.30, 0.25)
PICK = (-1.06, 1.20, 1.02)    # V4.3 DONE position (box center)
PLACE = (0.0, 1.20, 0.275)    # pallet_main center, deck top z=0.15


class TestKinematics(unittest.TestCase):
    def test_home_fk_matches_urdf(self):
        p, R = K.fk(K.HOME)
        np.testing.assert_allclose(p, [1.535, 0.0, 1.95], atol=1e-3)
        np.testing.assert_allclose(R[:, 0], [1.0, 0.0, 0.0], atol=1e-3)

    def test_ik_roundtrip(self):
        R = K.tool_down_rotation(0.0)
        for target in ((-1.06, 1.2, 1.15), (0.0, 1.2, 0.41), (0.5, -1.0, 0.9)):
            q = K.ik(target, R, [np.arctan2(target[1], target[0]), 1.2, 0, 0, -1.0, 0])
            self.assertIsNotNone(q)
            p, Rq = K.fk(q)
            np.testing.assert_allclose(p, target, atol=1e-3)
            np.testing.assert_allclose(Rq[:, 0], [0, 0, -1], atol=1e-2)

    def test_unreachable_returns_none(self):
        self.assertIsNone(K.ik((5.0, 0.0, 1.0), K.tool_down_rotation(), K.HOME))


class TestPickPlacePlan(unittest.TestCase):
    def setUp(self):
        self.segs = plan_pick_place(K.HOME, PICK, PLACE, BOX)

    def test_segment_order_and_gripper(self):
        names = [s.name for s in self.segs]
        self.assertEqual(names, ["to_pick_transfer", "descend_pick", "lift", "transfer",
                                 "descend_place", "retreat", "home"])
        grips = {s.name: s.gripper_after for s in self.segs if s.gripper_after}
        self.assertEqual(grips, {"descend_pick": "attach", "descend_place": "detach"})

    def test_grip_and_release_poses(self):
        cfg = PlanConfig()
        by = {s.name: s for s in self.segs}
        p, R = K.fk(by["descend_pick"].points[-1])
        np.testing.assert_allclose(p, [PICK[0], PICK[1], PICK[2] + BOX[2] / 2 + cfg.grip_gap_m], atol=1e-3)
        np.testing.assert_allclose(R[:, 0], [0, 0, -1], atol=1e-2)
        p, _ = K.fk(by["descend_place"].points[-1])
        np.testing.assert_allclose(
            p, [PLACE[0], PLACE[1], PLACE[2] + BOX[2] / 2 + cfg.grip_gap_m + cfg.place_drop_m], atol=1e-3)

    def test_transfer_keeps_box_above_stopper(self):
        # Pick stopper top z=1.05; box bottom = flange z - gap - box height.
        for s in self.segs[2:4]:
            for q in s.points[1:]:
                p, _ = K.fk(q)
                self.assertGreater(p[2] - PlanConfig().grip_gap_m - BOX[2], 1.05 - 0.15 if s.name == "lift" else 1.05)

    def test_continuous_within_limits_and_speed(self):
        q_prev = np.array(K.HOME)
        for s in self.segs:
            self.assertGreaterEqual(s.duration_s, 1.0)
            pts = np.vstack(s.points)
            self.assertTrue(np.all(pts >= K.LOWER - 1e-9) and np.all(pts <= K.UPPER + 1e-9))
            steps = np.abs(np.diff(np.vstack([q_prev, pts]), axis=0))
            self.assertLess(steps.max(), 0.36)
            dt = s.duration_s / len(s.points)
            self.assertTrue(np.all(steps / dt <= K.VEL_LIMIT * 0.5 + 1e-9))
            q_prev = pts[-1]

    def test_unreachable_place_raises(self):
        with self.assertRaises(ValueError):
            plan_pick_place(K.HOME, PICK, (4.0, 1.2, 0.275), BOX)


if __name__ == "__main__":
    unittest.main()
