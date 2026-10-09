import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scale_cycle_core_v43 import AutoScaleCycle, Config, G, Phase, config_for_box


class TestAutoScaleCycle(unittest.TestCase):
    def make_cycle(self):
        cycle = AutoScaleCycle(Config(), now=0.0)
        # 7 kg empty weighing apparatus, force-based tare (not fake mass).
        for i in range(50):
            t = 0.01 * i
            cycle.update_wrench(7 * G, t)
            cycle.tick(t)
        self.assertEqual(cycle.phase, Phase.WAIT_BOX)
        return cycle

    def test_full_cycle(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, 0.5)
        self.assertAlmostEqual(c.tick(0.5), 8.57)
        self.assertEqual(c.phase, Phase.TO_SCALE)
        c.update_pose(-3.86, 1.2, 0.9)
        self.assertEqual(c.tick(0.9), 0.0)
        self.assertEqual(c.phase, Phase.SETTLING)
        for i in range(100):
            t = 0.9 + 0.01 * (i + 1)
            c.update_wrench(12 * G, t)
            if i % 10 == 0: c.update_pose(-3.83, 1.2, t)
            speed = c.tick(t)
        self.assertEqual(c.phase, Phase.TO_PICK)
        self.assertAlmostEqual(c.measured_kg, 5.0, places=6)
        self.assertEqual(speed, 8.57)
        for i in range(30):
            t += .01
            c.update_wrench(7 * G, t)
            c.update_pose(-3.0, 1.2, t)
            c.tick(t)
        self.assertTrue(c.unloaded)
        c.update_pose(-1.07, 1.2, t + 0.1)
        self.assertEqual(c.tick(t + 0.1), 0.0)
        self.assertEqual(c.phase, Phase.DONE)

    def test_no_load_on_scale_fails_closed(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, .6)
        c.tick(.6)
        c.update_pose(-3.86, 1.2, 1.0)
        c.tick(1.0)
        for i in range(1501):
            t = 1.01 + i * .01
            c.update_wrench(7 * G, t)
            if i % 20 == 0: c.update_pose(-3.8, 1.2, t)
            c.tick(t)
        self.assertEqual(c.phase, Phase.ERROR)
        self.assertEqual(c.tick(30), 0)

    def test_wrong_mass_fails_closed(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, .6)
        c.tick(.6)
        c.update_pose(-3.86, 1.2, 1.0)
        c.tick(1.0)
        for i in range(100):
            t = 1.01 + i * .01
            c.update_wrench(12.7 * G, t) # 5.7 kg payload
            if i % 20 == 0: c.update_pose(-3.8, 1.2, t)
            c.tick(t)
        self.assertEqual(c.phase, Phase.ERROR)

    def test_stale_sensor_forces_zero(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, .5)
        self.assertEqual(c.tick(.5), 8.57)
        self.assertEqual(c.tick(7.0), 0.0)
        self.assertEqual(c.phase, Phase.ERROR)

    def test_out_of_lane_fails_closed(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.7, .5)
        self.assertEqual(c.tick(.5), 0.0)
        self.assertEqual(c.phase, Phase.ERROR)

    # --- Regression tests for review probes (2026-10-08) ---

    def test_wait_box_without_pose_keeps_waiting(self):
        c = self.make_cycle()
        for i in range(200):  # box not spawned yet: normal wait, rollers off
            t = 0.5 + 0.1 * i
            c.update_wrench(7 * G, t)
            self.assertEqual(c.tick(t), 0.0)
        self.assertEqual(c.phase, Phase.WAIT_BOX)

    def test_wait_box_does_not_start_on_stale_pose(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, 0.5)
        c.update_wrench(7 * G, 8.0)  # wrench fresh, pose 7.5 s old
        self.assertEqual(c.tick(8.0), 0.0)
        self.assertEqual(c.phase, Phase.WAIT_BOX)
        c.update_pose(-4.45, 1.2, 8.05)  # fresh pose -> start
        self.assertAlmostEqual(c.tick(8.05), 8.57)
        self.assertEqual(c.phase, Phase.TO_SCALE)

    def test_lane_departure_while_travelling_fails_closed(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, 0.5)
        self.assertAlmostEqual(c.tick(0.5), 8.57)
        c.update_wrench(7 * G, 0.6)
        c.update_pose(-4.20, 2.0, 0.6)
        self.assertEqual(c.tick(0.6), 0.0)
        self.assertEqual(c.phase, Phase.ERROR)

    def test_lane_departure_after_weighing_fails_closed(self):
        c = self.make_cycle()
        c.update_pose(-4.45, 1.2, 0.5)
        c.tick(0.5)
        c.update_pose(-3.86, 1.2, 0.9)
        c.tick(0.9)
        for i in range(100):
            t = 0.9 + 0.01 * (i + 1)
            c.update_wrench(12 * G, t)
            if i % 10 == 0: c.update_pose(-3.83, 1.2, t)
            c.tick(t)
        self.assertEqual(c.phase, Phase.TO_PICK)
        c.update_wrench(7 * G, t + 0.1)
        c.update_pose(-2.5, 1.2 + 0.25, t + 0.1)
        self.assertEqual(c.tick(t + 0.1), 0.0)
        self.assertEqual(c.phase, Phase.ERROR)

    def test_tare_times_out_with_insufficient_samples(self):
        c = AutoScaleCycle(Config(), now=0.0)
        for i in range(31):  # 1 Hz wrench never fills the 0.65 s window
            t = float(i)
            c.update_wrench(7 * G, t)
            self.assertEqual(c.tick(t), 0.0)
        self.assertEqual(c.phase, Phase.ERROR)
        self.assertIn("Tare", c.error)


    # --- Generator SKUs: length-dependent thresholds (2026-10-09) ---

    def test_config_for_default_box_matches_tuned_values(self):
        c = config_for_box(0.40, 5.0)
        self.assertAlmostEqual(c.unload_x_m, Config().unload_x_m, places=6)
        self.assertAlmostEqual(c.pick_x_m, Config().pick_x_m, places=6)
        self.assertLessEqual(c.scale_center_tolerance_m, Config().scale_center_tolerance_m)

    def test_large_box_full_cycle(self):
        cfg = config_for_box(0.52, 21.3)        # K13 length, generator mass
        c = AutoScaleCycle(cfg, now=0.0)
        for i in range(50):
            c.update_wrench(7 * G, 0.01 * i); c.tick(0.01 * i)
        c.update_pose(-4.45, 1.2, 0.5)
        self.assertAlmostEqual(c.tick(0.5), cfg.velocity_rad_s)
        c.update_pose(-3.85, 1.2, 0.9); c.tick(0.9)
        for i in range(100):
            t = 0.9 + 0.01 * (i + 1)
            c.update_wrench((7 + 21.3) * G, t)
            if i % 10 == 0: c.update_pose(-3.82, 1.2, t)
            c.tick(t)
        self.assertEqual(c.phase, Phase.TO_PICK)
        self.assertAlmostEqual(c.measured_kg, 21.3, places=6)
        for i in range(30):
            t += .01
            c.update_wrench(7 * G, t); c.update_pose(cfg.unload_x_m + 0.01, 1.2, t); c.tick(t)
        self.assertTrue(c.unloaded)
        stop_centre = -0.845 - 0.26             # box front touching the stopper
        c.update_pose(stop_centre, 1.2, t + 0.1)
        self.assertEqual(c.tick(t + 0.1), 0.0)
        self.assertEqual(c.phase, Phase.DONE)

    def test_large_box_off_centre_on_scale_fails(self):
        cfg = config_for_box(0.52, 21.3)
        self.assertLess(cfg.scale_center_tolerance_m, 0.10)   # 0.52 m box overhangs sooner

    def test_tare_parameter_for_v44_rollers(self):
        cfg = config_for_box(0.25, 0.9, tare_kg=10.0)
        c = AutoScaleCycle(cfg, now=0.0)
        for i in range(50):
            c.update_wrench(10 * G, 0.01 * i); c.tick(0.01 * i)
        self.assertEqual(c.phase, Phase.WAIT_BOX)
        c2 = AutoScaleCycle(config_for_box(0.25, 0.9), now=0.0)   # 7 kg expectation: 10 kg is off
        for i in range(50):
            c2.update_wrench(10 * G, 0.01 * i); c2.tick(0.01 * i)
        self.assertEqual(c2.phase, Phase.ERROR)

    def test_box_too_long_for_scale_rejected(self):
        with self.assertRaises(ValueError):
            config_for_box(0.80, 10.0)


if __name__ == "__main__":
    unittest.main()
