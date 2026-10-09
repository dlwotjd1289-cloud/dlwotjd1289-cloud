"""Synthetic-image tests for the V4.4 box localization (needs ROS Humble sourced)."""
import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import box_perception_v44 as P

BOX_RGB = (199, 163, 114)         # measured lit box-top colour in Gazebo
FLOOR_RGB = (90, 90, 95)
K01 = (0.22, 0.19, 0.09)          # smallest generator SKU
K13 = (0.52, 0.48, 0.40)          # largest generator SKU


def render(cam, x, y, yaw, size, extra=None):
    """Top face only (sides are darker and excluded by the mask anyway)."""
    img = np.full((cam.height, cam.width, 3), FLOOR_RGB, np.uint8)
    z = P.CONVEYOR_TOP_Z + size[2]
    pts = [P.world_to_pixel(x + sx * size[0] / 2 * math.cos(yaw) - sy * size[1] / 2 * math.sin(yaw),
                            y + sx * size[0] / 2 * math.sin(yaw) + sy * size[1] / 2 * math.cos(yaw), z, cam)
           for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    cv2.fillPoly(img, [np.array(pts, np.int32)], BOX_RGB)
    if extra is not None:
        extra(img)
    return img


class TestBoxPerception(unittest.TestCase):
    def check(self, cam, x, y, yaw, size, tol_m=0.005, yaw_tol_deg=1.5):
        r = P.localize(render(cam, x, y, yaw, size), cam, size)
        self.assertEqual(r["status"], "OK", r)
        self.assertLess(math.hypot(r["x"] - x, r["y"] - y), tol_m)
        self.assertLess(abs(math.degrees(r["yaw"] - yaw)), yaw_tol_deg)
        self.assertAlmostEqual(r["z"], P.CONVEYOR_TOP_Z + size[2] / 2)

    def test_topview_box_at_pick(self):
        self.check(P.TOPVIEW_TEST, -1.045, 1.20, 0.0, P.DEFAULT_BOX)

    def test_far_cctv_default_box(self):
        self.check(P.FAR_CCTV, -1.045, 1.18, 0.0, P.DEFAULT_BOX)

    def test_far_cctv_smallest_and_largest_sku(self):
        # far CCTV is the coarse stage: few pixels on the smallest box -> 3 deg yaw tolerance
        self.check(P.FAR_CCTV, -0.845 - K01[0] / 2, 1.20, math.radians(3), K01, yaw_tol_deg=3.0)
        self.check(P.FAR_CCTV, -0.845 - K13[0] / 2, 1.20, 0.0, K13)

    def test_far_cctv_rotated(self):
        self.check(P.FAR_CCTV, -1.10, 1.25, math.radians(-10), P.DEFAULT_BOX)

    def test_no_box(self):
        cam = P.FAR_CCTV
        r = P.localize(np.full((cam.height, cam.width, 3), FLOOR_RGB, np.uint8), cam)
        self.assertEqual(r["status"], "NO_BOX")

    def test_wrong_sku_size_rejected(self):
        cam = P.FAR_CCTV
        r = P.localize(render(cam, -1.045, 1.20, 0.0, P.DEFAULT_BOX), cam, K01)
        self.assertEqual(r["status"], "SIZE_MISMATCH")

    def test_pixels_outside_roi_ignored(self):
        cam = P.TOPVIEW_TEST

        def pallet_blob(img):
            u, v = P.world_to_pixel(-0.40, 1.20, 1.145, cam)
            cv2.rectangle(img, (u - 80, v - 80), (u + 80, v + 80), BOX_RGB, -1)
        r = P.localize(render(cam, -1.06, 1.20, 0.0, P.DEFAULT_BOX, extra=pallet_blob), cam)
        self.assertEqual(r["status"], "OK")
        self.assertLess(math.hypot(r["x"] + 1.06, r["y"] - 1.20), 0.005)

    def test_wrist_camera_ignores_placed_box_in_view(self):
        # A same-coloured box top on the pallet in the corner of the gripper camera image.
        cam = P.Camera(np.array([-1.03, 1.29, 1.045 + 0.45]), P.rot_rpy(0.0, 1.5708, 0.0),
                       P.WRIST_HFOV, P.WRIST_W, P.WRIST_H)
        x, y, size = -1.02, 1.20, (0.35, 0.25, 0.15)

        def other_box(img):
            cv2.rectangle(img, (0, 380), (110, 479), BOX_RGB, -1)
        r = P.localize(render(cam, x, y, 0.0, size, extra=other_box), cam, size,
                       roi=((x - 0.35, x + 0.35), (y - 0.35, y + 0.35)), expect_xy=(x + 0.01, y))
        self.assertEqual(r["status"], "OK", r)
        self.assertLess(math.hypot(r["x"] - x, r["y"] - y), 0.005)

    def test_wrist_camera_looking_down(self):
        # Gripper camera 0.45 m above the box top, looking straight down (as before a pick).
        cam = P.Camera(np.array([-1.04, 1.29, 1.145 + 0.45]), P.rot_rpy(0.0, 1.5708, 0.0),
                       P.WRIST_HFOV, P.WRIST_W, P.WRIST_H)
        x, y = -1.045, 1.20
        r = P.localize(render(cam, x, y, math.radians(2), P.DEFAULT_BOX), cam, P.DEFAULT_BOX,
                       roi=((x - 0.35, x + 0.35), (y - 0.35, y + 0.35)))
        self.assertEqual(r["status"], "OK", r)
        self.assertLess(math.hypot(r["x"] - x, r["y"] - y), 0.005)


if __name__ == "__main__":
    unittest.main()


def test_known_size_fit_with_touching_neighbour():
    """Pallet re-check: the target footprint touches a same-coloured neighbour on one side."""
    import numpy as np
    from box_perception_v44 import fit_known_rect
    g = np.arange(-0.6, 0.6, 0.003)
    X, Y = np.meshgrid(g, g)
    tx, ty, L, W = 0.03, -0.02, 0.41, 0.31          # target box (offset from the expected centre)
    tgt = (abs(X - tx) <= L / 2) & (abs(Y - ty) <= W / 2)
    nb = (abs(X - tx) <= 0.175) & (Y > ty + W / 2) & (Y <= ty + W / 2 + 0.25)   # neighbour above
    m = tgt | nb
    fit = fit_known_rect(X[m], Y[m], (L, W, 0.28), (0.0, 0.0), 0.0)
    assert fit is not None
    x, y, yaw, inside, ring = fit[:5]
    assert abs(x - tx) < 0.006 and abs(y - ty) < 0.006 and abs(yaw) < 0.02
    assert inside > 0.95 and ring < 0.45
