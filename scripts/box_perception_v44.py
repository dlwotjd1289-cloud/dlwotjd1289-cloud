#!/usr/bin/env python3
"""V4.4 box localization at PICK with two RGB cameras (camera type/model still undecided).

- far:   fixed pole CCTV camera_1_cctv_base (world pose = calibration), sees PICK + pallet.
         Coarse pose of the box resting at the pick stopper.
- wrist: gripper camera wrist_camera_link (pose from TF at the image time). Used just before
         the pick, from above the coarse pose, to refine centre and yaw.

Per image: colour mask (lit cardboard top face) -> pixels projected onto the box-top plane
(conveyor top + SKU height; an RGB camera cannot measure height, the SKU comes from the
label/WMS identity of the arriving box) -> ROI filter -> centre (footprint), yaw (PCA),
footprint size checked against the SKU.

Inputs : /pac/perception/target  std_msgs/String JSON {"box": "box_03", "size": [x, y, z]}
         optional "base_z" (surface the box rests on, default conveyor top), "expect" [x, y] and
         "expect_yaw": re-perception of a box already placed on the pallet (gripper camera only,
         SKU-sized rectangle fit because same-coloured neighbours may touch it in the image).
Outputs: /pac/perception/<cam>/box_info  std_msgs/String JSON (status, centre, yaw, footprint)
         /pac/perception/<cam>/box_pose  geometry_msgs/PoseStamped (world, status OK only)
         /pac/perception/<cam>/debug_image sensor_msgs/Image (RViz)
"""
from __future__ import annotations

import json
import math
import signal
import sys
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

CONVEYOR_TOP_Z = 0.895
DEFAULT_BOX = (0.40, 0.30, 0.25)
CONVEYOR_ROI = ((-1.75, -0.84), (0.80, 1.60))   # lane before the pick stopper (world x, y)
SIZE_TOL_M = 0.05
# Known-size fit on the pallet: accepted on the filled fraction of the SKU rectangle (a tilted or
# fallen box does not fill it on the slot plane). The outside band only steers the position: a box
# on a larger lower box or between neighbours has a same-coloured band on several sides (box_10).
FIT_MIN_INSIDE, FIT_MAX_RING = 0.85, 0.95
MIN_PIXELS = 300


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@dataclass
class Camera:
    pos: np.ndarray            # world position of the camera frame
    R: np.ndarray              # world rotation; Gazebo camera looks along +x, +y left, +z up
    hfov: float
    width: int
    height: int


# Fixed pole CCTV (camera_1_cctv_base / cctv_camera_1 in the workcell SDF).
FAR_CCTV = Camera(np.array([-0.6, 1.78, 2.4]), rot_rpy(0.0, 1.178996, -1.317721), 1.40, 1280, 720)
# Straight-down reference camera above PICK (synthetic tests only).
TOPVIEW_TEST = Camera(np.array([-1.06, 1.20, 2.60]), rot_rpy(0.0, 1.5708, 0.0), 1.0, 1280, 960)
WRIST_HFOV, WRIST_W, WRIST_H = 1.30, 640, 480


def box_mask(rgb: np.ndarray) -> np.ndarray:
    """Lit cardboard top face, measured in Gazebo (HSV ~ 17/109/199). Excludes the orange pick
    stopper (S ~ 207), shaded box sides, and the pallet boards (V ~ 173 when seen up close by the
    gripper camera) by value."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((h >= 10) & (h <= 25) & (s >= 70) & (s <= 160) & (v >= 185)).astype(np.uint8)


def pixels_to_plane(us, vs, cam: Camera, z_plane):
    fx = (cam.width / 2.0) / math.tan(cam.hfov / 2.0)
    d_cam = np.stack([np.ones_like(us, dtype=float), -(us - cam.width / 2.0) / fx, -(vs - cam.height / 2.0) / fx])
    d = cam.R @ d_cam
    t = (z_plane - cam.pos[2]) / d[2]
    return cam.pos[0] + t * d[0], cam.pos[1] + t * d[1]


def world_to_pixel(x, y, z, cam: Camera):
    fx = (cam.width / 2.0) / math.tan(cam.hfov / 2.0)
    d = cam.R.T @ (np.array([x, y, z]) - cam.pos)
    return int(round(cam.width / 2.0 - fx * d[1] / d[0])), int(round(cam.height / 2.0 - fx * d[2] / d[0]))


def localize(rgb, cam: Camera, box_size=DEFAULT_BOX, roi=CONVEYOR_ROI, expect_xy=None, base_z=CONVEYOR_TOP_Z,
             split_px=0, expect_yaw=None):
    """Return dict(status, x, y, z, yaw, length, width, pixels).

    The mask is split into connected blobs and only the blob whose projected centre is closest
    to `expect_xy` (default: ROI centre) is used: tops of boxes already on the pallet are the
    same colour and were merged into the PICK box by the gripper camera (2026-10-09)."""
    mask = box_mask(rgb)
    out = {"status": "NO_BOX", "pixels": int(mask.sum())}
    if out["pixels"] < MIN_PIXELS:
        return out
    z_top = base_z + box_size[2]
    if expect_xy is None:
        expect_xy = ((roi[0][0] + roi[0][1]) / 2, (roi[1][0] + roi[1][1]) / 2)
    # split_px: erode before labelling so that same-coloured neighbours touching in the image
    # (boxes 20 mm apart on the pallet) become separate blobs; the chosen blob is grown back
    # by the same amount inside the original mask.
    kernel = np.ones((3, 3), np.uint8)
    lab_mask = cv2.erode(mask, kernel, iterations=split_px) if split_px else mask
    n, labels, stats, cents = cv2.connectedComponentsWithStats(lab_mask, connectivity=8)
    best, best_d = None, None
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS:
            continue
        cx, cy = pixels_to_plane(np.array([cents[i][0]]), np.array([cents[i][1]]), cam, z_top)
        if not (roi[0][0] < cx[0] < roi[0][1] and roi[1][0] < cy[0] < roi[1][1]):
            continue
        d = math.hypot(cx[0] - expect_xy[0], cy[0] - expect_xy[1])
        if best_d is None or d < best_d:
            best, best_d = i, d
    if best is None:
        return out
    sel = (labels == best).astype(np.uint8)
    if split_px:
        # The suction cup hides part of the top face (camera 90 mm beside the TCP), which can cut
        # the box into pieces: merge every blob whose centre lies inside the expected footprint
        # (neighbours' centres are farther than half the box size + the 20 mm placement gap).
        radius = max(box_size[:2]) / 2 + 0.02
        for i in range(1, n):
            if i == best or stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS // 4:
                continue
            cx, cy = pixels_to_plane(np.array([cents[i][0]]), np.array([cents[i][1]]), cam, z_top)
            if math.hypot(cx[0] - expect_xy[0], cy[0] - expect_xy[1]) <= radius:
                sel |= (labels == i).astype(np.uint8)
        sel = cv2.dilate(sel, kernel, iterations=split_px) & mask
    vs, us = np.nonzero(sel)
    xs, ys = pixels_to_plane(us.astype(float), vs.astype(float), cam, z_top)
    keep = (xs > roi[0][0]) & (xs < roi[0][1]) & (ys > roi[1][0]) & (ys < roi[1][1])
    xs, ys = xs[keep], ys[keep]
    out["pixels"] = int(len(xs))
    if len(xs) < MIN_PIXELS:
        return out
    if expect_yaw is not None:
        # Box on the pallet: known-size rectangle fit (neighbours of the same colour may touch it).
        fit = fit_known_rect(xs, ys, box_size, expect_xy, expect_yaw)
        if fit is None:
            return out
        # Sub-grid refinement from the box edges (the 5 mm grid alone reported exactly 5.0 mm offsets).
        fit = refine_rect_edges(xs, ys, box_size, fit)
        x, y, yaw, ins, rg = fit
        ok = ins >= FIT_MIN_INSIDE and rg <= FIT_MAX_RING
        out.update({"status": "OK" if ok else "FIT_POOR", "x": float(x), "y": float(y),
                    "z": z_top - box_size[2] / 2, "yaw": float(yaw), "length": float(box_size[0]),
                    "width": float(box_size[1]), "fit_inside": round(ins, 3), "fit_ring": round(rg, 3)})
        return out
    pts = np.stack([xs, ys], axis=1)
    c = pts.mean(axis=0)
    evals, evecs = np.linalg.eigh(np.cov((pts - c).T))
    major = evecs[:, int(np.argmax(evals))]
    yaw = (math.atan2(major[1], major[0]) + math.pi / 2) % math.pi - math.pi / 2
    R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    local = (pts - c) @ R.T
    lo, hi = np.percentile(local, [1, 99], axis=0)
    length, width = float(hi[0] - lo[0]), float(hi[1] - lo[1])
    centre = c + R.T @ ((lo + hi) / 2.0)
    want = sorted(box_size[:2], reverse=True)
    ok = abs(length - want[0]) < SIZE_TOL_M and abs(width - want[1]) < SIZE_TOL_M
    out.update({"status": "OK" if ok else "SIZE_MISMATCH", "x": float(centre[0]), "y": float(centre[1]),
                "z": z_top - box_size[2] / 2, "yaw": yaw, "length": length, "width": width})
    return out


def fit_known_rect(xs, ys, box_size, expect_xy, yaw0, search=0.08, cell=0.005, band=0.025, yaw_step=1.0,
                   yaw_span=4.0):
    """Known-size footprint fit (box on the pallet): slide/turn an SKU-sized rectangle around
    `expect_xy` / `yaw0` and maximise (covered fraction inside) - (occupied fraction of a band just
    outside). A same-coloured neighbour touching the box in the image fills one side of the band
    wherever the rectangle is, the three free sides pin the position. Returns
    (x, y, yaw, inside, band) or None."""
    L, W = box_size[0], box_size[1]
    ex, ey = expect_xy
    half = search + max(L, W) / 2 + band + 0.01
    nb = int(round(2 * half / cell))
    k = int(round(search / cell))
    hl, hw, bb = int(round(L / 2 / cell)), int(round(W / 2 / cell)), int(round(band / cell))
    best = None
    for dyaw in np.radians(np.arange(-yaw_span, yaw_span + 1e-6, yaw_step)):
        yaw = yaw0 + dyaw
        c, s_ = math.cos(yaw), math.sin(yaw)
        u = (xs - ex) * c + (ys - ey) * s_
        v = -(xs - ex) * s_ + (ys - ey) * c
        H, _, _ = np.histogram2d(u, v, bins=nb, range=[[-half, half], [-half, half]])
        occ = (H > 0).astype(np.int32)
        I = np.zeros((nb + 1, nb + 1), np.int64)
        I[1:, 1:] = occ.cumsum(0).cumsum(1)
        mid = nb // 2
        du, dv = np.meshgrid(np.arange(-k, k + 1), np.arange(-k, k + 1), indexing="ij")

        def bsum(a0, a1, b0, b1):
            a0, a1 = np.clip(a0, 0, nb), np.clip(a1, 0, nb)
            b0, b1 = np.clip(b0, 0, nb), np.clip(b1, 0, nb)
            return I[a1, b1] - I[a0, b1] - I[a1, b0] + I[a0, b0]

        cu, cv = mid + du, mid + dv
        inside = bsum(cu - hl, cu + hl, cv - hw, cv + hw) / float(4 * hl * hw)
        outer = bsum(cu - hl - bb, cu + hl + bb, cv - hw - bb, cv + hw + bb)
        ring = (outer - bsum(cu - hl, cu + hl, cv - hw, cv + hw)) / float(4 * (hl + bb) * (hw + bb) - 4 * hl * hw)
        score = inside - ring
        i = np.unravel_index(int(np.argmax(score)), score.shape)
        if best is None or score[i] > best[0]:
            ou, ov = du[i] * cell, dv[i] * cell
            best = (score[i], ex + ou * c - ov * s_, ey + ou * s_ + ov * c, yaw, float(inside[i]), float(ring[i]))
    if best is None:
        return None
    _, x, y, yaw, ins, rg = best
    return x, y, (yaw + math.pi / 2) % math.pi - math.pi / 2, ins, rg


def refine_rect_edges(xs, ys, box_size, fit, band=0.025):
    """Refine a known-size fit from its free edges: along each box axis, an edge whose outside band
    is empty is measured (1st / 99th percentile of the top-face points); a side touching a
    same-coloured neighbour is not used (centre = free edge -+ half size). Returns the fit tuple."""
    x, y, yaw, ins, rg = fit
    L, W = box_size[0], box_size[1]
    c, s_ = math.cos(yaw), math.sin(yaw)
    u = (xs - x) * c + (ys - y) * s_
    v = -(xs - x) * s_ + (ys - y) * c
    du = dv = 0.0
    for axis, half, other_half in ((0, L / 2, W / 2), (1, W / 2, L / 2)):
        a, b = (u, v) if axis == 0 else (v, u)
        core = np.abs(b) < other_half * 0.8
        near = core & (np.abs(a) < half + band)
        if near.sum() < 50:
            continue
        lo_free = not np.any(near & (a < -half - 0.006))
        hi_free = not np.any(near & (a > half + 0.006))
        inner = core & (np.abs(a) < half + 0.006)
        if inner.sum() < 50:
            continue
        lo, hi = np.percentile(a[inner], [0.5, 99.5])
        if lo_free and hi_free:
            d = (lo + hi) / 2
        elif lo_free:
            d = lo + half
        elif hi_free:
            d = hi - half
        else:
            d = 0.0
        if abs(d) > 0.006:            # refinement only within one coarse cell
            d = 0.0
        if axis == 0:
            du = d
        else:
            dv = d
    return x + du * c - dv * s_, y + du * s_ + dv * c, yaw, ins, rg


def draw(rgb, res, cam: Camera, box_size, label):
    dbg = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
    if "x" in res:
        x, y, yaw, L, W = res["x"], res["y"], res["yaw"], res["length"], res["width"]
        z = res["z"] + box_size[2] / 2
        corners = [world_to_pixel(x + sx * L / 2 * math.cos(yaw) - sy * W / 2 * math.sin(yaw),
                                  y + sx * L / 2 * math.sin(yaw) + sy * W / 2 * math.cos(yaw), z, cam)
                   for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        colour = (0, 200, 0) if res["status"] == "OK" else (0, 0, 255)
        cv2.polylines(dbg, [np.array(corners, np.int32)], True, colour, 3)
        text = f"{label} {res['status']} x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}"
    else:
        colour, text = (0, 0, 255), f"{label} {res['status']}"
    cv2.putText(dbg, text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
    return dbg


class BoxPerception(Node):
    def __init__(self):
        super().__init__("pac_box_perception_v44",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.box_name, self.box_size, self.base_z, self.expect_yaw = None, DEFAULT_BOX, None, None
        self.coarse = None
        self.pubs = {}
        for cam in ("far", "wrist"):
            self.pubs[cam] = (self.create_publisher(String, f"/pac/perception/{cam}/box_info", 10),
                              self.create_publisher(PoseStamped, f"/pac/perception/{cam}/box_pose", 10),
                              self.create_publisher(Image, f"/pac/perception/{cam}/debug_image", 2))
        self.create_subscription(String, "/pac/perception/target", self.on_target, 10)
        self.create_subscription(Image, "/pac/top_camera/image", lambda m: self.on_image(m, "far"),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/pac/wrist_camera/image", lambda m: self.on_image(m, "wrist"),
                                 qos_profile_sensor_data)
        self.last_status = {}

    def on_target(self, msg: String) -> None:
        t = json.loads(msg.data)
        base_z, expect = t.get("base_z"), t.get("expect")
        self.expect_yaw = t.get("expect_yaw")
        if t.get("box") != self.box_name or base_z != self.base_z or (expect and tuple(expect) != self.coarse):
            self.box_name, self.box_size, self.coarse = t.get("box"), tuple(t.get("size", DEFAULT_BOX)), None
            self.base_z = base_z
            if expect:
                self.coarse = tuple(expect)
            self.get_logger().info(f"target {self.box_name} size {self.box_size}"
                                   + (f" on surface z {base_z:.3f} near {expect}" if base_z is not None else ""))

    def wrist_camera(self, stamp) -> Camera | None:
        try:
            tf = self.tf_buffer.lookup_transform("world", "wrist_camera_link", Time.from_msg(stamp))
        except Exception:
            try:  # latest available if the exact stamp is not buffered yet
                tf = self.tf_buffer.lookup_transform("world", "wrist_camera_link", Time())
            except Exception:
                return None
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return Camera(np.array([t.x, t.y, t.z]), R, WRIST_HFOV, WRIST_W, WRIST_H)

    def on_image(self, msg: Image, cam_name: str) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        if msg.encoding == "bgr8":
            rgb = rgb[..., ::-1]
        rgb = np.ascontiguousarray(rgb)
        if cam_name == "far":
            if self.base_z is not None:   # box on the pallet: gripper camera only
                return
            cam, roi = FAR_CCTV, CONVEYOR_ROI
        else:
            cam = self.wrist_camera(msg.header.stamp)
            if cam is None or self.coarse is None or cam.R[2, 0] > -0.9:   # only when looking down
                return
            cx, cy = self.coarse
            # On the pallet the neighbours are close: look only around the expected footprint.
            r = 0.35 if self.base_z is None else max(self.box_size[:2]) / 2 + 0.13
            roi = ((cx - r, cx + r), (cy - r, cy + r))
        expect = self.coarse if cam_name == "wrist" else (-0.845 - self.box_size[0] / 2, 1.20)
        res = localize(rgb, cam, self.box_size, roi, expect,
                       CONVEYOR_TOP_Z if self.base_z is None else self.base_z,
                       split_px=0 if self.base_z is None else 4,
                       expect_yaw=None if self.base_z is None else (self.expect_yaw or 0.0))
        if cam_name == "far" and res["status"] == "OK":
            self.coarse = (res["x"], res["y"])
        res.update({"camera": cam_name, "box": self.box_name, "size_sku": list(self.box_size),
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, "frame": "world"})
        info_pub, pose_pub, dbg_pub = self.pubs[cam_name]
        info_pub.publish(String(data=json.dumps(res)))
        if self.last_status.get(cam_name) != res["status"]:
            self.get_logger().info(f"{cam_name}: {res['status']} {json.dumps({k: round(v, 3) for k, v in res.items() if isinstance(v, float)})}")
            self.last_status[cam_name] = res["status"]
        dbg = draw(rgb, res, cam, self.box_size, cam_name)
        dbg_pub.publish(Image(header=msg.header, height=dbg.shape[0], width=dbg.shape[1], encoding="bgr8",
                              is_bigendian=0, step=dbg.shape[1] * 3, data=dbg.tobytes()))
        if res["status"] == "OK":
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = res["x"], res["y"], res["z"]
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(res["yaw"] / 2), math.cos(res["yaw"] / 2)
            pose_pub.publish(ps)


def _raise(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    node = BoxPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
