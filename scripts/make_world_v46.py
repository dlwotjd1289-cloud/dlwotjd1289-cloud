#!/usr/bin/env python3
"""Generate the V4.6 workcell world from the V4.4 world (V4.4 stays unchanged).

V4.6 (2026-10-09, user request):
  - weighing section at the very start of the conveyor and a longer conveyor: the infeed and the
    scale move SHIFT_M upstream and the gap is filled with rollers (same 0.095 m pitch), so a box
    travels ~SHIFT_M longer from the scale to PICK (planning time while it travels);
  - two cameras only: camera 1 = straight-down top view of the weighing section (new pole),
    camera 2 = the existing pole CCTV that sees PICK and the pallet (the robot wrist camera is
    removed in the V4.6 launch);
  - floor extended upstream;
  - buffer table moved next to PICK on the robot side, parallel to the conveyor (user request), so
    the pole CCTV sees it; the pole CCTV is raised to 3.6 m with a wider view (hfov 1.6, 1920x1080)
    to see PICK, the whole pallet and both buffer bays (projection check: >= 190 px margin).
Everything downstream of the scale (PICK, stopper, robot, pallet, buffer table) is unchanged.

  python3 scripts/make_world_v46.py            # writes ahead_workcell_v4_6_two_cam.sdf
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / "ros2_ws/src/pac_simulation/worlds"
SRC = WORLDS / "ahead_workcell_v4_4_suction.sdf"
DST = WORLDS / "ahead_workcell_v4_6_two_cam.sdf"

PITCH = 0.095
N_NEW = 23                      # added rollers
SHIFT_M = round(N_NEW * PITCH, 3)   # 2.185 m
SPLIT_X = -3.50                 # V4.4: rollers upstream of this (x <= -3.56) belong to infeed + scale
FIRST_DOWNSTREAM = -3.465       # V4.4 first roller after the scale (roller_06m)
PICK_END_X = -0.85              # downstream end of the conveyor (pick stopper side)
BUFFER_XY = (-1.05, 0.40)       # buffer table centre (bays at x -/+ 0.215): x -1.50..-0.60, y 0.11..0.69
CAM2_Z = 3.60                   # pole CCTV height (was 2.40)
CAM2_POSE = f"-0.6 1.78 {CAM2_Z} 0 1.265364 -1.352630"   # pitch 72.5 deg, yaw -77.5 deg


def fnum(v):
    return f"{v:.6f}".rstrip("0").rstrip(".") if abs(v) > 1e-9 else "0"


def shift_pose(block, dx):
    def rep(m):
        vals = m.group(1).split()
        vals[0] = fnum(float(vals[0]) + dx)
        return f"<pose>{' '.join(vals)}</pose>"
    return re.sub(r"<pose>([^<]+)</pose>", rep, block, count=1)


def main() -> int:
    s = SRC.read_text()
    a = s.index('<model name="conveyor_main">')
    b = s.index("</model>", a)
    m = s[a:b]

    # --- rollers: link + joint + controller plugin blocks -------------------------------
    blocks = re.findall(r'(\s*<link name="(roller_[^"]+)_link">.*?</link>\s*<joint name="\2_joint".*?</joint>\s*'
                        r'<plugin filename="libignition-gazebo-joint-controller-system.so".*?</plugin>)', m, re.S)
    assert len(blocks) == 41, len(blocks)
    template = None
    out_rollers = []
    for blk, name in blocks:
        x = float(re.search(r"<pose>([^<]+)</pose>", blk).group(1).split()[0])
        if x < SPLIT_X:
            out_rollers.append(shift_pose(blk, -SHIFT_M))          # infeed + scale rollers
        else:
            if template is None and "<parent>conveyor_link</parent>" in blk:
                template = (blk, name, x)
            out_rollers.append(blk)
    tblk, tname, tx = template
    added = []
    for i in range(N_NEW):
        x = FIRST_DOWNSTREAM - SHIFT_M + i * PITCH
        nb = tblk.replace(f"{tname}_link", f"roller_ext{i:02d}_link").replace(f"{tname}_joint", f"roller_ext{i:02d}_joint")
        added.append(shift_pose(nb, x - tx))
    roller_text = "".join(out_rollers[:13]) + "".join(added) + "".join(out_rollers[13:])
    first = m.index(blocks[0][0])
    last = m.index(blocks[-1][0]) + len(blocks[-1][0])
    m = m[:first] + roller_text + m[last:]

    # --- frame (conveyor_link visuals / collisions) ----------------------------------------
    x_start = -4.7 - SHIFT_M - 0.10
    length = PICK_END_X + 0.05 - x_start
    centre = x_start + length / 2

    def set_box(text, name, pose_x=None, size_x=None, dx=None):
        i = text.index(f'name="{name}"')
        j = text.index("</", text.index("</geometry>", i))
        blk = text[i:j]
        if dx is not None:
            blk = shift_pose(blk, dx)
        if pose_x is not None:
            blk = re.sub(r"<pose>(\S+)", f"<pose>{fnum(pose_x)}", blk, count=1)
        if size_x is not None:
            blk = re.sub(r"<size>(\S+)", f"<size>{fnum(size_x)}", blk, count=1)
        return text[:i] + blk + text[j:]

    for rail in ("rail_left", "rail_right"):
        m = set_box(m, rail, pose_x=centre, size_x=length)
    m = set_box(m, "drive_motor", dx=-SHIFT_M)
    m = set_box(m, "infeed_support_collision", dx=-SHIFT_M)
    out_start = -3.38 - SHIFT_M
    m = set_box(m, "outfeed_support_collision", pose_x=(out_start + PICK_END_X) / 2, size_x=PICK_END_X - out_start)
    # legs + crossbars: regenerate every ~1.15 m
    leg_tpl = re.search(r'\s*<visual name="leg_l_0">.*?</visual>', m, re.S).group(0)
    legr_tpl = re.search(r'\s*<visual name="leg_r_0">.*?</visual>', m, re.S).group(0)
    bar_tpl = re.search(r'\s*<visual name="crossbar_0">.*?</visual>', m, re.S).group(0)
    m = re.sub(r'\s*<visual name="(leg_[lr]|crossbar)_\d+">.*?</visual>', "", m, flags=re.S)
    xs = [-4.55 - SHIFT_M + 1.15 * k for k in range(6)] + [-1.10]
    legs = ""
    for k, x in enumerate(xs):
        for tpl, nm in ((leg_tpl, "leg_l_0"), (legr_tpl, "leg_r_0"), (bar_tpl, "crossbar_0")):
            blk = tpl.replace(nm, nm[:-1] + str(k))
            legs += re.sub(r"<pose>(\S+)", f"<pose>{fnum(x)}", blk, count=1)
    anchor = m.index('<visual name="pick_stopper">')
    anchor = m.rfind("\n", 0, anchor)
    m = m[:anchor] + legs + m[anchor:]

    # --- weigh platform link (+ its force/torque joint stays) ----------------------------
    i = m.index('<link name="weigh_platform_link">')
    m = m[:i] + shift_pose(m[i:], -SHIFT_M)
    s = s[:a] + m + s[b:]

    # --- inline_scale visuals -----------------------------------------------------------------
    a2 = s.index('<model name="inline_scale">')
    b2 = s.index("</model>", a2)
    sc = re.sub(r"<pose>([^<]+)</pose>",
                lambda mm: "<pose>" + " ".join([fnum(float(mm.group(1).split()[0]) - SHIFT_M)] + mm.group(1).split()[1:]) + "</pose>",
                s[a2:b2])
    s = s[:a2] + sc + s[b2:]

    # --- floor: extend upstream ---------------------------------------------------------------
    s = s.replace("<pose>-1.0 -0.5 -0.05 0 0 0</pose>", "<pose>-2.1 -0.5 -0.05 0 0 0</pose>")
    s = s.replace("<size>9.0 7.0 0.10</size>", "<size>11.2 7.0 0.10</size>")

    # --- buffer table next to PICK (robot side, parallel to the conveyor) -----------------------
    a4 = s.index('<model name="buffer_rack">')
    k4 = s.index("<pose>1.45 -0.55 0 0 0 0</pose>", a4)       # model pose (after the V4.4 comment)
    s = s[:k4] + f"<pose>{BUFFER_XY[0]} {BUFFER_XY[1]} 0 0 0 0</pose>" + s[k4 + len("<pose>1.45 -0.55 0 0 0 0</pose>"):]

    # --- camera 2 (pole CCTV): higher and wider -------------------------------------------------
    a3 = s.index('<model name="camera_1_cctv_base">')
    b3 = s.index("</model>", a3)
    c2 = s[a3:b3]
    c2 = c2.replace("<pose>-0.6 1.95 1.17 0 0 0</pose>", f"<pose>-0.6 1.95 {CAM2_Z / 2 - 0.03:.3f} 0 0 0</pose>")
    c2 = c2.replace("<size>0.07 0.07 2.34</size>", f"<size>0.07 0.07 {CAM2_Z - 0.06:.2f}</size>")
    c2 = c2.replace("<pose>-0.60 1.865 2.34 0 0 0</pose>", f"<pose>-0.60 1.865 {CAM2_Z - 0.06:.2f} 0 0 0</pose>")
    c2 = c2.replace("<pose>-0.6 1.78 2.4 0 1.178996 -1.317721</pose>", f"<pose>{CAM2_POSE}</pose>")
    c2 = c2.replace("<horizontal_fov>1.40</horizontal_fov>", "<horizontal_fov>1.60</horizontal_fov>")
    c2 = c2.replace("<width>1280</width>", "<width>1920</width>").replace("<height>720</height>", "<height>1080</height>")
    assert c2.count(CAM2_POSE) == 2, c2.count(CAM2_POSE)
    s = s[:a3] + c2 + s[b3:]

    # --- camera 1: straight-down top view of the weighing section -------------------------
    sx = -3.80 - SHIFT_M
    cam = f"""
    <model name="camera_scale_top">
      <!-- V4.6 camera 1: top view of the weighing section (pole beside the conveyor, arm over it).
           Camera 2 is camera_1_cctv_base (PICK + pallet). -->
      <static>true</static>
      <link name="camera_scale_link">
        <collision name="pole_base_collision"><pose>{fnum(sx)} 2.00 0.025 0 0 0</pose><geometry><box><size>0.24 0.24 0.05</size></box></geometry></collision>
        <visual name="pole_base"><pose>{fnum(sx)} 2.00 0.025 0 0 0</pose><geometry><box><size>0.24 0.24 0.05</size></box></geometry></visual>
        <collision name="pole_collision"><pose>{fnum(sx)} 2.00 1.15 0 0 0</pose><geometry><box><size>0.07 0.07 2.30</size></box></geometry></collision>
        <visual name="pole"><pose>{fnum(sx)} 2.00 1.15 0 0 0</pose><geometry><box><size>0.07 0.07 2.30</size></box></geometry></visual>
        <visual name="arm"><pose>{fnum(sx)} 1.60 2.27 0 0 0</pose><geometry><box><size>0.06 0.80 0.06</size></box></geometry></visual>
        <visual name="camera_housing"><pose>{fnum(sx)} 1.20 2.20 0 1.5708 0</pose><geometry><box><size>0.20 0.11 0.095</size></box></geometry></visual>
        <sensor name="scale_top_camera" type="camera">
          <pose>{fnum(sx)} 1.20 2.20 0 1.5708 0</pose>
          <always_on>true</always_on>
          <update_rate>15</update_rate>
          <topic>/pac/scale_camera/image</topic>
          <camera>
            <horizontal_fov>1.00</horizontal_fov>
            <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
            <clip><near>0.10</near><far>6.0</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""
    k = s.index('<model name="camera_1_cctv_base">')
    s = s[:k] + cam.lstrip("\n") + "    " + s[k:]
    s = s.replace("<world name=", "<!-- V4.6: generated by scripts/make_world_v46.py from ahead_workcell_v4_4_suction.sdf -->\n  <world name=", 1)
    DST.write_text(s)
    print(f"wrote {DST.name}: conveyor {length:.2f} m (x {x_start:.3f}..{PICK_END_X + 0.05:.3f}), "
          f"scale centre x {sx:.3f}, inlet x {-4.45 - SHIFT_M:.3f}, shift {SHIFT_M} m, rollers {41 + N_NEW}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
