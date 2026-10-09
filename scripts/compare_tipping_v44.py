#!/usr/bin/env python3
"""Board-gap tipping sweep: the same box released 5 mm above a pallet top-board edge with its centre
of mass d mm beyond (d < 0, over the gap) or before (d > 0) the edge; reports the settled tilt.

  bullet  : PyBullet (DIRECT, same settings as config/ahead_simulator.yaml: 240 Hz, 100 iterations,
            box-pallet friction 0.72) on a 5-board deck with --board-width / --board-pitch.
  gazebo  : the running V4.4 Gazebo world (give its --board-width / --board-pitch); probe_NN boxes are
            created on the empty front part of the pallet (no gripper joint binds to that name),
            measured, then moved off the pallet to the floor.

  python3 scripts/compare_tipping_v44.py bullet [--board-width 0.143 --board-pitch 0.23925]
  python3 scripts/compare_tipping_v44.py gazebo [--tag b]
"""
import argparse
import math
import os
import subprocess
import sys
import time

BOX = (0.25, 0.20, 0.15)        # box_05 (K02), y = across the boards
MASS = 0.547
GAP = 0.005                     # release height above the deck, as in the executors
OFFSETS_MM = (-20, -10, -7, -4, -2, 2, 4, 7, 10)


def tilt_deg(q):
    return math.degrees(2 * math.asin(min(1.0, math.hypot(q[0], q[1]))))


def bullet(a):
    import pybullet as p
    out = []
    for d in OFFSETS_MM:
        c = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.80665, physicsClientId=c)
        p.setTimeStep(1 / 240, physicsClientId=c)
        p.setPhysicsEngineParameter(numSolverIterations=100, physicsClientId=c)
        th = 0.15 * 0.22
        span = a.board_pitch * 4
        for i in range(5):
            y = -span / 2 + i * a.board_pitch
            s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.55, a.board_width / 2, th / 2], physicsClientId=c)
            b = p.createMultiBody(0, s, basePosition=[0, y, -th / 2], physicsClientId=c)
            p.changeDynamics(b, -1, lateralFriction=math.sqrt(0.72), rollingFriction=0.001,
                             spinningFriction=0.01, restitution=0.02, physicsClientId=c)
        edge = a.board_pitch - a.board_width / 2          # lower edge of board 3 (centre +pitch)
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[v / 2 for v in BOX], physicsClientId=c)
        box = p.createMultiBody(MASS, s, basePosition=[0, edge + d / 1000, BOX[2] / 2 + GAP], physicsClientId=c)
        p.changeDynamics(box, -1, lateralFriction=math.sqrt(0.72), rollingFriction=0.001,
                         spinningFriction=0.01, restitution=0.02, physicsClientId=c)
        for _ in range(240 * 4):
            p.stepSimulation(physicsClientId=c)
        pos, q = p.getBasePositionAndOrientation(box, physicsClientId=c)
        out.append((d, tilt_deg(q), (pos[2] - (BOX[2] / 2)) * 1000))
        p.disconnect(c)
    return out


def gazebo(a):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(root, "scripts"))
    from moveit_pick_place_v44 import gz_world_poses
    world = "ahead_workcell_v4_2_physical_scale"
    create = subprocess.run(["ros2", "pkg", "prefix", "ros_gz_sim"], capture_output=True, text=True).stdout.strip()
    create = os.path.join(create, "lib/ros_gz_sim/create")
    sdf = "/tmp/compare_tipping_probe.sdf"
    subprocess.run([sys.executable, os.path.join(root, "scripts/make_box_sdf_v44.py"), "--size", *map(str, BOX),
                    "--mass", str(MASS), "--out", sdf], check=True, capture_output=True)
    edge = 1.20 - a.board_pitch - a.board_width / 2  # lower edge of board 1 (pallet centre y 1.20)
    deck = 0.15
    out = []
    for k, d in enumerate(OFFSETS_MM):
        name = f"probe_{a.tag}{k:02d}"
        x = -0.40 if k % 2 == 0 else -0.10
        y = edge + d / 1000
        subprocess.run([create, "-name", name, "-file", sdf, "-x", str(x), "-y", str(y),
                        "-z", str(deck + GAP + BOX[2] / 2)], capture_output=True, timeout=30)
        time.sleep(4.0)
        pose = gz_world_poses().get(name)
        if pose is None:
            out.append((d, float("nan"), float("nan")))
            continue
        out.append((d, tilt_deg(pose[1]), (pose[0][2] - deck - BOX[2] / 2) * 1000))
        subprocess.run(["ign", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req",
                        f'name: "{name}" position {{x: {-5.0 + 0.4 * k} y: -3.6 z: 0.1}} orientation {{w: 1}}'],
                       capture_output=True, timeout=10)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("engine", choices=["bullet", "gazebo"])
    # Defaults = config/ahead_simulator.yaml and the V4.4 world since 2026-10-09 (was 0.130 / 0.242).
    ap.add_argument("--board-width", type=float, default=0.143)
    ap.add_argument("--board-pitch", type=float, default=0.23925)
    ap.add_argument("--tag", default="a")
    a = ap.parse_args()
    rows = bullet(a) if a.engine == "bullet" else gazebo(a)
    print(f"{a.engine}: box {BOX} m {MASS} kg released {GAP * 1000:.0f} mm above the deck")
    print("  COM offset from board edge (mm, <0 = over the gap) | settled tilt (deg) | z drop (mm)")
    for d, t, z in rows:
        print(f"  {d:+4d} | {t:6.2f} | {z:+7.1f}")


if __name__ == "__main__":
    main()
