"""PyBullet tilt tests for hard-mask step 13 (lateral stability).

short_tilt_test: stage 3 of the cascade (borderline LP results only). The
    affected system is built at its planned poses, settles 0.25 s, then gravity
    is tilted by ``accel_g`` (ramp ``sim_ramp_s``, hold ``sim_hold_s``) in the
    worst LP directions. Fails if a box moves > sim_disp_tol_m, tilts >
    sim_tilt_tol_deg or still moves faster than sim_speed_tol_m_s at the end.
reference_tilt_test: slow test used only to calibrate the bands
    (tools/donghan/lateral_calibrate.py).

Physics follows config/ahead_simulator.yaml (240 Hz, 100 solver iterations,
restitution 0.02, damping 0.04, spinning 0.01, rolling 0.001, deck boards
0.22 x 0.15 m thick). Friction uses the hard-mask values (box-box mu_box_box,
box-deck mu_box_pallet) instead of the simulator's development values.
Importing this module requires pybullet (optional dependency group "sim").
"""

import math

import pybullet as p

G = 9.80665
PHYSICS_HZ = 240
SOLVER_ITERATIONS = 100
RESTITUTION = 0.02
DAMPING = 0.04
SPINNING_FRICTION = 0.01
ROLLING_FRICTION = 0.001
DECK_THICKNESS_M = 0.22 * 0.15
SETTLE_S = 0.25
STATIC_DISP_TOL_M = 0.005
STATIC_TILT_TOL_DEG = 2.0


class TiltWorld:
    """Static deck plus the given bodies at their planned poses."""

    def __init__(self, bodies, deck, mu_box_box, mu_box_pallet):
        self.cid = p.connect(p.DIRECT)
        cid = self.cid
        p.setTimeStep(1.0 / PHYSICS_HZ, physicsClientId=cid)
        p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS, physicsClientId=cid)
        p.setGravity(0.0, 0.0, -G, physicsClientId=cid)
        # Bullet multiplies the two body frictions of a contact
        box_mu = math.sqrt(mu_box_box)
        deck_mu = mu_box_pallet / box_mu
        shape = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * len(deck),
            halfExtents=[[0.5 * r.width, 0.5 * r.depth, 0.5 * DECK_THICKNESS_M] for r in deck],
            collisionFramePositions=[[r.center[0], r.center[1], -0.5 * DECK_THICKNESS_M] for r in deck],
            physicsClientId=cid,
        )
        self.deck = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, physicsClientId=cid)
        p.changeDynamics(self.deck, -1, lateralFriction=deck_mu, restitution=RESTITUTION,
                         spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION,
                         physicsClientId=cid)
        self.ids = []
        for b in bodies:
            half = [0.5 * (h - l) for l, h in zip(b.lo, b.hi)]
            centre = list(b.center)
            centre[2] += 1e-4
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=cid)
            body = p.createMultiBody(baseMass=max(b.mass, 1e-3), baseCollisionShapeIndex=col,
                                     basePosition=centre, physicsClientId=cid)
            p.changeDynamics(body, -1, lateralFriction=box_mu, restitution=RESTITUTION,
                             spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION,
                             linearDamping=DAMPING, angularDamping=DAMPING, physicsClientId=cid)
            self.ids.append(body)

    def step(self, n):
        for _ in range(max(1, int(n))):
            p.stepSimulation(physicsClientId=self.cid)

    def poses(self):
        out = []
        for b in self.ids:
            pos, q = p.getBasePositionAndOrientation(b, physicsClientId=self.cid)
            r, pt, _ = p.getEulerFromQuaternion(q)
            out.append((pos, math.degrees(max(abs(r), abs(pt)))))
        return out

    def save(self):
        return p.saveState(physicsClientId=self.cid)

    def restore(self, state):
        p.restoreState(stateId=state, physicsClientId=self.cid)

    def close(self):
        p.disconnect(physicsClientId=self.cid)


def _displacement(ref, now, watch):
    disp = max((math.dist(ref[k][0], now[k][0]) for k in watch), default=0.0)
    tilt = max((now[k][1] for k in watch), default=0.0)
    return disp, tilt


def _offset_torques(bodies, u, accel_g):
    """Torque of each box's CoG uncertainty shifted towards ``u`` (same worst case
    as the hard mask), as a body torque: r x m*g*(a u, -1), r = (+-dx, +-dy, 0)."""
    out = []
    for b in bodies:
        rx = math.copysign(b.delta[0], u[0]) if abs(u[0]) > 1e-12 else 0.0
        ry = math.copysign(b.delta[1], u[1]) if abs(u[1]) > 1e-12 else 0.0
        w = b.mass * G
        out.append((-w * ry, w * rx, w * accel_g * (rx * u[1] - ry * u[0])))
    return out


def tilt_test(bodies, deck, mu_box_box, mu_box_pallet, accel_g, dirs, *, ramp_s, hold_s,
              disp_tol_m, tilt_tol_deg, speed_tol_m_s=math.inf, watch=None, cog_offset=True):
    """Settle, then for each direction: restore, ramp gravity tilt (and the CoG
    offset torque), hold, judge.

    Returns {"passed", "reason", "direction", "max_disp_m", "max_tilt_deg", "end_speed_m_s"}."""
    world = TiltWorld(bodies, deck, mu_box_box, mu_box_pallet)
    watch = range(len(bodies)) if watch is None else watch
    try:
        before = world.poses()
        world.step(SETTLE_S * PHYSICS_HZ)
        ref = world.poses()
        disp, tilt = _displacement(before, ref, watch)
        out = {"passed": True, "reason": "", "direction": None, "max_disp_m": disp,
               "max_tilt_deg": tilt, "end_speed_m_s": 0.0}
        if disp > STATIC_DISP_TOL_M or tilt > STATIC_TILT_TOL_DEG:
            out.update(passed=False, reason="static")
            return out
        state = world.save()
        n_ramp, n_hold = int(ramp_s * PHYSICS_HZ), int(hold_s * PHYSICS_HZ)
        for u in dirs:
            world.restore(state)

            torques = _offset_torques(bodies, u, accel_g) if cog_offset else None
            worst_d = worst_t = 0.0
            for k in range(n_ramp + n_hold):
                s = min(1.0, (k + 1) / n_ramp) if n_ramp else 1.0
                s = 0.5 - 0.5 * math.cos(math.pi * s)
                p.setGravity(u[0] * accel_g * G * s, u[1] * accel_g * G * s, -G, physicsClientId=world.cid)
                if torques:
                    for body, tq in zip(world.ids, torques):
                        p.applyExternalTorque(body, -1, [s * t for t in tq], p.WORLD_FRAME,
                                              physicsClientId=world.cid)
                p.stepSimulation(physicsClientId=world.cid)
                if k % 4 == 3:
                    d, t = _displacement(ref, world.poses(), watch)
                    worst_d, worst_t = max(worst_d, d), max(worst_t, t)
            speed = max(math.sqrt(sum(v * v for v in p.getBaseVelocity(world.ids[k], physicsClientId=world.cid)[0]))
                        for k in watch) if len(world.ids) else 0.0
            out["max_disp_m"] = max(out["max_disp_m"], worst_d)
            out["max_tilt_deg"] = max(out["max_tilt_deg"], worst_t)
            out["end_speed_m_s"] = max(out["end_speed_m_s"], speed)
            if worst_d > disp_tol_m or worst_t > tilt_tol_deg or speed > speed_tol_m_s:
                out.update(passed=False, reason="tilt", direction=list(u))
                return out
        return out
    finally:
        world.close()


def short_tilt_test(scene, system, dirs, cfg):
    """Stage 3 of the cascade on the affected system only."""
    bodies = [scene.bodies[i] for i in system]
    return tilt_test(bodies, scene.deck, cfg.mu_box_box, cfg.mu_box_pallet, cfg.accel_g, dirs,
                     ramp_s=cfg.sim_ramp_s, hold_s=cfg.sim_hold_s, disp_tol_m=cfg.sim_disp_tol_m,
                     tilt_tol_deg=cfg.sim_tilt_tol_deg, speed_tol_m_s=cfg.sim_speed_tol_m_s)


# Reference (calibration only): whole pallet, every direction, slow ramp, long hold.
REF_RAMP_S = 0.5
REF_HOLD_S = 1.0
REF_DISP_TOL_M = 0.02
REF_TILT_TOL_DEG = 5.0


def reference_tilt_test(scene, watch, cfg, dirs):
    return tilt_test(scene.bodies, scene.deck, cfg.mu_box_box, cfg.mu_box_pallet, cfg.accel_g, dirs,
                     ramp_s=REF_RAMP_S, hold_s=REF_HOLD_S, disp_tol_m=REF_DISP_TOL_M,
                     tilt_tol_deg=REF_TILT_TOL_DEG, watch=watch)


__all__ = ["TiltWorld", "reference_tilt_test", "short_tilt_test", "tilt_test"]
