"""0.3 g shock test with a measurable load profile (draft, beside shock_test.py).

Same rebuild and return keys as shock_test.shock_test(layout). What changes:
1. Load profile: lateral acceleration ramps up (half-cosine, rise_s) and holds
   (hold_s) instead of switching gravity instantly (step = infinite jerk).
2. Measurement: every box carries a virtual accelerometer (specific force from
   base velocity, low-pass filtered). Peak / nominal = how much harder the box
   was actually pushed than the nominal level (dynamic amplification).
3. Vertical bump: off by default. STABILITY_0_3G_BASIS lists vertical loads as
   non-overturning; shock_test.py applied 1.6 g for 0.15 s at every level.
4. Pass criteria: residual (permanent) and transient (elastic) displacement
   relative to the box height above the deck, plus tilt. Drift with no
   excitation is reported separately (a build/support problem, not a 0.3 g failure).

STEP_TEST reproduces shock_test.py, so one code path can be checked against the
old numbers after merging; REVISED is the proposal. Values marked ASSUMED still
need a source or a site measurement.
"""
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "wave"))
import physics_check as pc  # noqa: E402  (team simulator imports)
from pac_candidates.geometry import rotated_dims  # noqa: E402


@dataclass(frozen=True)
class Protocol:
    levels: tuple = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    directions: tuple = ((1, 0), (-1, 0), (0, 1), (0, -1))
    profile: str = "ramp"          # "step" | "ramp"
    rise_s: float = 0.3            # ASSUMED (forklift start/stop 1-2 s in STABILITY_0_3G_BASIS)
    hold_s: float = 0.4            # plateau at the nominal level
    settle_s: float = 0.3
    bump: tuple = ()               # ((g, s), ...) vertical gravity after each level
    idle_s: float = 1.0            # pre-test drift window (0 = skip)
    lpf_hz: float = 10.0           # accelerometer low-pass, ASSUMED (0 = raw physics rate)
    residual_frac: float = 0.05    # residual displacement <= frac * box top height, ASSUMED
    transient_frac: float = math.inf  # transient displacement <= frac * top height (inf = off)
    min_tol_m: float = 0.02        # never stricter than this
    tilt_deg: float = 10.0
    deck: str = ""                 # "" = simulator config, or "slatted" / "solid"


STEP_TEST = Protocol(profile="step", bump=((1.6, 0.15), (0.4, 0.15)), idle_s=0.0, residual_frac=0.0)
REVISED = Protocol(transient_frac=0.10)


def _rebuild(layout, proto):
    """Same as shock_test.shock_test: z/y/x order, lift 0/15/30 mm on overlap, 0.1 s per box, 0.5 s settle."""
    cfg, _ = pc.load_sim_config(pc.REPO / "config/ahead_simulator.yaml")
    if proto.deck:
        cfg = replace(cfg, pallet=replace(cfg.pallet, collision_model=proto.deck))
    sim = pc.AheadLiveSimulator(cfg)
    hz, L, W = cfg.physics.physics_hz, cfg.pallet.length_m, cfg.pallet.width_m
    bodies, mass, top = [], [], []
    for k, (box, pose) in enumerate(sorted(layout, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x))):
        dx, dy, dz = rotated_dims(box.size, pose.yaw)
        centre = (pose.x + dx / 2 - L / 2, pose.y + dy / 2 - W / 2, pose.z + dz / 2)
        for lift in (0.0, 0.015, 0.03):
            try:
                body = sim.place_box(pc.BoxSpec(f"b{k:03d}", (box.size.x, box.size.y, box.size.z), box.weight_kg,
                                                (centre[0], centre[1], centre[2] + lift), pose.yaw, source="shock"))
                break
            except ValueError as exc:
                err = str(exc)
        else:
            sim.close()
            return None, err
        bodies.append(body)
        mass.append(box.weight_kg)
        top.append(pose.z + dz)
        sim.step(int(0.1 * hz))
    sim.step(int(0.5 * hz))
    return (sim, cfg, bodies, np.array(mass), np.array(top)), None


class _Probe:
    """Per-step pose and virtual-accelerometer log for all boxes."""

    def __init__(self, sim, bodies, hz):
        self.sim, self.bodies, self.hz, self.cid = sim, bodies, hz, sim.world.client_id
        self.v_prev = self._vel()
        self.ref = self.pos()

    def _vel(self):
        return np.array([p.getBaseVelocity(b, physicsClientId=self.cid)[0] for b in self.bodies])

    def pos(self):
        return np.array([p.getBasePositionAndOrientation(b, physicsClientId=self.cid)[0] for b in self.bodies])

    def tilt(self):
        out = []
        for b in self.bodies:
            r, q, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(b, physicsClientId=self.cid)[1])
            out.append(math.degrees(max(abs(r), abs(q))))
        return np.array(out)

    def run(self, gravity):
        """Step once per gravity vector. Returns (specific force [n, boxes, 3], max transient displacement)."""
        spec, transient = [], np.zeros(len(self.bodies))
        for gv in gravity:
            p.setGravity(*gv, physicsClientId=self.cid)
            self.sim.step(1)
            v = self._vel()
            spec.append((v - self.v_prev) * self.hz - np.asarray(gv))   # accelerometer reading
            self.v_prev = v
            np.maximum(transient, np.linalg.norm(self.pos() - self.ref, axis=1), out=transient)
        return np.array(spec), transient


def _lateral(proto, a, d, g, hz):
    if proto.profile == "step":
        shape = np.ones(int(proto.hold_s * hz))
    else:
        rise = 0.5 - 0.5 * np.cos(np.pi * np.arange(int(proto.rise_s * hz)) / (proto.rise_s * hz))
        shape = np.concatenate([rise, np.ones(int(proto.hold_s * hz)), rise[::-1]])
    lat = [(d[0] * a * g * s, d[1] * a * g * s, -g) for s in shape]
    return lat + [(0.0, 0.0, -g)] * int(proto.settle_s * hz)


def _lowpass(x, hz, fc):
    from scipy.signal import butter, filtfilt
    b, a = butter(2, fc / (hz / 2))
    return filtfilt(b, a, x, axis=0) if len(x) > 9 else x


def shock_test(layout, proto=REVISED):
    """layout: [(PlacedBox, pose)] of one pallet. Returns shock_test.py keys plus measurements."""
    built, err = _rebuild(layout, proto)
    if built is None:
        return {"boxes": 0, "passed_g": None, "failed_at_g": None, "moved": 0, "tilted": 0, "rebuild_error": err}
    sim, cfg, bodies, mass, top = built
    hz, g = cfg.physics.physics_hz, cfg.physics.gravity_m_s2
    tol = np.maximum(proto.min_tol_m, proto.residual_frac * top)
    probe = _Probe(sim, bodies, hz)
    out = {"boxes": len(bodies), "pre_drift_m": 0.0, "pre_drift_fail": False, "levels": []}
    if proto.idle_s > 0:                                   # does anything move with no excitation?
        probe.run([(0.0, 0.0, -g)] * int(proto.idle_s * hz))
        drift = np.linalg.norm(probe.pos() - probe.ref, axis=1)
        out["pre_drift_m"] = float(drift.max())
        out["pre_drift_fail"] = bool((drift > tol).any() or (probe.tilt() > proto.tilt_deg).any())
        probe.ref = probe.pos()                            # shaking is judged from here on
    passed, failed = 0.0, None
    for a in proto.levels:
        peak, load_peak, transient = np.zeros(len(bodies)), 0.0, np.zeros(len(bodies))
        for d in proto.directions:
            spec, tr = probe.run(_lateral(proto, a, d, g, hz))
            h = spec[:, :, :2] / g                                      # horizontal, in g
            if proto.lpf_hz:
                h = _lowpass(h, hz, proto.lpf_hz)
            peak = np.maximum(peak, np.linalg.norm(h, axis=2).max(axis=0))
            load = (h * mass[None, :, None]).sum(axis=1) / mass.sum()   # = pallet-to-load force / total mass
            load_peak = max(load_peak, float(np.linalg.norm(load, axis=1).max()))
            transient = np.maximum(transient, tr)
        if proto.bump:
            probe.run([(0.0, 0.0, -gz * g) for gz, s in proto.bump for _ in range(int(s * hz))])
        probe.run([(0.0, 0.0, -g)] * int(0.3 * hz))
        residual = np.linalg.norm(probe.pos() - probe.ref, axis=1)
        tilt = probe.tilt()
        moved = residual > tol
        shaken = transient > proto.transient_frac * top
        tilted = tilt > proto.tilt_deg
        out["levels"].append({
            "a": a,
            "box_peak_over_nominal_p90": float(np.percentile(peak, 90) / a),
            "load_peak_over_nominal": load_peak / a,
            "max_residual_m": float(residual.max()), "max_transient_m": float(transient.max()),
            "max_tilt_deg": float(tilt.max()),
        })
        if moved.any() or shaken.any() or tilted.any():
            failed = a
            out.update(moved=int(moved.sum()), shaken=int(shaken.sum()), tilted=int(tilted.sum()))
            break
        passed = a
    sim.close()
    out.update(passed_g=passed, failed_at_g=failed)
    out.setdefault("moved", 0)
    out.setdefault("tilted", 0)
    return out
