"""PyBullet stability tests for one pallet layout (pallet frame, metres).

Only test conditions confirmed in two primary sources are built in:

* Loading test (Mazur, Gamer, Ramos & Schoder 2025, ITOR 34(1):501-527):
  boxes are loaded one by one in sequence, each step is simulated for
  ``loading_step_s`` (0.3 s); a box is unstable when its translation along x,
  y or z exceeds T or its roll/pitch/yaw exceeds R (eq. 7). Loading stability
  ls = j_max / J (eq. 3). Friction mu_s 0.5, mu_d = 0.75 mu_s = 0.375 (Table 7).
* Tilt and sustained acceleration tests (Jagelcak & Kubanova 2024, Appl. Sci.
  14(9):3846): levels and hold times come from the measured events (Tables 3,
  5, 10) and the EN 12195-1 design values the paper cites; a tilt angle maps to
  a lateral level by tan (prEN 17321: 10 deg ~ 0.18 g).

A horizontal acceleration ``a`` of the pallet is applied as the equivalent
inertial load in the pallet frame: gravity becomes g * (-a * u, -1). A tilt
by theta rotates gravity: g * (sin(theta) u, -cos(theta)). Both act on a
pallet that does not move, so box motion is motion relative to the pallet.
"""

from dataclasses import dataclass, field
import math

try:
    import pybullet as p
except ImportError:          # Profile / config parsing still work without PyBullet
    p = None

G = 9.80665
SAMPLE_S = 0.05     # motion is sampled at this interval while a level is held
DIRECTIONS = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}


@dataclass(frozen=True)
class SimSettings:
    physics_hz: int = 240
    solver_iterations: int = 100
    loading_step_s: float = 0.3          # Mazur Table 7 (0.3 s per loading step)
    restitution: float = 0.0
    place_gap_m: float = 0.0005          # boxes are created this far above their pose


@dataclass(frozen=True)
class Profile:
    """One acceleration test: ``accel_g`` along each direction, raised over
    ``ramp_s``, held for ``hold_s``. ``mode`` = "tilt" or "acceleration"."""

    name: str
    mode: str
    accel_g: float
    hold_s: float
    ramp_s: float = 0.5
    directions: tuple = ("+x", "-x", "+y", "-y")

    def __post_init__(self):
        if self.mode not in ("tilt", "acceleration"):
            raise ValueError(f"{self.name}: mode must be tilt or acceleration")
        if self.accel_g <= 0 or self.hold_s <= 0 or self.ramp_s < 0:
            raise ValueError(f"{self.name}: accel_g, hold_s must be > 0 and ramp_s >= 0")
        unknown = set(self.directions) - set(DIRECTIONS)
        if unknown:
            raise ValueError(f"{self.name}: unknown directions {sorted(unknown)}")

    @property
    def tilt_deg(self):
        return math.degrees(math.atan(self.accel_g))


@dataclass
class Motion:
    """Largest per-axis translation (m) and rotation (deg) of any box."""

    translation_m: float = 0.0
    rotation_deg: float = 0.0
    box_id: str = ""

    def update(self, box_id, translation_m, rotation_deg):
        if max(translation_m - self.translation_m, rotation_deg - self.rotation_deg) > 0:
            self.box_id = box_id
        self.translation_m = max(self.translation_m, translation_m)
        self.rotation_deg = max(self.rotation_deg, rotation_deg)

    def within(self, max_translation_m, max_rotation_deg):
        return self.translation_m <= max_translation_m and self.rotation_deg <= max_rotation_deg

    def as_dict(self):
        return {"translation_m": round(self.translation_m, 5), "rotation_deg": round(self.rotation_deg, 3),
                "box_id": self.box_id}


@dataclass
class LayoutWorld:
    """A solid pallet top (z = 0 is the deck surface) and the layout boxes."""

    boxes: list
    pallet_xy: tuple
    friction: float
    settings: SimSettings = field(default_factory=SimSettings)

    def __post_init__(self):
        if p is None:
            raise ImportError("pybullet is required for the simulation tests (docker/run.sh)")
        s = self.settings
        self.cid = p.connect(p.DIRECT)
        p.setTimeStep(1.0 / s.physics_hz, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(numSolverIterations=s.solver_iterations, physicsClientId=self.cid)
        self.set_gravity((0.0, 0.0, -G))
        # Bullet multiplies the two bodies' friction: sqrt on each body gives
        # the wanted contact coefficient (as PhysicsConfig.bullet_body_frictions).
        self.body_mu = math.sqrt(self.friction)
        lx, ly = self.pallet_xy
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[lx / 2, ly / 2, 0.05], physicsClientId=self.cid)
        self.pallet = p.createMultiBody(0.0, shape, basePosition=[lx / 2, ly / 2, -0.05], physicsClientId=self.cid)
        self._dynamics(self.pallet)
        self.bodies = {}
        self.home = {}

    def _dynamics(self, body):
        p.changeDynamics(body, -1, lateralFriction=self.body_mu, restitution=self.settings.restitution,
                         physicsClientId=self.cid)

    def set_gravity(self, vec):
        p.setGravity(*vec, physicsClientId=self.cid)

    def step_seconds(self, seconds):
        for _ in range(max(1, int(round(seconds * self.settings.physics_hz)))):
            p.stepSimulation(physicsClientId=self.cid)

    def add(self, box):
        """``box``: pac_candidates.stability.StackBox (min corner, extents)."""
        half = [box.dx / 2, box.dy / 2, box.dz / 2]
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid)
        centre = [box.x + half[0], box.y + half[1], box.z + half[2] + self.settings.place_gap_m]
        body = p.createMultiBody(max(1e-3, box.mass_kg), shape, basePosition=centre,
                                 baseInertialFramePosition=list(box.com_offset), physicsClientId=self.cid)
        self._dynamics(body)
        self.bodies[box.box_id] = body
        self.home[box.box_id] = self.pose(body)
        return body

    def pose(self, body):
        return p.getBasePositionAndOrientation(body, physicsClientId=self.cid)

    def motion_since(self, reference, into):
        """Per-axis translation and roll/pitch/yaw of every box against ``reference``."""
        for box_id, body in self.bodies.items():
            (x0, q0) = reference[box_id]
            pos, quat = self.pose(body)
            translation = max(abs(a - b) for a, b in zip(pos, x0))
            _, inv0 = p.invertTransform([0, 0, 0], q0)
            _, rel = p.multiplyTransforms([0, 0, 0], quat, [0, 0, 0], inv0)
            rotation = math.degrees(max(abs(a) for a in p.getEulerFromQuaternion(rel)))
            into.update(box_id, translation, rotation)

    def snapshot(self):
        return {box_id: self.pose(body) for box_id, body in self.bodies.items()}

    def save(self):
        return p.saveState(physicsClientId=self.cid)

    def restore(self, state_id):
        p.restoreState(stateId=state_id, physicsClientId=self.cid)

    def close(self):
        if self.cid >= 0:
            p.disconnect(physicsClientId=self.cid)
            self.cid = -1


def loading_test(world, boxes, stop_translation_m, stop_rotation_deg):
    """Mazur et al. eq. 3-7: load box by box. Returns the largest motion of any
    loaded box after each step (measured against each box's placed pose);
    loading stops after the first step beyond the stop thresholds."""
    steps = []
    for box in boxes:
        world.add(box)
        step = Motion()
        world.step_seconds(world.settings.loading_step_s)
        world.motion_since(world.home, step)
        steps.append(step)
        if not step.within(stop_translation_m, stop_rotation_deg):
            break
    return steps


def loading_verdict(steps, total, max_translation_m, max_rotation_deg):
    """ls = j_max / J with j_max = steps before the first unstable one."""
    j_max = 0
    for step in steps:
        if not step.within(max_translation_m, max_rotation_deg):
            break
        j_max += 1
    worst = max(steps, key=lambda m: (m.translation_m, m.rotation_deg), default=Motion())
    return {"ls": j_max / total if total else 1.0, "j_max": j_max, "boxes": total,
            "stable": j_max == total, "motion": worst.as_dict()}


def gravity_for(profile, direction, level):
    """Gravity vector in the pallet frame for ``level`` (0..1 of the profile)."""
    ux, uy = DIRECTIONS[direction]
    a = profile.accel_g * level
    if profile.mode == "tilt":
        theta = math.atan(a)
        return (G * math.sin(theta) * ux, G * math.sin(theta) * uy, -G * math.cos(theta))
    return (-G * a * ux, -G * a * uy, -G)   # pallet accelerates along u: load is pushed to -u


def acceleration_test(world, saved_state, profile):
    """Run ``profile`` along each direction from the loaded state; returns the
    largest motion per direction, measured against the poses at the start."""
    out = {}
    hz = world.settings.physics_hz
    for direction in profile.directions:
        world.restore(saved_state)
        reference = world.snapshot()
        motion = Motion()
        ramp_steps = int(round(profile.ramp_s * hz))
        for k in range(ramp_steps):
            world.set_gravity(gravity_for(profile, direction, (k + 1) / ramp_steps))
            world.step_seconds(1.0 / hz)
            world.motion_since(reference, motion)
        world.set_gravity(gravity_for(profile, direction, 1.0))
        remaining = profile.hold_s
        while remaining > 1e-9:             # sample the motion every SAMPLE_S
            chunk = min(SAMPLE_S, remaining)
            world.step_seconds(chunk)
            world.motion_since(reference, motion)
            remaining -= chunk
        out[direction] = motion
    world.restore(saved_state)
    world.set_gravity((0.0, 0.0, -G))
    return out


def profile_verdict(profile, motions, max_translation_m, max_rotation_deg):
    directions = {d: {"pass": m.within(max_translation_m, max_rotation_deg), "motion": m.as_dict()}
                  for d, m in motions.items()}
    return {"profile": profile.name, "mode": profile.mode, "accel_g": profile.accel_g,
            "tilt_deg": round(profile.tilt_deg, 2) if profile.mode == "tilt" else None,
            "hold_s": profile.hold_s, "pass": all(r["pass"] for r in directions.values()),
            "directions": directions}


@dataclass
class LayoutRun:
    """Raw motions of one layout at one friction value; ``verdict`` applies
    any (T, R) pair to them (Mazur Fig. 6 sensitivity needs no re-run)."""

    friction: float
    boxes: int
    loading_steps: list
    tests: list          # [(Profile, {direction: Motion})]; empty if loading failed

    def verdict(self, max_translation_m, max_rotation_deg):
        loading = loading_verdict(self.loading_steps, self.boxes, max_translation_m, max_rotation_deg)
        tests = [profile_verdict(prof, motions, max_translation_m, max_rotation_deg)
                 for prof, motions in self.tests] if loading["stable"] else []
        return {"friction": self.friction, "loading": loading, "tests": tests,
                "pass": loading["stable"] and all(t["pass"] for t in tests)}


def run_layout(boxes, pallet_xy, friction, profiles, thresholds, settings=SimSettings()):
    """Loading test, then every profile on the loaded layout. ``thresholds``:
    (T, R) pairs; loading stops and the profiles are skipped beyond the
    largest pair (such a layout fails at every pair)."""
    stop_t = max(t for t, _ in thresholds)
    stop_r = max(r for _, r in thresholds)
    world = LayoutWorld(boxes, pallet_xy, friction, settings)
    try:
        steps = loading_test(world, boxes, stop_t, stop_r)
        tests = []
        if len(steps) == len(boxes) and all(m.within(stop_t, stop_r) for m in steps):
            saved = world.save()
            tests = [(prof, acceleration_test(world, saved, prof)) for prof in profiles]
        return LayoutRun(friction, len(boxes), steps, tests)
    finally:
        world.close()
