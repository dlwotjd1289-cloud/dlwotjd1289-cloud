"""ROS-independent, single-box in-line weighing state machine.

All readings are real sensor readings in Gazebo; expected_mass_kg is a
quality-control assertion, never used to calculate the measured mass.
"""
from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Optional

G = 9.80665


class Phase(str, Enum):
    TARE = "TARE"
    WAIT_BOX = "WAIT_BOX"
    TO_SCALE = "TO_SCALE"
    SETTLING = "SETTLING"
    TO_PICK = "TO_PICK"
    DONE = "DONE"
    ERROR = "ERROR"


@dataclass(frozen=True)
class Config:
    velocity_rad_s: float = 8.57
    scale_stop_x_m: float = -3.86
    scale_center_x_m: float = -3.80
    scale_center_tolerance_m: float = 0.16
    unload_x_m: float = -3.18
    pick_x_m: float = -1.08
    scale_y_m: float = 1.20
    max_y_error_m: float = 0.18
    expected_tare_n: float = 7.0 * G
    tare_tolerance_n: float = 6.0
    tare_timeout_s: float = 10.0
    settle_time_s: float = 0.65
    settle_timeout_s: float = 15.0
    mass_stdev_tolerance_kg: float = 0.12
    expected_mass_kg: Optional[float] = 5.0
    expected_mass_tolerance_kg: float = 0.35
    sensor_timeout_s: float = 5.0
    start_pose_max_age_s: float = 0.5
    travel_timeout_s: float = 120.0
    spawn_wait_timeout_s: float = 30.0


# Workcell geometry behind the length-dependent thresholds (V4.2 world).
SCALE_CENTER_X_M = -3.80          # weigh platform 0.70 x 0.70 m
SCALE_HALF_LEN_M = 0.35
PICK_STOPPER_FACE_X_M = -0.845    # upstream face of the pick stopper


def config_for_box(length_m: float, expected_mass_kg: Optional[float], tare_kg: float = 7.0) -> "Config":
    """Thresholds for a box of `length_m` along the conveyor (generator SKUs vary in size).
    For the original 0.40 m test box this reproduces the hand-tuned defaults. `tare_kg`: empty
    platform + its rollers (7 kg in the V4.2 world, 10 kg with the V4.4 intermediate rollers)."""
    if not 0.10 <= length_m <= 0.66:
        raise ValueError(f"box length {length_m:.3f} m does not fit the 0.70 m scale platform")
    return Config(
        # box must rest fully on the platform (default 0.16 m for 0.40 m boxes)
        scale_center_tolerance_m=min(0.16, max(0.02, SCALE_HALF_LEN_M - length_m / 2 + 0.01)),
        # whole box past the platform's downstream edge (+0.07 m margin, as tuned)
        unload_x_m=SCALE_CENTER_X_M + SCALE_HALF_LEN_M + length_m / 2 + 0.07,
        # centre when the box touches the pick stopper, minus 0.035 m (as tuned)
        pick_x_m=PICK_STOPPER_FACE_X_M - length_m / 2 - 0.035,
        expected_mass_kg=expected_mass_kg,
        # +-7 % (at least the original 0.35 kg) of the expected fixture mass
        expected_mass_tolerance_kg=max(0.35, 0.07 * (expected_mass_kg or 0.0)),
        expected_tare_n=tare_kg * G,
    )


class AutoScaleCycle:
    """One automatic cycle; call update_wrench, update_pose, then tick."""

    def __init__(self, cfg: Config | None = None, now: float = 0.0):
        self.cfg = cfg or Config()
        self.phase = Phase.TARE
        self.phase_since = now
        self.baseline_n: Optional[float] = None
        self.force_z_n: Optional[float] = None
        self.last_wrench_at: Optional[float] = None
        self.last_pose_at: Optional[float] = None
        self.pose_x_m: Optional[float] = None
        self.pose_y_m: Optional[float] = None
        self.measured_kg: Optional[float] = None
        self.unloaded = False
        self.unload_residual_kg: Optional[float] = None
        self.error: Optional[str] = None
        self.messages: list[str] = []
        self.tare_samples: Deque[tuple[float, float]] = deque()
        self.mass_samples: Deque[tuple[float, float]] = deque()
        self.unload_samples: Deque[tuple[float, float]] = deque()

    def _transition(self, phase: Phase, now: float, msg: str) -> None:
        if self.phase != phase:
            self.phase = phase
            self.phase_since = now
            self.messages.append(f"[{phase.value}] {msg}")

    def _fail(self, now: float, reason: str) -> None:
        self.error = reason
        self._transition(Phase.ERROR, now, reason)

    def update_wrench(self, force_z_n: float, now: float) -> None:
        if not math.isfinite(force_z_n):
            return
        self.force_z_n = force_z_n
        self.last_wrench_at = now
        if self.phase == Phase.TARE:
            self.tare_samples.append((now, force_z_n))
            self._trim(self.tare_samples, now, 0.65)
        elif self.phase == Phase.SETTLING and self.baseline_n is not None:
            value = (force_z_n - self.baseline_n) / G
            self.mass_samples.append((now, value))
            self._trim(self.mass_samples, now, 0.50)
        elif self.phase == Phase.TO_PICK and self.baseline_n is not None:
            self.unload_samples.append((now, (force_z_n - self.baseline_n) / G))
            self._trim(self.unload_samples, now, 0.50)

    def update_pose(self, x_m: float, y_m: float, now: float) -> None:
        if math.isfinite(x_m) and math.isfinite(y_m):
            self.pose_x_m = x_m
            self.pose_y_m = y_m
            self.last_pose_at = now

    @staticmethod
    def _trim(samples: Deque[tuple[float, float]], now: float, duration: float) -> None:
        while samples and samples[0][0] < now - duration:
            samples.popleft()

    def tick(self, now: float) -> float:
        """Return roller angular velocity; return zero for all fail-safe states."""
        c = self.cfg
        if self.phase in (Phase.DONE, Phase.ERROR):
            return 0.0

        if self.phase == Phase.TARE:
            if self.last_wrench_at is None:
                if now - self.phase_since > c.tare_timeout_s:
                    self._fail(now, f"Scale sensor has not published a wrench ({c.tare_timeout_s:.0f} s).")
                return 0.0
            if now - self.phase_since > c.tare_timeout_s:
                self._fail(now, f"Tare not completed within {c.tare_timeout_s:.0f} s "
                                f"({len(self.tare_samples)} samples in window); check scale stream and load.")
                return 0.0
            if len(self.tare_samples) >= 20:
                readings = [z for _, z in self.tare_samples]
                baseline = statistics.median(readings)
                if abs(baseline - c.expected_tare_n) > c.tare_tolerance_n:
                    self._fail(now, f"Empty-scale tare {baseline:.2f} N is outside expected range; clear the scale first.")
                elif statistics.pstdev(readings) < 0.8:
                    self.baseline_n = baseline
                    self._transition(Phase.WAIT_BOX, now, f"Tare = {baseline:.3f} N (empty platform)")
            return 0.0

        # Do not command rollers when sensor data goes stale.
        if self.last_wrench_at is None or now - self.last_wrench_at > c.sensor_timeout_s:
            self._fail(now, "Scale wrench stream stopped; safety stop.")
            return 0.0

        if self.phase == Phase.WAIT_BOX:
            # No pose yet is a normal wait (box not spawned); a stale pose must not start rollers.
            pose_fresh = self.last_pose_at is not None and now - self.last_pose_at <= c.start_pose_max_age_s
            if pose_fresh and self.pose_x_m is not None and -4.80 < self.pose_x_m < -4.10:
                if abs((self.pose_y_m or 0) - c.scale_y_m) > c.max_y_error_m:
                    self._fail(now, "Box is not on the conveyor centerline.")
                else:
                    self._transition(Phase.TO_SCALE, now, "Incoming box detected; start rollers")
                    return c.velocity_rad_s
            if now - self.phase_since > c.spawn_wait_timeout_s:
                if self.last_pose_at is None:
                    self._fail(now, "No incoming demo box position received.")
                elif not pose_fresh:
                    self._fail(now, "Demo box position stream is stale; not starting.")
                else:
                    self._fail(now, f"Demo box is not at the inlet: x={self.pose_x_m:.3f} m.")
            return 0.0

        if self.last_pose_at is None or now - self.last_pose_at > c.sensor_timeout_s:
            self._fail(now, "Box pose stream stopped; safety stop.")
            return 0.0

        assert self.pose_x_m is not None
        # Lane check applies for the whole transfer, not only at the inlet.
        if abs((self.pose_y_m or 0) - c.scale_y_m) > c.max_y_error_m:
            self._fail(now, f"Box left the conveyor lane: y={self.pose_y_m:.3f} m; safety stop.")
            return 0.0
        if self.phase == Phase.TO_SCALE:
            if self.pose_x_m >= c.scale_stop_x_m:
                self.mass_samples.clear()
                self._transition(Phase.SETTLING, now, f"x={self.pose_x_m:.3f} m; stop and stabilize")
                return 0.0
            if now - self.phase_since > c.travel_timeout_s:
                self._fail(now, "Incoming box failed to reach the scale.")
                return 0.0
            return c.velocity_rad_s

        if self.phase == Phase.SETTLING:
            if now - self.phase_since >= c.settle_timeout_s:
                self._fail(now, "Weighing did not stabilize; do not release box.")
                return 0.0
            if now - self.phase_since < c.settle_time_s:
                return 0.0
            # Must be fully supported on the scale rollers, not overlapping
            # the upstream/downstream non-scale rollers.
            if abs(self.pose_x_m - c.scale_center_x_m) > c.scale_center_tolerance_m:
                self._fail(now, f"Box stopped outside center region: x={self.pose_x_m:.3f} m")
                return 0.0
            if not self.mass_samples or now - self.mass_samples[-1][0] > 0.3:
                return 0.0
            values = [m for _, m in self.mass_samples]
            if len(values) < 15:
                return 0.0
            if statistics.pstdev(values) > c.mass_stdev_tolerance_kg:
                return 0.0
            measured = statistics.median(values)
            if measured < 0.5:
                return 0.0
            if c.expected_mass_kg is not None and abs(measured - c.expected_mass_kg) > c.expected_mass_tolerance_kg:
                self._fail(now, f"Measured mass {measured:.3f} kg differs from test fixture {c.expected_mass_kg:.3f} kg.")
                return 0.0
            self.measured_kg = measured
            self.unloaded = False
            self.unload_samples.clear()
            self._transition(Phase.TO_PICK, now, f"WEIGHED {measured:.3f} kg; resume conveyor")
            return c.velocity_rad_s

        if self.phase == Phase.TO_PICK:
            if now - self.phase_since > c.travel_timeout_s:
                self._fail(now, "Box failed to reach the PICK zone.")
                return 0.0
            if self.pose_x_m >= c.unload_x_m and len(self.unload_samples) >= 5:
                values = [m for _, m in self.unload_samples]
                residual = statistics.median(values)
                if abs(residual) < 0.25:
                    if not self.unloaded:
                        self.unloaded = True
                        self.unload_residual_kg = residual
                        self.messages.append(f"[TO_PICK] Scale unloaded; returned to zero (residual {residual:+.3f} kg)")
            if self.pose_x_m >= c.pick_x_m:
                if not self.unloaded:
                    self._fail(now, "Box reached PICK before scale returned to zero.")
                    return 0.0
                self._transition(Phase.DONE, now, f"PICK zone reached at x={self.pose_x_m:.3f}; stop")
                return 0.0
            return c.velocity_rad_s

        return 0.0
