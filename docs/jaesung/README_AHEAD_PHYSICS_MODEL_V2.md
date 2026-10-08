# AHEAD Physics Model V2 patch

This patch strengthens the algorithm-validation physics model in three areas.

## 1. Pallet collision geometry

The old PyBullet pallet was one solid 1100 x 1100 mm collision box.

The new default `collision_model: slatted` uses one compound static Bullet body:
- top deck boards,
- support blocks,
- bottom runners.

The geometry is configurable in `config/ahead_simulator.yaml`.

**Important:** the default board/block ratios are development placeholders.
They are not claimed to be the official PAC/HD Hyundai pallet specification.
When the actual pallet CAD/dimensions are known, replace the config values.

If you need the previous behavior:

```yaml
pallet:
  collision_model: "solid"
```

## 2. Friction model / calibration

The config now uses **target combined contact coefficients**:

```yaml
target_box_box_friction: 0.64
target_box_pallet_friction: 0.72
```

Bullet combines body frictions by multiplication, so the simulator converts
these targets to the appropriate Bullet body friction values.

The default targets preserve the current development setup:
- box body = 0.8
- pallet body = 0.9
- box-box = 0.8 x 0.8 = 0.64
- box-pallet = 0.8 x 0.9 = 0.72

These are still development values, not measured material constants.

A simple physical tilt-table test can later estimate static friction:

```bash
python3 scripts/calibrate_friction_from_tilt.py 25.0
```

where 25.0 is the measured critical sliding angle.

## 3. Optional box compression/top-load limit

PyBullet boxes remain rigid bodies.  Cardboard crushing is not simulated.

Instead, each box may carry an experimentally/manufacturer-derived top-load
limit:

```json
{
  "id": "B101",
  "size_m": [0.40, 0.30, 0.20],
  "mass_kg": 8.0,
  "target_position_m": [0.0, 0.0, 0.10],
  "max_top_load_n": 1200.0
}
```

or for convenience:

```json
"max_supported_load_kg": 122.36
```

Internally the simulator uses Newtons.

Actual PyBullet contact forces are averaged over a configurable simulation-time
window (default 0.5 s), then compared against the supplied limit.

If no strength value is provided, the result is `UNKNOWN`, not guessed.

## Other fixes included

- preserves the initial-overlap rejection added during debugging;
- duplicate contact points are summarized by body in the overlap error;
- simulation time is now based on Bullet physics steps rather than wall clock;
- load reporting uses rolling contact-force averages to reduce solver jitter.

## Apply

Stop the current simulator first (`Ctrl+C` in Terminal 4), then:

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
unzip -o \
~/다운로드/PAC2026_ahead_physics_model_v2_patch.zip \
-d .
```

Check:

```bash
python3 scripts/check_physics_model_v2.py
```

Then run:

```bash
python3 scripts/run_ahead_simulator.py
```
