# PAC2026 AHEAD Live Physics Simulator

## Purpose

This patch adds a standalone, always-running rigid-body pallet simulator for
AHEAD algorithm validation.

- Pallet footprint: **1100 × 1100 mm**
- Maximum stack height: configurable
- Physics: **PyBullet**
- Contact/support graph: **NetworkX**
- Browser dashboard/server: **aiohttp + WebSocket**
- 3D display: **Three.js**
- Existing Shapely/NumPy `PalletPhysicsEngine` remains the fast candidate
  evaluator; this simulator is the live physical execution world after AHEAD
  chooses a placement.

The simulator does **not** contain rules such as
`if support_ratio < X: make_box_fall()`.
Once a box is placed in Bullet, gravity/contact/friction/collision determine
what happens.

## Files added

```text
config/ahead_simulator.yaml
requirements-ahead-sim.txt
scripts/run_ahead_simulator.py

ros2_ws/src/pac_simulation/pac_simulation/ahead_sim/
├── __init__.py
├── config.py
├── models.py
├── world.py
├── metrics.py
├── contact_graph.py
├── simulator.py
└── server.py

viewer/ahead_live/
├── index.html
├── app.js
└── style.css
```

## Install

Terminal 4:

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
python3 -m pip install --user -r requirements-ahead-sim.txt
```

Do not upgrade the project's NumPy just for this simulator.

## Run

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
python3 scripts/run_ahead_simulator.py
```

Browser:

```text
http://127.0.0.1:4173
```

Physics starts immediately and keeps running.  The buttons only control the
demo scenario; they do not turn physics on/off.

### Override stack height

```bash
python3 scripts/run_ahead_simulator.py --max-height 1.80
```

`1.80` above is only an example.

## Demo

Click `Auto demo sequence`.

B001–B005 are ordinary placements.  `B006_UNSTABLE` is intentionally placed
poorly.  There is no scripted collapse condition: PyBullet determines the
motion from rigid-body dynamics.

The right side shows:

- total/on-pallet mass
- actual stack height
- remaining configured height
- volume utilization
- actual combined CoM
- moving boxes
- boxes outside the pallet footprint
- maximum observed tilt
- pallet contact-force heatmap
- equivalent vertically supported mass
- contact/load graph information per box

## AHEAD integration

After AHEAD selects one placement, send the selected placement to:

```text
POST http://127.0.0.1:4173/api/place
```

Example:

```json
{
  "id": "B101",
  "size_m": [0.40, 0.30, 0.20],
  "mass_kg": 7.2,
  "target_position_m": [0.10, -0.20, 0.10],
  "yaw_rad": 0.0,
  "source": "ahead"
}
```

The body is created at the requested pose plus a 2 mm configurable contact
clearance, then Bullet determines its actual settled pose.

The same call is also available directly in Python:

```python
sim.place_mapping({
    "id": "B101",
    "size_m": [0.40, 0.30, 0.20],
    "mass_kg": 7.2,
    "target_position_m": [0.10, -0.20, 0.10],
    "yaw_rad": 0.0,
})
```

## Important calibration note

`box_lateral_friction`, `pallet_lateral_friction`, restitution and the other
material parameters in `config/ahead_simulator.yaml` are explicitly starting
values for simulation development, not measured carton/pallet constants.
Calibrate them against the actual box/pallet material before using dynamic
results as quantitative evidence.

The current rigid-body model handles:

- settling
- sliding
- tipping
- falling
- collision
- cascading motion
- contact-force distribution

It does not model cardboard crushing, buckling, tearing or other deformable
material failure.
