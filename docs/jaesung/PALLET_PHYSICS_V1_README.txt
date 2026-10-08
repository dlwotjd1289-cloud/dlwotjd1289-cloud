PAC 2026 AHEAD - Pallet Physics V1 patch

Adds:
- Shapely-based pallet boundary / horizontal box collision / support geometry
- NumPy-based combined center of mass
- Four-quadrant projected mass-balance heuristic
- PalletPhysicsEngine and deterministic PhysicsReport
- pytest coverage for first-layer support, partial support rejection, collision,
  out-of-bound rejection, and balanced quadrant loads

Planning convention used by Physics V1:
- frame_id = pallet
- pallet x/y origin = pallet geometric center
- z = 0 = pallet top surface
- Pose3D position = box center

Important:
- min_support_ratio=0.80 is a provisional engineering parameter, NOT an
  official competition threshold. It is configurable via PhysicsLimits.
- quadrant_loads_kg is a projected mass-balance heuristic, NOT Gazebo contact
  force or structural stress.
- Structural multi-layer load propagation / support graph is intentionally V2.

Validation in scaffold: 12 pytest tests passed.
