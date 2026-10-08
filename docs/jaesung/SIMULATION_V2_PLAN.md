# Workcell V2
Included now: floor, roller-conveyor visuals, inline scale, fixed top-view CCTV + mast, pallet, 3-level buffer rack, safety fence, controller cabinet.
HDP160-31 is spawned later after description verification.

Initial observation mode: Ground Truth -> Observation Generator -> Observed State -> Planner. Later inject camera noise, missed detection, tracking loss, size/weight correction without changing planner contracts.

Conveyor: first end-to-end benchmark should use explicit deterministic conveyor motion rather than relying on roller-friction physics.
