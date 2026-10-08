# Runtime architecture
Sensor / Simulation -> Perception -> Observation Event -> StateManager -> immutable SystemState -> Candidate Generator -> Hard Constraint -> Score/Look-ahead -> Ranked Candidates -> RobotCapability Adapter -> HDP160-31 Robot Feasibility -> Execution -> ExecutionResult -> StateManager -> state_version+1

Rules:
- AHEAD core remains robot-agnostic.
- `pac_robot` owns robot-specific feasibility and controller adapters.
- only StateManager commits ACTUAL state.
- simulation rollouts use independent immutable snapshots.
