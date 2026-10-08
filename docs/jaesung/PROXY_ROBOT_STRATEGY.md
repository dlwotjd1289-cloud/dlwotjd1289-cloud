# Robot proxy strategy

## Final target vs simulation proxy

- Final target: `HDP160-31`
- Current simulation proxy: `HDR50-22`
- The proxy is NOT considered final HDP160-31 robot feasibility.

The public HD Hyundai ROS 2 Humble repositories provide HDR50-22 description,
MoveIt configuration and Gazebo integration, while no HDP160-31 files were found
in public Humble/Jazzy/Main branches checked by the team.

## Why HDR50-22 is constrained like a palletizer

HDR50-22 has 6 DOF. HDP160-31 is a 4-axis palletizer. AHEAD therefore sends only:

```text
x, y, z, yaw
```

The HDR50-22 simulation adapter fixes roll/pitch by policy and uses vertical
approach/retreat. Extra 6-DOF freedom must not leak into AHEAD candidate logic.

## EOAT independence

`pac_eoat` defines `vacuum_gripper_v1` independently of the robot. The verified
HDR50-22 Humble description exposes a fixed `flange` link, so the simulation
wrapper may mount the EOAT there. When HDP160-31 becomes available, only the
mount parent/transform and robot adapter should change.

No real EOAT mass, CoM, vacuum capacity, or maximum workpiece mass is fabricated
in this scaffold. Those fields remain null until verified hardware is selected.
