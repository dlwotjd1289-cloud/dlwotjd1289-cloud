# HDP160-31 integration plan

> **2026-10-08 팀 결정:** 목표 로봇은 HDR50-22입니다 (HDP160-31은 보관). 이 문서는 이전 계획 기록입니다.

## Gate 0: do not fabricate
Do not copy kinematics from another HDR model. Do not infer joint origins, inertias, collision geometry, tool CoM, or torque limits.

## Gate 1: official references
Fetch `hdr_description`, `hdr_ros2_driver`, `hdr_simulation_gz` and run `scripts/check_hdp160_support.sh`.

## Gate 2: verified description
If HDP160-31 assets exist, verify joint names/limits, visual mesh, collision mesh, base/gripper frames and zero pose in RViz. If not, obtain verified CAD/kinematics from official or competition-provided assets before enabling the backend.

## Gate 3: MoveIt / Gazebo
Create HDP160-31 MoveIt config + ros2_control config, spawn into `ahead_workcell_v2_hdp160.sdf`, attach EOAT separately, keep fixed camera independent.

## Gate 4: RobotCapability Adapter
Check state version, tool/workpiece load inputs, reachability, IK, collision, approach, place, retreat. Nominal payload alone never decides validity.

## 4-DOF implication
Map AHEAD placement candidates primarily through x/y/z/yaw instead of assuming arbitrary 6-DOF tool orientation.
