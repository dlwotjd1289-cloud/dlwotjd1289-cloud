#!/usr/bin/env python3
"""Generate the HDR50-22 (pedestal + suction cup + suction_tcp) URDF for the PyBullet live simulator.

Same robot description as Gazebo/MoveIt (pac_bringup hdr50_pedestal_gripper.urdf.xacro); mesh
`file://` URIs are turned into plain absolute paths (PyBullet) and the per-box Gazebo gripper
plugins are dropped (stack_box_count:=0). Output: models/pybullet/hdr50_22_suction.urdf
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  python3 scripts/make_pybullet_robot_urdf_v44.py
"""
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "models" / "pybullet" / "hdr50_22_suction.urdf"


def main():
    xacro_file = os.path.join(get_package_share_directory("pac_bringup"), "urdf", "hdr50_pedestal_gripper.urdf.xacro")
    init = os.path.join(get_package_share_directory("hdr50_22_moveit_config"), "config", "initial_positions.yaml")
    urdf = subprocess.run(["xacro", xacro_file, "name:=hdr_robot", "use_sim:=true", "use_mock_hardware:=false",
                           "hdr_ros2_control:=none", f"initial_positions_file:={init}", "stack_box_count:=0"],
                          check=True, capture_output=True, text=True).stdout
    urdf = urdf.replace("file://", "")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(urdf)
    print(OUT)


if __name__ == "__main__":
    main()
