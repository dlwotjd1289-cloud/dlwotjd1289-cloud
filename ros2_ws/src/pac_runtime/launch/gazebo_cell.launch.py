"""Runtime node + Gazebo cell driver for the pac2026-ahead HDR50-22 workcell.

Start the Gazebo workcell first (pac2026-ahead workspace, other terminal):
    ros2 launch pac_bringup hdr50_workcell.launch.py
then (this workspace):
    ros2 launch pac_runtime gazebo_cell.launch.py repo:=$HOME/pac-mission1-shared

Both nodes use config/taehyeon/robot_check_gazebo.yaml (robot at the world
origin, pallet centre (1.35, -1.0)), so the joint values match the robot in
Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    cfg = lambda name: PathJoinSubstitution([repo, "config", "taehyeon", name])  # noqa: E731
    order = LaunchConfiguration("order_file")
    return LaunchDescription([
        DeclareLaunchArgument("repo", description="pac-mission1-shared checkout (for config/taehyeon)"),
        DeclareLaunchArgument("order_file", default_value=PathJoinSubstitution(
            [LaunchConfiguration("repo"), "config", "taehyeon", "example_order.json"])),
        DeclareLaunchArgument("speed_scale", default_value="0.3"),
        DeclareLaunchArgument("seed", default_value="0"),
        Node(package="pac_runtime", executable="runtime_node", output="screen", parameters=[{
            "order_file": order,
            "candidates_config": cfg("candidates.yaml"),
            "highlevel_config": cfg("highlevel.yaml"),
            "runtime_config": cfg("runtime.yaml"),
            "robot_config": cfg("robot_check_gazebo.yaml"),
            "policy": "rule",
        }]),
        Node(package="pac_runtime", executable="gazebo_cell", output="screen", parameters=[{
            "order_file": order,
            "robot_config": cfg("robot_check_gazebo.yaml"),
            "speed_scale": LaunchConfiguration("speed_scale"),
            "seed": LaunchConfiguration("seed"),
        }]),
    ])
