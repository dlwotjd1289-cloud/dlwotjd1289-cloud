"""Start the AHEAD runtime node (stages 1-8 decisions) with taehyeon's configs.

    ros2 launch pac_runtime runtime.launch.py repo:=/path/to/pac-mission1-shared order_file:=/path/order.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    cfg = lambda name: PathJoinSubstitution([repo, "config", "taehyeon", name])  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("repo", description="pac-mission1-shared checkout (for config/taehyeon)"),
        DeclareLaunchArgument("order_file", description="order list JSON (see pac_runtime/order.py)"),
        DeclareLaunchArgument("policy", default_value="rule"),
        DeclareLaunchArgument("policy_file", default_value=""),
        Node(
            package="pac_runtime",
            executable="runtime_node",
            output="screen",
            parameters=[{
                "order_file": LaunchConfiguration("order_file"),
                "candidates_config": cfg("candidates.yaml"),
                "highlevel_config": cfg("highlevel.yaml"),
                "runtime_config": cfg("runtime.yaml"),
                "robot_config": cfg("robot_check.yaml"),
                "policy": LaunchConfiguration("policy"),
                "policy_file": LaunchConfiguration("policy_file"),
            }],
        ),
    ])
