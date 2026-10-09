"""Start the AHEAD runtime node (stages 1-8 decisions) with the team configs.

    source scripts/env.sh          # sets PAC_REPO to this checkout
    ros2 launch pac_runtime runtime.launch.py order_file:=/path/order.json ranker:=donghan
    # without env.sh: add repo:=/path/to/this/checkout
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    cfg = lambda name: PathJoinSubstitution([repo, "config", "taehyeon", name])  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("repo", default_value=EnvironmentVariable("PAC_REPO", default_value=""),
                              description="monorepo checkout (config/); default $PAC_REPO from scripts/env.sh"),
        DeclareLaunchArgument("order_file", description="order list JSON (see pac_runtime/order.py)"),
        DeclareLaunchArgument("policy", default_value="rule"),
        DeclareLaunchArgument("ranker", default_value="dblf",
                              description="low-level ordering: dblf or donghan"),
        DeclareLaunchArgument("ranker_model_path", default_value="",
                              description="optional validated donghan model; empty uses deterministic heuristic"),
        DeclareLaunchArgument("ranker_config", default_value="",
                              description="matching Donghan planner config (horizon/scenario contract)"),
        DeclareLaunchArgument("ranker_seed", default_value="7"),
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
                "ranker": LaunchConfiguration("ranker"),
                "ranker_model_path": LaunchConfiguration("ranker_model_path"),
                "ranker_config": LaunchConfiguration("ranker_config"),
                "ranker_seed": LaunchConfiguration("ranker_seed"),
            }],
        ),
    ])
