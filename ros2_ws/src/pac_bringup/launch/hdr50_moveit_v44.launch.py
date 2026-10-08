#!/usr/bin/env python3
# V4.4 MoveIt2 move_group for the Gazebo workcell (start after hdr50_workcell_v4_4_pick.launch.py).
# Same robot description as Gazebo (pedestal z=0.40 + suction cup, TCP = suction_tcp);
# planner / limit / controller YAML reused unchanged from hdr50_22_moveit_config.

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    bringup = get_package_share_directory("pac_bringup")
    moveit_pkg = "hdr50_22_moveit_config"

    robot_description = xacro.process_file(
        os.path.join(bringup, "urdf", "hdr50_pedestal_gripper.urdf.xacro"),
        mappings={
            "name": "hdr_robot",
            "use_sim": "true",
            "use_mock_hardware": "false",
            "hdr_ros2_control": os.path.join(
                get_package_share_directory("hdr_simulation_gz"), "config", "hdr_controllers.yaml"),
            "initial_positions_file": os.path.join(
                get_package_share_directory(moveit_pkg), "config", "initial_positions.yaml"),
        },
    ).toxml()
    robot_description_semantic = xacro.process_file(
        os.path.join(bringup, "urdf", "hdr50_22_suction.srdf.xacro")).toxml()

    planning = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": {"planning_plugin": "ompl_interface/OMPLPlanner",
                 **_yaml(moveit_pkg, "config/ompl_planning.yaml")},
        "pilz": {"planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
                 **_yaml(moveit_pkg, "config/pilz_industrial_motion_planner_planning.yaml")},
    }
    params = [
        {"robot_description": robot_description,
         "robot_description_semantic": robot_description_semantic,
         "publish_robot_description_semantic": True,
         "robot_description_kinematics": _yaml(moveit_pkg, "config/kinematics.yaml"),
         "robot_description_planning": {**_yaml(moveit_pkg, "config/joint_limits.yaml"),
                                        **_yaml(moveit_pkg, "config/pilz_cartesian_limits.yaml")}},
        planning,
        {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
         "moveit_simple_controller_manager": _yaml(moveit_pkg, "config/moveit_controllers.yaml"),
         "moveit_manage_controllers": False,
         "trajectory_execution.allowed_execution_duration_scaling": 1.5,
         "trajectory_execution.allowed_goal_duration_margin": 1.0,
         # Position-controlled Gazebo joints lag slightly at segment ends (~0.01 rad observed).
         "trajectory_execution.allowed_start_tolerance": 0.02,
         "trajectory_execution.execution_duration_monitoring": False},
        {"publish_planning_scene": True, "publish_geometry_updates": True,
         "publish_state_updates": True, "publish_transforms_updates": True},
        {"use_sim_time": True},
    ]

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with MoveIt plugin"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=params),
        Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
             arguments=["-d", os.path.join(get_package_share_directory(moveit_pkg), "config", "moveit.rviz")],
             parameters=params[:2] + [{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
