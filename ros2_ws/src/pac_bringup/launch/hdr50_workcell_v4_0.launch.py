#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ros_gz_share = get_package_share_directory("ros_gz_sim")
    sim_share = get_package_share_directory("pac_simulation")
    hdr_description_share = get_package_share_directory("hdr_description")

    world = os.path.join(
        sim_share,
        "worlds",
        "ahead_workcell_v4_0_physical_rollers.sdf",
    )

    controllers_file = PathJoinSubstitution([
        FindPackageShare("hdr_simulation_gz"),
        "config",
        "hdr_controllers.yaml",
    ])

    initial_positions = PathJoinSubstitution([
        FindPackageShare("hdr50_22_moveit_config"),
        "config",
        "initial_positions.yaml",
    ])

    pedestal_xacro = PathJoinSubstitution([
        FindPackageShare("pac_bringup"),
        "urdf",
        "hdr50_pedestal.urdf.xacro",
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        pedestal_xacro,
        " use_sim:=true",
        " use_mock_hardware:=false",
        " name:=hdr_robot",
        " hdr_ros2_control:=", controllers_file,
        " initial_positions_file:=", initial_positions,
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world}"
        }.items(),
    )

    # Robot world_joint itself contains Z=0.40.
    # Therefore spawn pose remains zero.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "hdr50_22",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-allow_renaming", "false",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    controllers = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster,
            trajectory_controller,
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=hdr_description_share,
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        controllers,
        clock_bridge,
    ])
