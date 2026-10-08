#!/usr/bin/env python3
"""pac2026-ahead workcell with the HDR50-22 on a 0.5 m pedestal (taehyeon).

Same world, robot description and controllers as pac2026-ahead's
``pac_bringup/hdr50_workcell.launch.py`` (that file is not changed); only the
robot spawn pose differs and a static pedestal is added, so the robot reaches
the whole 1.2 x 1.0 pallet up to the 1.35 m stacking height
(``config/taehyeon/robot_check_gazebo.yaml`` uses the same pose).

Run from a terminal with the pac2026-ahead workspace sourced:
    ros2 launch ~/pac-mission1-shared/tools/runtime/launch/hdr50_pedestal_workcell.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BASE_X, BASE_Y, BASE_Z = 1.35, 0.15, 0.5   # must match robot_check_gazebo.yaml
PEDESTAL = (0.6, 0.6)


def generate_launch_description():
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    sim_share = get_package_share_directory("pac_simulation")
    hdr_description_share = get_package_share_directory("hdr_description")
    hdr_hw_share = get_package_share_directory("hdr_hardware_interface")
    world = os.path.join(sim_share, "worlds", "ahead_workcell_v2_hdp160.sdf")
    controllers_file = PathJoinSubstitution([FindPackageShare("hdr_simulation_gz"), "config", "hdr_controllers.yaml"])
    initial_positions = PathJoinSubstitution([FindPackageShare("hdr50_22_moveit_config"), "config",
                                             "initial_positions.yaml"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 4 {world}"}.items())

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("hdr_description"), "urdf", "hdr.urdf.xacro"]),
        " use_sim:=true", " use_mock_hardware:=false", " robot_model:=hdr50_22", " name:=hdr",
        " hdr_ros2_control:=", controllers_file, " initial_positions_file:=", initial_positions,
    ])

    sx, sy = PEDESTAL
    pedestal_sdf = (
        '<sdf version="1.6"><model name="robot_pedestal"><static>true</static><link name="link">'
        f'<collision name="c"><geometry><box><size>{sx} {sy} {BASE_Z}</size></box></geometry></collision>'
        f'<visual name="v"><geometry><box><size>{sx} {sy} {BASE_Z}</size></box></geometry>'
        "<material><ambient>0.3 0.3 0.33 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material></visual>"
        "</link></model></sdf>")
    spawn_pedestal = Node(package="ros_gz_sim", executable="create", output="screen", arguments=[
        "-string", pedestal_sdf, "-name", "robot_pedestal",
        "-x", str(BASE_X), "-y", str(BASE_Y), "-z", str(BASE_Z / 2)])
    spawn_robot = Node(package="ros_gz_sim", executable="create", output="screen", arguments=[
        "-string", robot_description, "-name", "hdr50_22",
        "-x", str(BASE_X), "-y", str(BASE_Y), "-z", str(BASE_Z), "-allow_renaming", "false"])

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(hdr_hw_share, "launch", "ros2_control.launch.py")),
        launch_arguments={
            "robot_model": "hdr50_22", "use_sim": "true", "use_mock_hardware": "false",
            "initial_positions_file": initial_positions,
            "controllers_config_package": "hdr_simulation_gz", "controllers_file": "hdr_controllers.yaml",
        }.items())

    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"])

    return LaunchDescription([
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=hdr_description_share),
        gazebo,
        ros2_control,
        spawn_pedestal,
        TimerAction(period=2.0, actions=[spawn_robot]),
        clock_bridge,
    ])
