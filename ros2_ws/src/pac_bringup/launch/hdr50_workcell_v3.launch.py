#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ros_gz_share = get_package_share_directory('ros_gz_sim')
    sim_share = get_package_share_directory('pac_simulation')
    hdr_description_share = get_package_share_directory('hdr_description')
    hdr_hw_share = get_package_share_directory('hdr_hardware_interface')

    world = os.path.join(
        sim_share,
        'worlds',
        'ahead_workcell_v3_hdr50.sdf'
    )

    controllers_file = PathJoinSubstitution([
        FindPackageShare('hdr_simulation_gz'),
        'config',
        'hdr_controllers.yaml'
    ])

    initial_positions = PathJoinSubstitution([
        FindPackageShare('hdr50_22_moveit_config'),
        'config',
        'initial_positions.yaml'
    ])

    # 1. PAC Workcell V2 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world}'
        }.items()
    )

    # 2. HDR50-22 URDF for Gazebo spawn
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('hdr_description'),
            'urdf',
            'hdr.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_mock_hardware:=false',
        ' robot_model:=hdr50_22',
        ' name:=hdr',
        ' hdr_ros2_control:=', controllers_file,
        ' initial_positions_file:=', initial_positions,
    ])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', 'hdr50_22',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-allow_renaming', 'false',
        ],
    )

    # 3. robot_state_publisher + controller spawners
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hdr_hw_share,
                'launch',
                'ros2_control.launch.py'
            )
        ),
        launch_arguments={
            'robot_model': 'hdr50_22',
            'use_sim': 'true',
            'use_mock_hardware': 'false',
            'initial_positions_file': initial_positions,
            'controllers_config_package': 'hdr_simulation_gz',
            'controllers_file': 'hdr_controllers.yaml',
        }.items(),
    )

    # 4. Gazebo simulation clock -> ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=hdr_description_share
        ),
        gazebo,
        ros2_control,
        spawn_robot,
        clock_bridge,
    ])
