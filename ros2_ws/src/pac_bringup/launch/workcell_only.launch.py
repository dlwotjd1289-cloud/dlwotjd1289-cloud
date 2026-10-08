import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ros_gz_share=get_package_share_directory('ros_gz_sim')
    sim_share=get_package_share_directory('pac_simulation')
    world=os.path.join(sim_share,'worlds','ahead_workcell_v2_hdp160.sdf')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share,'launch','gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r {world}'}.items())])
