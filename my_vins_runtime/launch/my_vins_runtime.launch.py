from launch import LaunchDescription as ld
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

from pkg_resources import declare_namespace
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction


package_name = "my_vins_runtime"
shared_dir = get_package_share_directory(package_name)
path_code = os.path.join(shared_dir, "../../../../src/my_vins", "my_vins_runtime")

my_vins_shared_dir = get_package_share_directory("my_vins")
front_end_shared_dir = get_package_share_directory("my_vins_frontend")

bag_file_arg = '/root/workspace/data/Euroc/MH_01_easy'
# bag_file_arg = "/root/workspace/data/lc_calib/ahu_material_building_outdoor"
rviz_file = os.path.join(path_code, "config/rviz.rviz")

def generate_launch_description():
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_arg , '--clock'],
        output='screen'
    )

    include_back_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_vins_shared_dir, 'launch', 'back_end.launch.py'))
    )
    include_front_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(front_end_shared_dir, 'launch', 'front_end.launch.py'))
    )

    return ld([
        rviz,
        include_back_end,
        include_front_end,
        bag_play,
    ])