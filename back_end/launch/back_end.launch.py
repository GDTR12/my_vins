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

def getFiles(dir, file_type=['.*'], exclude_file_type=[''], recursive=False):
    all_files = []
    if (recursive == True):
        for root, dirs, files in os.walk(dir):
            for file in files:
                f = os.path.splitext(file)
                if f[1] in exclude_file_type:
                    continue
                if ('.*' in file_type or f[1] in file_type):
                    full_path = os.path.join(root, file)
                    all_files.append(full_path)
    else:
        files = [f for f in os.listdir(dir) if os.path.isfile(os.path.join(dir, f))]
        for file in files:
            f = os.path.splitext(file)
            if f[1] in exclude_file_type:
                continue
            if ('.*' in file_type or f[1] in file_type):
                full_path = os.path.join(dir, file)
                all_files.append(full_path)
    return all_files




package_name = "my_vins"
shared_dir = get_package_share_directory(package_name)
app_path = os.path.join(shared_dir, "../../../../src/" + package_name + "/back_end/src/app")
path_code = os.path.join(shared_dir, "../../../../src/", package_name + "/back_end")
node_files = getFiles(app_path)
nodes_name = []



while True:
    n = int(input("Run(1) or Debug(2):"))
    if(n != 1 and n != 2):
        print("Wrong option:%d Resume" % n)
    else:
        break

str_info_level = ""
while True:
    level = int(input("Info level: Info(1) or Debug(2):"))
    if(level != 1 and level != 2):
        print("Wrong option:%d Resume" % level)
    else:
        if(level == 1):
            str_info_level = "info"
        elif(level == 2):
            str_info_level = "debug"
        break


def generate_launch_description():

    rviz_file = os.path.join(path_code, "config/rviz.rviz")
    config_param = os.path.join(path_code, "config/config.yaml")


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
    )

    bag_file_arg = '/root/workspace/data/Euroc/MH_01_easy',
    # bag_file_arg = "/root/workspace/data/lc_calib/ahu_material_building_outdoor"

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_arg , '--clock'],
        output='screen'
    )

    node_back = Node(
        name=package_name,
        package=package_name,
        executable='back_end',
        parameters=[config_param],
        output='screen',
        prefix=["gdbserver localhost:3000"] if n == 2 else None,
        # logging=logging_config
        arguments=['--ros-args', '--log-level', package_name +':=' + str_info_level]
    )

    node_front = Node(
        name=package_name,
        package=package_name,
        executable="front_end",
        parameters=[config_param],
        output='screen',
        prefix=["gdbserver localhost:3001"] if n == 2 else None,
        # logging=logging_config
        arguments=['--ros-args', '--log-level', package_name +':=' + str_info_level]
    )

    return ld(
        [
            # rviz,
            node_back,
            # node_front,
            # bag_play,
        ]
    )

