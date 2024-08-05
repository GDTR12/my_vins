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
app_path = os.path.join(shared_dir, "../../../../src/"+ package_name +"/src/app")
node_files = getFiles(app_path)
nodes_name = []

print("Select file:")
for i in range(len(node_files)):
    file_name = os.path.splitext(os.path.basename(node_files[i]))[0]
    print("%d) %s  " % (i, file_name), end="")
    nodes_name.append(file_name)

print('')
while True:
    node_idx = int(input()) 
    if (node_idx >= len(nodes_name)):
        print("wrong option: %d. Resume" % node_idx)
    else:
        break

node_str = nodes_name[node_idx]
print(node_str)

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

# python_path = sys.executable
# print(f"Python interpreter path: {python_path}")

def generate_launch_description():

    config_param = os.path.join(
        get_package_share_directory(package_name),
        'config',
        node_str + ".yaml"
    )

    rviz_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'rviz.rviz2'
    )
    # osm_file = DeclareLaunchArgument("osm_file", default_value="/root/slam/slam/osm/osm-nav/src/osm_navigation/osm_file/map.osm")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_file]
    )
    node = Node(
        name=package_name,
        package=package_name,
        executable=node_str,
        parameters=[config_param],
        output='screen',
        prefix=["gdbserver localhost:3000"] if n == 2 else None,
        # logging=logging_config
        arguments=['--ros-args', '--log-level', package_name +':=' + str_info_level]
    )


    return ld(
        [

            node
        ]
    )

