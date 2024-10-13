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


package_name = 'my_vins_frontend'

def generate_launch_description():

    node_front = Node(
        name=package_name,
        package=package_name,
        executable="spsg",
        # parameters=[config_param],
        output='screen',
        # prefix=["gdbserver localhost:3001"] if n == 2 else None,
        # logging=logging_config
        # arguments=['--ros-args', '--log-level', package_name +':=' + str_info_level]
    )
    return ld([
        node_front
    ])
