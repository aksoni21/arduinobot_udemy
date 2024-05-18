from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    task_server_node=Node(
        package='arduino_bot_remote',
        executable='task_server_node',
    )
    
    alexa_interface_node=Node(
        package='arduino_bot_remote',
        executable='alexa_interface.py',
    )
    
    
    return LaunchDescription([
        # task_server_node,
        alexa_interface_node,
    ])