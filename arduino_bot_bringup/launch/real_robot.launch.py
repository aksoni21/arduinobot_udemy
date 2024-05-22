from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduino_bot_controller"),
            "launch", 
            "controller.launch.py"),
        launch_arguments={"is_sim":"False"}.items()
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduino_bot_misa"),
            "launch", 
            "moveit.launch.py"),
        launch_arguments={"is_sim":"False"}.items()
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduino_bot_remote"),
            "launch", 
            "remote.launch.py"),
    )

    return LaunchDescription([
        controller,
        moveit,
        remote_interface,
    ])
