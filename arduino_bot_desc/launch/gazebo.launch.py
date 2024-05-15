from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    arduino_bot_desc=get_package_share_directory("arduino_bot_desc")
    arduino_desc_prefix= get_package_prefix("arduino_bot_desc")

    model_path=os.path.join(arduino_bot_desc, 'models')
    model_path += pathsep + os.path.join(arduino_desc_prefix,'share')

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(arduino_bot_desc, 'urdf', 'arduinobot.urdf.xacro'),
        description="Absolute path to the URDF model file"
    )
    
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server_cmd = ExecuteProcess(
       cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            
        ],        
        output='screen',
    )
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=[
            'gzclient',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            
        ],
        output='screen',
       # arguments=[os.path.join(get_package_prefix("my_robot_bringup"), "share","my_world.world")],
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-entity","arduino_bot","-topic","robot_description"],
        output='screen',
                
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
    ])