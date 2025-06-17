from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.substitutions import Command

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from launch_ros.parameter_descriptions import ParameterValue

import yaml

import numpy as np





def generate_launch_description():

    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    astrobee_gazebo_dir = get_package_share_directory('astrobee_gazebo')
    astrobee_description_dir = get_package_share_directory('astrobee_description')

    astrobee_sim_dir = get_package_share_directory('astrobee_sim')

    

    execute_trajectory_node = Node(
        package='astrobee_sim',
        executable='execute_trajectory',
        name='execute_trajectory',
        output='screen',
        parameters=[os.path.join(astrobee_sim_dir, 'resource', 'trajectory_params.yaml'), 
                    {'robot_name': 'chaser', 'body_frame': 'chaser/base_link', 'robot_number': 0}],
        
    )

    ld.add_action(execute_trajectory_node)

    return ld