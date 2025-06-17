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

    
    rotate_target = Node(
            package='astrobee_sim',
            executable='rotate_robot',
            name='rotate_robot',
            output='screen',
            prefix='gnome-terminal --tab --',
            parameters=[{
                'angular_velocity': [0.001, 0.0698, 0.001],  # Angular velocity in radians per second
                'robot_name': 'target',
                'body_frame': 'target/base_link',
                'robot_number': 1,
    
            }],
        )
    ld.add_action(rotate_target)

    
    return ld