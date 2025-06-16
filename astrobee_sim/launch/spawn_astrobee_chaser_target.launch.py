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
import yaml

import numpy as np





def generate_launch_description():

    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    astrobee_gazebo_dir = get_package_share_directory('astrobee_gazebo')
    astrobee_description_dir = get_package_share_directory('astrobee_description')

    astrobee_sim_dir = get_package_share_directory('astrobee_sim')

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),

		launch_arguments={
			'gz_args': [PathJoinSubstitution([ astrobee_gazebo_dir, 'worlds', "oos_cam.world"]),
                # ' -r'
                ],
			'on_exit_shutdown': 'True', 
            'use_sim_time': 'True',
            
		}.items(),
	)

    # Bridge ROS topics and Gazebo messages for establishing communication
    ros_gz_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		parameters=[{
			'config_file': os.path.join(astrobee_sim_dir, 'resource', 'astrobee_sim_bridge.yaml'),
		}],
		prefix='gnome-terminal --tab --',
		output='screen'
	)

    ld.add_action(gz_sim)
    ld.add_action(ros_gz_bridge)


    gz_spawn_chaser = Node(
    package='ros_gz_sim',
    executable='create',
    name='astrobee_spawn_chaser',
    arguments=['-file', [os.path.join(astrobee_description_dir, 'models/chaser/model.sdf')], '-z', '1.0', '-name', 'chaser'],
    prefix='gnome-terminal --tab --',
    output='screen'
    )

    gz_spawn_target = Node(
    package='ros_gz_sim',
    executable='create',
    name='astrobee_spawn_target',
    arguments=['-file', [os.path.join(astrobee_description_dir, 'models/target/model.sdf')],'-x', '5.5', '-z', '1.0', '-name', 'target'],
    prefix='gnome-terminal --tab --',
    output='screen'
    )
    


    ld.add_action(gz_spawn_chaser)
    ld.add_action(gz_spawn_target)


    chaser_urdf_rviz = IncludeLaunchDescription(
                PathJoinSubstitution([get_package_share_directory('urdf_launch'), 'launch', 'display.launch.py']),
                launch_arguments={
                    'urdf_package': 'astrobee_description',
                    'urdf_package_path': PathJoinSubstitution(['models','astrobee_unstable', 'model.urdf.xacro'])
                }.items()
            )

    return ld