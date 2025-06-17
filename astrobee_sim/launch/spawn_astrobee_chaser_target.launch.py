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

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),

		launch_arguments={
			'gz_args': [PathJoinSubstitution([ astrobee_gazebo_dir, 'worlds', "oos_cam.world"]),
                ' -r'
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
    arguments=['-file', [os.path.join(astrobee_description_dir, 'models/chaser/model.sdf')], '-z', '2.0', '-name', 'chaser'],
    prefix='gnome-terminal --tab --',
    output='screen'
    )

    gz_spawn_target = Node(
    package='ros_gz_sim',
    executable='create',
    name='astrobee_spawn_target',
    arguments=['-file', [os.path.join(astrobee_description_dir, 'models/target/model.sdf')],'-x', '5.5', '-z', '2.0', '-name', 'target'],
    prefix='gnome-terminal --tab --',
    output='screen'
    )


    ld.add_action(gz_spawn_chaser)

    ld.add_action(gz_spawn_target)

    chaser_urdf_path = PathJoinSubstitution([astrobee_description_dir, 'models', 'chaser', 'model.urdf.xacro'])
    target_urdf_path = PathJoinSubstitution([astrobee_description_dir, 'models', 'target', 'model.urdf.xacro'])

    chaser_description_content = ParameterValue(Command(['xacro ', chaser_urdf_path]), value_type=str)
    target_description_content = ParameterValue(Command(['xacro ', target_urdf_path]), value_type=str)

    chaser_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = 'chaser_robot_state_publisher',
        parameters=[{
            'robot_description': chaser_description_content,
            'tf_prefix': 'chaser/',
        }],
        remappings=[
            ('/robot_description', '/chaser/robot_description')
        ]
    )

    ld.add_action(chaser_robot_state_publisher_node)

    target_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = 'target_robot_state_publisher',
        parameters=[{
            'robot_description': target_description_content,
            'tf_prefix': 'target/',
        }],
        remappings=[
            ('/robot_description', '/target/robot_description')
        ]
    )

    ld.add_action(target_robot_state_publisher_node)


    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([astrobee_sim_dir, 'resource', 'rviz_config.rviz'])],
    ))


    return ld