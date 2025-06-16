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
    # gazebo_world_value = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),

		launch_arguments={
			'gz_args': [PathJoinSubstitution([ astrobee_gazebo_dir, 'worlds', "empty.world"]),
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
			# 'qos_overrides./tf_static.publisher.durability': 'transient_local',
		}],
		prefix='gnome-terminal --tab --',
		output='screen'
	)

    ld.add_action(gz_sim)
    ld.add_action(ros_gz_bridge)

    # Iterate astrobees

    rotations_filepath = os.path.join(astrobee_sim_dir, 'resource', 'unstableRotation.yaml')

    with open(rotations_filepath, 'r') as f:
        rotations_data = yaml.safe_load(f)

        # Extract quaternions 
        orient_data = np.array(rotations_data[0]['x'])

    orient_data = np.reshape(orient_data, (-1, 7))
    quats = orient_data[:, 0:4]  # Extract the quaternion columns

    quats = np.vstack(([0,1,0,0], quats))  # Add the first quaternion as [0, 0, 0, 1]
       
    quats_list = quats.tolist()  # Convert to a list for easier iteration
    print(quats)

    id = 0
    for i in range(0, len(quats_list), 5):

    
        
        
        namespace = 'astrobee_' + str(id)

        orientation = quats_list[i]
        roll, pitch, yaw = euler_from_quaternion(orientation)



    #     # roll = -0.2 + i * 0.2  # Roll angle in radians
    #     roll = 0  # Roll angle in radians

        if i == 0:
            gz_spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='astrobee_spawn',
            # parameters=[{'file': os.path.join(astrobee_description_dir, 'models/astrobee_freeflyer/model.urdf.xacro'), 'x': 0.0, 'y': 0.0, 'z': 20, 'Y': 0.0}],
            arguments=['-file', [os.path.join(astrobee_description_dir, 'models/astrobee_unstable/model.sdf')], '-z', '1.0','-P', str(pitch), '-Y', str(yaw), '-R', str(roll), '-name', namespace],
            # arguments=['-file', [os.path.join(astrobee_description_dir, 'models/astrobee_unstable/model.sdf')], '-z', str(i), '-R', '0.5'],
            prefix='gnome-terminal --tab --',
            output='screen'
            )
            
        else:

            
            gz_spawn = Node(
                package='ros_gz_sim',
                executable='create',
                name='astrobee_spawn',
                # parameters=[{'file': os.path.join(astrobee_description_dir, 'models/astrobee_freeflyer/model.urdf.xacro'), 'x': 0.0, 'y': 0.0, 'z': 20, 'Y': 0.0}],
                arguments=['-file', [os.path.join(astrobee_description_dir, 'models/astrobee_unstable_transparent/model.sdf')],'-y', str(0.5*id), '-z', '1.0', '-P', str(pitch), '-Y', str(yaw), '-R', str(roll), '-name', namespace],
                # arguments=['-file', [os.path.join(astrobee_description_dir, 'models/astrobee_unstable/model.sdf')], '-z', str(i), '-R', '0.5'],
                prefix='gnome-terminal --tab --',
                output='screen'
            )

        urdf_rviz = IncludeLaunchDescription(
                PathJoinSubstitution([get_package_share_directory('urdf_launch'), 'launch', 'display.launch.py']),
                launch_arguments={
                    'urdf_package': 'astrobee_description',
                    'urdf_package_path': PathJoinSubstitution(['models','astrobee_unstable', 'model.urdf.xacro'])
                }.items()
            )


        rotate_robot = Node(
            package='astrobee_sim',
            executable='rotate_robot',
            name='rotate_robot' + str(id),
            output='screen',
            prefix='gnome-terminal --tab --',
            parameters=[{
                'angular_velocity': [0.1, -280.0, 0.1],  # Angular velocity in radians per second
                
            }],
        )

        ld.add_action(gz_spawn)
        ld.add_action(rotate_robot)

        id +=1

        bridge_string = namespace + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        bridge_twist = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[bridge_string],
        output='screen'
        )




#     current_nodes = GroupAction(
#      actions=[
#          PushRosNamespace(namespace),
#          urdf_rviz,
#          gz_spawn, 
#         #  bridge_twist, 
#          rotate_robot

#       ]
#    )
    


    # gz_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py')),
    #     launch_arguments={
    #         'entity': 'astrobee',
    #         'file': PathJoinSubstitution([
    #             astrobee_gazebo_dir,
    #             'models',
    #             'astrobee_freeflyer',
    #             'model.urdf.xacro'
    #         ]),
    #         'x': '0.0',
    #         'y': '0.0',
    #         'z': '0.5',
    #         'Y': '0.0'
    #     }.items(),
    # )
    
    # robot_names = ["astrobee1", "astrobee2", "astrobee3"]
    # orientations = [
    #     [0, 0, 0],
    #     [0, 0.1, 0],
    #     [0, 0, 0.1]
    # ]

    # spawn_nodes = []
    # for i, name in enumerate(robot_names):
    #     yaw = orientations[i][2]
    #     spawn_nodes.append(
    #         Node(
    #             package='ros_gz',
    #             executable='spawn_entity.py',
    #             arguments=[
    #                 '-entity', name,
    #                 '-x', str(0.5*i),
    #                 '-y', '0',
    #                 '-z', '0.5',
    #                 '-Y', str(yaw),
    #                 '-file', '/path/to/astrobee.urdf'
    #             ],
    #             output='screen'
    #         )
    #     )

    return ld