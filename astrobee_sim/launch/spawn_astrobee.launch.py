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



def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    astrobee_gazebo_dir = get_package_share_directory('astrobee_gazebo')
    astrobee_description_dir = get_package_share_directory('astrobee_description')
    # gazebo_world_value = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': PathJoinSubstitution([
				astrobee_gazebo_dir,
				'worlds',
				"oos_cam.world", 
			]),
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)


    urdf_rviz = IncludeLaunchDescription(
            PathJoinSubstitution([get_package_share_directory('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'astrobee_description',
                'urdf_package_path': PathJoinSubstitution(['models','astrobee_freeflyer', 'model.urdf.xacro'])
            }.items()
        )

    
    gz_spawn = Node(
        package='ros_gz_sim',
		executable='create',
		name='astrobee_spawn',
        # parameters=[{'file': os.path.join(astrobee_description_dir, 'models/astrobee_freeflyer/model.urdf.xacro'), 'x': 0.0, 'y': 0.0, 'z': 20, 'Y': 0.0}],
        arguments=['-file', [os.path.join(astrobee_description_dir, 'models/astrobee_freeflyer/model.sdf')], '-z', '1.0'],
		prefix='gnome-terminal --tab --',
		output='screen'
    )

    bridge_twist = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['astrobee/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    output='screen'
)



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

    return LaunchDescription(
        [
            gz_sim, 
            gz_spawn,
            # urdf_rviz,
            bridge_twist,
            # robot_state_publisher_node,
        ]
    )
