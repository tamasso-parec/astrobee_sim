import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

	

	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	pkg_project_description = get_package_share_directory('ros_gz_example_description')

	pkg_traj = get_package_share_directory('traj')
	
	ros_gz_example_gazebo_dir = get_package_share_directory('ros_gz_example_gazebo')

	gz_model_value = "x500_realsense"

	sdf_file  =  os.path.join(pkg_project_description, 'models', gz_model_value, 'model.sdf')
	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()


	
	world_name = 'default_custom.sdf'
	world_value = PathJoinSubstitution([
				ros_gz_example_gazebo_dir,
				'worlds',
				world_name])



	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': world_value,
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)

	spawn_entity = Node(
		package='ros_gz_sim',
		executable='create',
		name='spawn_entity',
		output='screen',
		arguments=[
			'-file', sdf_file,
			'-name', gz_model_value,
			'-allow_renaming', 'true',
			'-x', '0.0',
			'-y', '0.0',
			'-z', '0.1'
		]
	)
	camera_static_transform_publisher = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_transform_publisher',
		output='screen',
		arguments=[
			'0', '0', '0', '0', '0', '0',  # Translation and rotation
			'drone', 'x500_realsense/realsense_d435/base_link/realsense_d435'
		]
	)

	# Bridge ROS topics and Gazebo messages for establishing communication
	ros_gz_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		parameters=[{
			'config_file': os.path.join(pkg_traj, 'resource', 'traj_bridge.yaml'),
			# 'qos_overrides./tf_static.publisher.durability': 'transient_local',
		}],
		prefix='gnome-terminal --tab --',
		output='screen'
	)
	
	px4_env_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[os.path.join(pkg_traj, 'px4_env.launch.py')]
		),
		launch_arguments={
			'gz_model': gz_model_value, 
			'gz_world': world_name
		}.items()
	)
	visualizer_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[os.path.join(pkg_traj, 'visualization.launch.py')]
		)
	)

	return LaunchDescription(
	[
	px4_env_launch,
	camera_static_transform_publisher,
	gz_sim,
	spawn_entity,
	ros_gz_bridge, 
	visualizer_launch
	]
	)    

