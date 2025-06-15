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
	sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_realsense', 'model.sdf')
	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()

	ros_gz_example_gazebo_dir = get_package_share_directory('ros_gz_example_gazebo')

	set_px4_sim_model = SetEnvironmentVariable(
		name='PX4_SIM_MODEL',
		value=''
	)
	set_px4_model_name = SetEnvironmentVariable(
		name='PX4_GZ_MODEL_NAME',
		value='x500_realsense'
	)
	
	set_px4_gz_world = SetEnvironmentVariable(
		name='PX4_GZ_WORLD',
		value='x500_realsense'
	)

	px4_src_dir =  os.path.expanduser('~/PX4-Autopilot')


	px4_sim_cmd = ExecuteProcess(
		# cmd=[
		# 	'gnome-terminal',
		# 	'--',
		# 	# "PX4_GZ_STANDALONE=1",
		# 	"make", 
		# 	"px4_sitl",
		# 	airframe_value
		# ],
		cmd=[
			
			"build/px4_sitl_default/bin/px4",
		],
		cwd = px4_src_dir,
		output="screen"
	)



	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': PathJoinSubstitution([
				ros_gz_example_gazebo_dir,
				'worlds',
				'default_custom.sdf', 
			]),
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
			'-name', 'x500_realsense',
			'-allow_renaming', 'true',
			'-x', '0.0',
			'-y', '0.0',
			'-z', '0.5'
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
	


	return LaunchDescription(
	[
	
	gz_sim,
	spawn_entity,
	set_px4_sim_model, 
	set_px4_model_name,
	set_px4_gz_world,
	px4_sim_cmd,
	# ros_gz_bridge, 
	]
	)    