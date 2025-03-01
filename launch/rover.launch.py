from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	node_lists = []
	# heartbeat
	node_lists.append([
		Node(
			package='comms_heartbeat',
			executable='heartbeat',
			remappings=[
				('/heartbeat/write', '/heartbeat_rover'),
				('/heartbeat/read', '/heartbeat_station'),
				('/heartbeat/status', '/connection_status/rover')
			],
			output='screen'
		)
	])
	
	# rover_control
	node_lists.append([
		Node(
			package='rover_control',
			executable='esc_driver',
			output='screen'
		)
	])
	
	# rover_sensors
	# node_lists.append([
	# 	Node(
	# 		package='rover_sensors',
	# 		executable='realsense_camera',
	# 		output='screen'
	# 	),
	# ])
	
	# Launch nodes from the drive_train package

	return LaunchDescription([
		node 
		for node_list in node_lists 
		for node in node_list
	])