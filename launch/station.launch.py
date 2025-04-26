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
				('/heartbeat/write', '/heartbeat_station'),
				('/heartbeat/read', '/heartbeat_rover'),
				('/heartbeat/status', '/connection_status/station')
			],
			output='screen'
		)
	])
	
	# joystick control
	node_lists.append([
		Node(
			package='joy',
			executable='joy_node',
			output='screen'
		),
		Node(
			package='station_ui',
			executable='joystick_command',
			output='screen',
			parameters=[{'max_speed': 0.75, 'turn_scaling': 0.5}]
		)
	])
 
	node_lists.append([
		Node(
			package='station_ui',
			executable='camera_display',
			output='screen'
		)
	])
	
	# Launch nodes from the drive_train package

	return LaunchDescription([
		node 
		for node_list in node_lists 
		for node in node_list
	])