from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch nodes from the ds4_driver package
    ds4_driver_package_path = get_package_share_directory('ds4_driver')
    config_file_path = os.path.join(ds4_driver_package_path, 'config', 'twist_2dof.yaml')

    ds4_driver_nodes = [
        Node(
            package='ds4_driver',
            executable='ds4_driver_node.py',
            output='screen'
        ),
        Node(
            package='ds4_driver',
            executable='ds4_twist_node.py',
            arguments=['--ros-args', f'--params-file', config_file_path],
            output='screen'
        ),
    ]


    # Launch nodes from the drive_train package
    drive_train_nodes = [
        Node(
            package='drive_train',
            executable='controller_node',
            output='screen'
        ),
        Node(
            package='drive_train',
            executable='driver_node',
            output='screen'
        ),
    ]

    return LaunchDescription(ds4_driver_nodes + drive_train_nodes)
