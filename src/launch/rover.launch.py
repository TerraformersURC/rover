from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch nodes from the drive_train package
    drive_train_nodes = [
        Node(
            package='drivetrain',
            executable='driver_node',
            output='screen'
        ),
    ]

    return LaunchDescription(drive_train_nodes)
