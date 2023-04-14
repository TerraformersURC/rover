from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_train',
            namespace='drive_train1',
            executable='controller_node',
            name='controller'
        ),
        Node(
            package='drive_train',
            namespace='drive_train1',
            executable='driver_node',
            name='driver'
        ),
    ])