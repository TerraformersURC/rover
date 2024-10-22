from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='moon',
            choices=['moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[],
            remappings=[],
            output='screen'
        ),
    
        Node(
            name="controller_spawner",
            package='controller_manager',
            executable='controller_spawner',
            type="spawner",
            arguments=["right_wheel_link_to_base_link_joint_position_controller",
                       "left_wheel_link_to_base_link_joint_position_controller",
                       "rear_right_wheel_to_wheel_link_joint_position_controller",
                       "front_right_wheel_to_wheel_link_joint_position_controller",
                       "front_left_wheel_to_wheel_link_joint_position_controller",
                       "rear_left_wheel_to_wheel_link_joint_position_controller",
                       "mid_balance_link_to_base_link_joint_position_controller",
                       "right_balance_link_to_mid_balance_link_joint_position_controller",
                       "left_balance_link_to_mid_balance_link_joint_position_controller ",
                       "joint_state_controller"],
            respawn="false",
            output="screen",
            namespace="rover"
        ),
        Node(
            name="robot_state_publisher",
            package='robot_state_publisher',
            type="robot_state_publisher",
            executable='robot_state_publisher',
            respawn="false",
            arguments=[],
            remappings=[("joint_states","rover/joint_states")],
            output='screen'
        ),
    ])