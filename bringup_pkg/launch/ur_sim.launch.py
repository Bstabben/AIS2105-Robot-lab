"""
Simulation launch: UR robot + MoveIt2 without hardware.
Uses ur_rsp for robot description and a ros2_control setup
that avoids the UR-specific GPIO controllers

Usage:
    ros2 launch bringup_pkg ur_sim.launch.py [ur_type:=ur5e]
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_rsp_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'),
            'launch',
            'ur_rsp.launch.py'
        ])
    )

    ur_moveit_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'),
            'launch',
            'ur_moveit.launch.py'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot type',
            choices=['ur3', 'ur5', 'ur10', 'ur3e', 'ur5e', 'ur10e', 'ur16e'],
        ),

        # Robot state publisher: generates /robot_description from the UR xacro
        IncludeLaunchDescription(
            ur_rsp_launch,
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': '192.168.0.1',  # dummy
                'use_mock_hardware': 'true',
            }.items(),
        ),

        # ros2_control + generic controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('bringup_pkg'),
                    'config',
                    'mock_controllers.yaml'
                ]),
            ],
            remappings=[('~/robot_description', '/robot_description')],
            output='screen',
        ),

        # Broadcast joint states so MoveIt2 can track robot pose
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        # Trajectory controller to execute plans with MoveIt2
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scaled_joint_trajectory_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        # MoveIt2 + RViz
        IncludeLaunchDescription(
            ur_moveit_launch,
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
            }.items(),
        ),
    ])
