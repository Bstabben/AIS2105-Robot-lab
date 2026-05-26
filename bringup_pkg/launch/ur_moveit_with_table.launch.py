from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_control_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'),
            'launch',
            'ur_control.launch.py'
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
            description='UR robot type'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.1',
            description='IP address of the UR robot. Ignored when use_mock_hardware is true.'
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for local simulation (no real robot needed).'
        ),

        # Robot control stack: robot state publisher + ros2_control + controllers
        IncludeLaunchDescription(
            ur_control_launch,
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                'launch_rviz': 'false',  # MoveIt launches its own RViz
            }.items(),
        ),

        # MoveIt2 + RViz
        IncludeLaunchDescription(
            ur_moveit_launch,
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
            }.items(),
        ),
    ])
