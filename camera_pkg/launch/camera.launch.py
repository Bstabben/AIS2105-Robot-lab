from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('camera_pkg'), 'config', 'camera_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='4',
            description='USB camera device index (e.g. 4 for /dev/video4)',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description=(
                'URL to camera calibration file, e.g. '
                'file:///home/user/calibration.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'table_z',
            default_value='0.0',
            description='Table surface Z in base_link frame (metres)',
        ),

        Node(
            package='camera_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='camera_pkg',
            executable='transform_node',
            name='transform_node',
            output='screen',
            parameters=[
                config_file,
                {'table_z': LaunchConfiguration('table_z')},
            ],
        ),
    ])
