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
            'table_z',
            default_value='0.0',
            description='Table surface Z in base_link frame (metres)',
        ),
        DeclareLaunchArgument(
            'cube_height',
            default_value='0.10',
            description='Height of cubes in metres — ray intersects at table_z + cube_height',
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
                {'table_z': LaunchConfiguration('table_z'),
                 'cube_height': LaunchConfiguration('cube_height')},
            ],
        ),
    ])
