from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detection_config = PathJoinSubstitution([
        FindPackageShare('detection_pkg'), 'config', 'detection_params.yaml'
    ])

    camera_launch = PathJoinSubstitution([
        FindPackageShare('camera_pkg'), 'launch', 'camera.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'table_z',
            default_value='0.047',
            description='Table surface Z in base_link frame (metres)',
        ),
        DeclareLaunchArgument(
            'cube_height',
            default_value='0.10',
            description='Height of cubes in metres',
        ),

        # Camera hardware interface + coordinate transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'table_z': LaunchConfiguration('table_z'),
                'cube_height': LaunchConfiguration('cube_height'),
            }.items(),
        ),

        # Cube detection
        Node(
            package='detection_pkg',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[detection_config],
        ),
    ])
