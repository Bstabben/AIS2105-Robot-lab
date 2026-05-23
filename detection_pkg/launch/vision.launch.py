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
            'device_id',
            default_value='/dev/v4l/by-id/auto',
            description='Camera device path or index',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='URL to camera calibration file',
        ),
        DeclareLaunchArgument(
            'table_z',
            default_value='0.0',
            description='Table surface Z in base_link frame (metres)',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated debug image on vision/debug_image',
        ),

        # Camera hardware interface + coordinate transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'device_id':        LaunchConfiguration('device_id'),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'table_z':          LaunchConfiguration('table_z'),
            }.items(),
        ),

        # Cube detection
        Node(
            package='detection_pkg',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[
                detection_config,
                {'publish_debug_image': LaunchConfiguration('publish_debug_image')},
            ],
        ),
    ])
