from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_launch = PathJoinSubstitution([
        FindPackageShare('detection_pkg'), 'launch', 'vision.launch.py'
    ])
    robot_launch = PathJoinSubstitution([
        FindPackageShare('robot_pkg'), 'launch', 'robot.launch.py'
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
            description='URL to camera calibration file, e.g. file:///home/user/calibration.yaml',
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
        DeclareLaunchArgument(
            'approach_height',
            default_value='0.10',
            description='Distance above cube surface to stop at (metres)',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'device_id':           LaunchConfiguration('device_id'),
                'calibration_file':    LaunchConfiguration('calibration_file'),
                'table_z':             LaunchConfiguration('table_z'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'approach_height': LaunchConfiguration('approach_height'),
            }.items(),
        ),
    ])
