from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_launch = PathJoinSubstitution([
        FindPackageShare('detection_pkg'), 'launch', 'vision.launch.py'
    ])
    robot_launch = PathJoinSubstitution([
        FindPackageShare('robot_pkg'), 'launch', 'robot.launch.py'
    ])

    return LaunchDescription([

        # Launch arguments

        DeclareLaunchArgument(
            'table_z',
            default_value='0.047',
            description='Table surface Z in base_link frame (metres). '
                        'Measure by jogging TCP to table surface and reading Z.',
        ),
        DeclareLaunchArgument(
            'approach_height',
            default_value='0.01',
            description='Distance above cube surface to stop at (metres)',
        ),
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.01',
            description='Camera offset from tool0 along X in metres — measure physically and tune',
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera offset from tool0 along Y in metres',
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.01',
            description='Camera offset from tool0 along Z in metres — measure physically and tune',
        ),

        # Camera TF: tool0 → camera_link
        # x, y, z describe where the camera sits relative to tool0 on the arm.


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf',
            arguments=[
                '--x',     LaunchConfiguration('camera_x'),
                '--y',     LaunchConfiguration('camera_y'),
                '--z',     LaunchConfiguration('camera_z'),
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',       'tool0',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # Camera TF: camera_link → camera_optical_link

        # camera_optical_link = camera_link orientation (identity rotation).
        # Tool0 Z points straight down toward the table, so Z_optical = Z_tool0 = down.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',       'camera_link',
                '--child-frame-id', 'camera_optical_link',
            ],
        ),

        # Vision (camera + detection)

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'table_z': LaunchConfiguration('table_z'),
            }.items(),
        ),

        # Robot (motion + coordinator)

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'approach_height': LaunchConfiguration('approach_height'),
            }.items(),
        ),
    ])