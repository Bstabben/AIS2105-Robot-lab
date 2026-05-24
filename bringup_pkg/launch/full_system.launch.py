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
    ur_launch = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'
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
            default_value='0.10',
            description='Distance above cube surface to stop at (metres)',
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='UR robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e)',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.102',
            description='IP address of the UR controller',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Set true to run without a physical robot (simulation only)',
        ),

        #UR driverpublishes /joint_states and starts robot_state_publisher.
        # robot_state_publisher converts joint states
        # to the TF chain: base_link to  (...) to tool0

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch),
            launch_arguments={
                'ur_type':           LaunchConfiguration('ur_type'),
                'robot_ip':          LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            }.items(),
        ),

        # Camera TF: tool0 → camera_link
        # x, y, z describe where the camera sits relative to tool0 on the arm.


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf',
            arguments=[
                '--x',     '0.05',
                '--y',     '0.0',
                '--z',     '0.05',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',       'tool0',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # Camera TF: camera_link → camera_optical_link

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll',  '-1.5708',
                '--pitch', '0.0',
                '--yaw',   '-1.5708',
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