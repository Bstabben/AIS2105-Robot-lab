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

        # ── Launch-argumenter ──────────────────────────────────────────────────

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
            description='Table surface Z in base_link frame (metres). '
                        'Measure by jogging TCP to table surface and reading Z.',
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

        # ── UR-driver (publiserer /joint_states + starter robot_state_publisher)
        # robot_state_publisher er inkludert i ur_control.launch.py og konverterer
        # joint_states til TF: base_link → shoulder_link → ... → tool0
        # ──────────────────────────────────────────────────────────────────────

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch),
            launch_arguments={
                'ur_type':           LaunchConfiguration('ur_type'),
                'robot_ip':          LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            }.items(),
        ),

        # ── Kamera-TF: tool0 → camera_link ────────────────────────────────────
        # Disse verdiene (x, y, z) må måles fysisk — de beskriver hvor kameraet
        # sitter relativt til tool0 på robotarmen.
        # ──────────────────────────────────────────────────────────────────────

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf',
            arguments=[
                '--x',     '0.05',   # fremover fra tool0 (juster)
                '--y',     '0.0',
                '--z',     '0.05',   # oppover fra tool0 (juster)
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',       'tool0',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # ── Kamera-TF: camera_link → camera_optical_link ──────────────────────
        # Fast rotasjon (-π/2, 0, -π/2) konverterer fra REP-103-konvensjon
        # (x fremover) til optisk konvensjon (z fremover, x høyre, y ned).
        # Pinhole-modellen i transform_node krever optisk konvensjon.
        # ──────────────────────────────────────────────────────────────────────

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

        # ── Visjon (kamera + deteksjon) ───────────────────────────────────────

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'device_id':           LaunchConfiguration('device_id'),
                'calibration_file':    LaunchConfiguration('calibration_file'),
                'table_z':             LaunchConfiguration('table_z'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }.items(),
        ),

        # ── Robot (bevegelse + koordinator) ───────────────────────────────────

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'approach_height': LaunchConfiguration('approach_height'),
            }.items(),
        ),
    ])