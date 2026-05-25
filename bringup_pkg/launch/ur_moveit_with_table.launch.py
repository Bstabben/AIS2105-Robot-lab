from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_moveit_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'),
            'launch',
            'ur_moveit.launch.py'
        ])
    )

    table_urdf = PathJoinSubstitution([
        FindPackageShare('ur_project'),
        'ur_description',
        'urdf',
        'table.urdf'
    ])

    return LaunchDescription([
        # Launch standard MoveIt2 with robot
        IncludeLaunchDescription(ur_moveit_launch),

        # Publish table as static URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='table_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(table_urdf).read(),
            }],
            remappings=[
                ('robot_description', 'table_description'),
            ],
        ),
    ])
