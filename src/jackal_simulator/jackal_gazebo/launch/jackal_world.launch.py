from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_world = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'),
        'worlds',
        'jackal_race.world'],
    )

    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=default_world,
        description='Gazebo world file forwarded to gazebo.launch.py',
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch gzclient (Gazebo GUI). Forwarded to gazebo.launch.py.',
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'),
        'launch',
        'gazebo.launch.py'],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={
            'world_path': LaunchConfiguration('world_path'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_path)
    ld.add_action(declare_gui)
    ld.add_action(gazebo_sim)

    return ld
