"""Top-level launch for the ROS Navigation scenario on ROS 2 Humble.

Brings up:
  - Jackal in Gazebo Classic (Clearpath jackal_gazebo)
  - pedestrian_simulator
  - mpc_planner_rosnavigation::jackal_planner (standalone rclcpp node)
  - goal_publisher.py (random goal generator)
  - rviz2

Use:
    ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py

Optional args:
    pedestrian_scenario:=open_space/24.xml
    world_path:=<path/to/world>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_rosnav = FindPackageShare("mpc_planner_rosnavigation")
    pkg_pedsim = FindPackageShare("pedestrian_simulator")
    pkg_jackal_gazebo = FindPackageShare("jackal_gazebo")
    pkg_mrsp = FindPackageShare("mobile_robot_state_publisher")

    pedestrian_scenario = LaunchConfiguration("pedestrian_scenario")
    world_path = LaunchConfiguration("world_path")

    declared_args = [
        DeclareLaunchArgument(
            "pedestrian_scenario",
            default_value="open_space/24.xml",
            description="Pedestrian scenario XML path under pedestrian_simulator/scenarios",
        ),
        DeclareLaunchArgument(
            "world_path",
            default_value=PathJoinSubstitution([pkg_rosnav, "worlds", "test.world"]),
            description="Gazebo world file passed to jackal_world.launch.py",
        ),
    ]

    jackal_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_jackal_gazebo, "launch", "jackal_world.launch.py"])
        ),
        launch_arguments={"world_path": world_path}.items(),
    )

    pedsim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_pedsim, "launch", "ros2_simulation.launch"])
        ),
        launch_arguments={"pedestrian_scenario": pedestrian_scenario}.items(),
    )

    guidance_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "ros2_guidance_planner.yaml"]
    )

    jackal_planner = Node(
        package="mpc_planner_rosnavigation",
        executable="jackal_planner",
        name="jackal_planner",
        output="screen",
        parameters=[guidance_params],
        remappings=[
            ("/input/state", "/odometry/filtered"),
            ("/input/goal", "/move_base_simple/goal"),
            ("/input/obstacles", "/pedestrian_simulator/trajectory_predictions"),
            ("/output/command", "/cmd_vel"),
        ],
    )

    goal_publisher = Node(
        package="mpc_planner_rosnavigation",
        executable="goal_publisher.py",
        name="goal_publisher",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_rosnav, "rviz", "ros2_3d.rviz"]),
        ],
        output="screen",
    )

    mobile_robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mrsp, "launch", "mobile_robot_publisher.launch.py"])
        ),
    )

    return LaunchDescription(declared_args + [
        jackal_world,
        pedsim,
        mobile_robot_state,
        jackal_planner,
        goal_publisher,
        rviz,
    ])
