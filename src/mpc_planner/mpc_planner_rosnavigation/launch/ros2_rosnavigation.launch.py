"""Top-level launch for the ROS Navigation scenario on ROS 2 Humble.

Brings up:
  - jackal_world.launch.py (Gazebo Classic + Jackal + diff_drive + UST10 laser)
  - map -> odom static TF (identity, since no SLAM/AMCL in this scenario)
  - pedestrian_simulator
  - mpc_planner_rosnavigation::jackal_planner (standalone rclcpp node) -- it
    brings up its own local_costmap internally and handshakes pedsim, so the
    pedsim_starter helper used by jackal_world_test.launch.py is not needed.
    On goalCallback it requests a global plan from planner_server via the
    ComputePathToPose action and uses the result as the MPC reference path
    (Phase 1 / Option C in docs/nav2_planner_integration_plan.md).
  - planner_server (Nav2 NavfnPlanner) + lifecycle_manager_navfn so the global
    plan is grid-based; the embedded global_costmap covers the test scenario
    (60x60, origin (-5,-5)).
  - goal_publisher.py (random goal generator only -- the reference path is
    produced by NavfnPlanner)
  - rviz2

Use:
    ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py

Optional args:
    pedestrian_scenario:=open_space/24.xml
    world_path:=<path/to/world>
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Package share dirs do not contain `worlds/` for some sibling packages, and
# the working jackal_world_test bringup pulls the world directly from the
# source tree; mirror that pattern so the world file resolves the same way
# in both setups.
WORKSPACE_PKG_DIR = "/workspace/src/mpc_planner/mpc_planner_rosnavigation"


def generate_launch_description():
    pkg_rosnav = FindPackageShare("mpc_planner_rosnavigation")
    pkg_pedsim = FindPackageShare("pedestrian_simulator")
    pkg_jackal_gazebo = FindPackageShare("jackal_gazebo")

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
            default_value=f"{WORKSPACE_PKG_DIR}/worlds/test.world",
            description="Gazebo world file passed to jackal_world.launch.py",
        ),
    ]

    # Enable the Hokuyo UST10 front laser so /front/scan feeds local_costmap.
    # Must be set before xacro renders the URDF (jackal_description/accessories.urdf.xacro
    # gates the lidar block on $(optenv JACKAL_LASER 0)).
    laser_env = [
        SetEnvironmentVariable(name="JACKAL_LASER", value="1"),
        SetEnvironmentVariable(name="JACKAL_LASER_MODEL", value="ust10"),
        SetEnvironmentVariable(name="JACKAL_LASER_TOPIC", value="front/scan"),
    ]

    jackal_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_jackal_gazebo, "launch", "jackal_world.launch.py"])
        ),
        launch_arguments={"world_path": world_path}.items(),
    )

    # JackalPlanner brings up its own local_costmap with global_frame=map, but
    # the EKF only publishes odom -> base_link (world_frame=odom). Without a
    # map -> odom link the costmap times out during activate() with
    # "Could not find a connection between 'map' and 'base_link'". A dedicated
    # static publisher resolves this before JackalPlanner starts; this
    # mirrors the working jackal_world_test bringup.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
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

    import os as _os
    jackal_planner_kwargs = dict(
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
    # Set JACKAL_PLANNER_GDB=1 to wrap the planner with gdb for crash diagnosis.
    if _os.environ.get("JACKAL_PLANNER_GDB") == "1":
        jackal_planner_kwargs["prefix"] = (
            "gdb -batch -ex 'set pagination off' -ex run "
            "-ex 'thread apply all bt full' -ex 'quit' --args"
        )
    jackal_planner = Node(**jackal_planner_kwargs)

    goal_publisher = Node(
        package="mpc_planner_rosnavigation",
        executable="goal_publisher.py",
        name="goal_publisher",
        output="screen",
    )

    # Nav2 NavfnPlanner global planner. JackalPlanner consumes the plan via
    # the ComputePathToPose action (sent from goalCallback) so the MPC
    # reference path is grid-based instead of the straight line that
    # goal_publisher.py used to publish.
    planner_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "planner_server.yaml"]
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[planner_params],
    )
    lifecycle_manager_navfn = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navfn",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["planner_server"],
        }],
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

    return LaunchDescription(declared_args + laser_env + [
        jackal_world,
        map_to_odom,
        pedsim,
        planner_server,
        lifecycle_manager_navfn,
        jackal_planner,
        goal_publisher,
        rviz,
    ])
