"""Nav2 A* global planner + MPC (JackalPlanner) local controller demo.

Stack:
  - jackal_world.launch.py (Gazebo Classic + Jackal + diff_drive + UST10 laser)
  - map -> odom static TF (identity; no SLAM/AMCL needed for the demo)
  - Nav2 planner_server (NavfnPlanner, A* enabled)  -> global plan, queried
                          by jackal_planner via the ComputePathToPose action
  - lifecycle_manager_navfn (auto-activates planner_server)
  - pedestrian_simulator   (marker-only; not spawned in Gazebo, so does not
                            show up in /front/scan or the costmaps -- by design)
  - jackal_planner          (mpc_planner_rosnavigation; standalone rclcpp::Node
                            that brings up its own local_costmap, runs the MPC
                            control loop, publishes /cmd_vel directly, and
                            handshakes pedsim in startEnvironment())
  - rviz2

The Nav2 controller / behavior / bt_navigator / goal_bridge stack is
intentionally absent: JackalPlanner owns its own control loop and consumes
/goal_pose directly via remap, so the BT and FollowPath layer would be
redundant. Recovery (spin / backup / wait) is therefore not available; this
matches the existing ros2_rosnavigation.launch.py topology, just with A*
enabled on planner_server.

Usage:
    ros2 launch mpc_planner_rosnavigation nav2_demo.launch.py
Send a goal from RViz's "2D Goal Pose" / "Nav2 Goal" tool. The MPC follows
the A* reference path via /cmd_vel.

Optional args:
    pedestrian_scenario:=open_space/24.xml
    world_path:=<path/to/world>
"""

import os as _os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORKSPACE_PKG_DIR = "/workspace/src/mpc_planner/mpc_planner_rosnavigation"


def generate_launch_description():
    pkg_rosnav = FindPackageShare("mpc_planner_rosnavigation")
    pkg_jackal_gazebo = FindPackageShare("jackal_gazebo")
    pkg_pedsim = FindPackageShare("pedestrian_simulator")

    world_path = LaunchConfiguration("world_path")
    pedestrian_scenario = LaunchConfiguration("pedestrian_scenario")

    declared_args = [
        DeclareLaunchArgument(
            "world_path",
            default_value=f"{WORKSPACE_PKG_DIR}/worlds/test.world",
            description="Gazebo world file forwarded to jackal_world.launch.py",
        ),
        DeclareLaunchArgument(
            "pedestrian_scenario",
            default_value="open_space/24.xml",
            description="Pedestrian scenario XML path under pedestrian_simulator/scenarios",
        ),
    ]

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

    # planner_server's global_costmap runs in the "map" frame, jackal_planner's
    # local_costmap and EKF run in "odom". Without an identity static TF the
    # global_costmap times out waiting for map -> base_link.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    nav2_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "nav2_demo_params.yaml"]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
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

    pedsim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_pedsim, "launch", "ros2_simulation.launch"])
        ),
        launch_arguments={"pedestrian_scenario": pedestrian_scenario}.items(),
    )

    guidance_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "ros2_guidance_planner.yaml"]
    )

    jackal_planner_kwargs = dict(
        package="mpc_planner_rosnavigation",
        executable="jackal_planner",
        name="jackal_planner",
        output="screen",
        parameters=[guidance_params],
        remappings=[
            ("/input/state", "/odometry/filtered"),
            # RViz "2D Goal Pose" / "Nav2 Goal" tools both publish PoseStamped
            # on /goal_pose; route that straight into JackalPlanner's
            # goalCallback (which then requests a global plan from
            # planner_server via ComputePathToPose).
            ("/input/goal", "/goal_pose"),
            ("/input/obstacles", "/pedestrian_simulator/trajectory_predictions"),
            ("/output/command", "/cmd_vel"),
        ],
    )
    # JACKAL_PLANNER_GDB=1 wraps the planner with gdb for crash diagnosis.
    if _os.environ.get("JACKAL_PLANNER_GDB") == "1":
        jackal_planner_kwargs["prefix"] = (
            "gdb -batch -ex 'set pagination off' -ex run "
            "-ex 'thread apply all bt full' -ex 'quit' --args"
        )
    jackal_planner = Node(**jackal_planner_kwargs)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_rosnav, "rviz", "nav2_demo.rviz"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + laser_env
        + [
            jackal_world,
            map_to_odom,
            planner_server,
            lifecycle_manager_navfn,
            pedsim,
            jackal_planner,
            rviz,
        ]
    )
