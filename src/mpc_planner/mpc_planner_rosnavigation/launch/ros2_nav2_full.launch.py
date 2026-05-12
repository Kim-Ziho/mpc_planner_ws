"""Phase 2 / Option A bring-up: full Nav2 stack with MPCController plugin.

Stack:
  - jackal_world.launch.py (Gazebo + Jackal + UST10 laser)
  - map -> odom static TF (identity; no SLAM/AMCL)
  - pedestrian_simulator
  - planner_server (NavfnPlanner)         -- global plan
  - controller_server (MPCController)     -- local control
  - behavior_server (spin/backup/wait)    -- recovery
  - bt_navigator                          -- behaviour tree
  - lifecycle_manager_navigation          -- activates the whole stack
  - scenario_orchestrator                 -- pedsim handshake + auto random
                                            goal generator (auto-loop:
                                            reach -> reset gazebo -> new goal)
                                            + RViz "2D Goal Pose" override
  - rviz2

The standalone JackalPlanner Node bring-up (ros2_rosnavigation.launch.py)
stays in tree as a fallback during migration. See
docs/nav2_full_plugin_migration_plan.md.

Usage:
    ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py
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
    gazebo_gui = LaunchConfiguration("gui")

    declared_args = [
        DeclareLaunchArgument(
            "world_path",
            default_value=f"{WORKSPACE_PKG_DIR}/worlds/test.world",
            description="Gazebo world file forwarded to jackal_world.launch.py",
        ),
        DeclareLaunchArgument(
            "pedestrian_scenario",
            default_value="open_space/24.xml",
            description="Pedestrian scenario XML under pedestrian_simulator/scenarios",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Launch the Gazebo client (gzclient). RViz is the primary visualization, so gzclient is disabled by default. Pass gui:=true to re-enable.",
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
        launch_arguments={
            "world_path": world_path,
            "gui": gazebo_gui,
        }.items(),
    )

    # Identity static TF so Nav2 nodes that run in the map frame can lock onto
    # base_link via odom (EKF only publishes odom -> base_link).
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

    nav2_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "nav2_full.yaml"]
    )
    # MPCController plugin runs inside controller_server's process, so the
    # guidance_planner.* parameters that the standalone JackalPlanner loaded
    # must be declared on controller_server too. The /**: wildcard at the top
    # of this yaml means controller_server picks them up automatically.
    guidance_params = PathJoinSubstitution(
        [pkg_rosnav, "config", "ros2_guidance_planner.yaml"]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, guidance_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            # Plugin subscribes to /input/obstacles (legacy MPC topic). The
            # pedestrian_simulator publishes on its own namespace; bridge them
            # the same way the standalone JackalPlanner did via launch remap.
            ("/input/obstacles", "/pedestrian_simulator/trajectory_predictions"),
        ],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Inject the custom 20Hz-replanning BT so bt_navigator stops gating
    # ComputePathToPose at 1Hz. yaml doesn't expand $(find-pkg-share ...),
    # so the absolute path is computed here.
    bt_xml_20hz = PathJoinSubstitution(
        [pkg_rosnav, "behavior_trees", "replanning_20hz.xml"]
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav2_params,
            {"default_nav_to_pose_bt_xml": bt_xml_20hz},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    scenario_orchestrator = Node(
        package="mpc_planner_rosnavigation",
        executable="scenario_orchestrator",
        name="scenario_orchestrator",
        output="screen",
        # Must follow Gazebo's clock so /set_pose calls (EKF reset) carry a
        # stamp the EKF accepts; otherwise the EKF's time frame jumps to
        # wall clock and Nav2 stops seeing robot motion despite the robot
        # physically driving in Gazebo.
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_rosnav, "rviz", "ros2_nav2_full.rviz"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + laser_env
        + [
            jackal_world,
            map_to_odom,
            pedsim,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
            scenario_orchestrator,
            rviz,
        ]
    )
