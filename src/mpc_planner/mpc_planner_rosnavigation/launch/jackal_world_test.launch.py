"""Standalone Jackal world bringup with RViz + local/global costmap + pedsim.

Used for component-level testing of jackal motion and pedestrian_simulator
visualization without the full MPC stack. Brings up:
  - jackal_world.launch.py (Gazebo Classic + Jackal + diff_drive + UST10 laser)
  - map -> odom static TF (identity, since no SLAM/AMCL in this scenario)
  - nav2_costmap_2d "local_costmap" + "global_costmap" lifecycle nodes
  - nav2 lifecycle_manager that auto-activates both costmaps
  - pedestrian_simulator (marker-only; not spawned in Gazebo, so it does not
    show up in /front/scan or the costmaps -- this is by design)
  - pedsim_starter.py: replicates JackalPlanner::startEnvironment() so pedsim
    actually ticks (publishes horizon/dt/hz then calls /pedestrian_simulator/start)
  - rviz2 with jackal_world_test.rviz preset (includes pedsim MarkerArrays)
"""

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

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # The stock nav2_costmap_2d executable hardcodes the node name to "costmap"
    # and ignores --remap __node:=..., which makes running two instances under
    # an external lifecycle_manager unworkable. costmap_pair_node uses the same
    # NodeOptions trick as JackalPlanner::initializeCostmap to bring up two
    # Costmap2DROS instances with proper node names ("local_costmap",
    # "global_costmap") and auto-activates them.
    costmap_pair = Node(
        package="mpc_planner_rosnavigation",
        executable="costmap_pair_node",
        name="costmap_pair_node",
        output="screen",
    )

    pedsim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_pedsim, "launch", "ros2_simulation.launch"])
        ),
        launch_arguments={"pedestrian_scenario": pedestrian_scenario}.items(),
    )

    # Without the MPC planner there is nothing publishing horizon / dt / hz to
    # pedsim or calling /pedestrian_simulator/start, so pedsim would sit idle
    # at spawn. pedsim_starter mirrors JackalPlanner::startEnvironment().
    pedsim_starter = Node(
        package="mpc_planner_rosnavigation",
        executable="pedsim_starter.py",
        name="pedsim_starter",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_rosnav, "rviz", "jackal_world_test.rviz"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + laser_env
        + [
            jackal_world,
            map_to_odom,
            costmap_pair,
            pedsim,
            pedsim_starter,
            rviz,
        ]
    )
