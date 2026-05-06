from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mobile_robot_state_publisher")
    config = PathJoinSubstitution([pkg, "config", "jackal.yaml"])

    return LaunchDescription([
        Node(
            package="mobile_robot_state_publisher",
            executable="mobile_robot_state_publisher_node",
            name="mobile_robot_state_publisher",
            output="screen",
            parameters=[config],
        ),
    ])
