#!/usr/bin/env python3
"""Forward /goal_pose -> NavigateToPose action.

RViz's "Nav2 Goal" tool (nav2_rviz_plugins/GoalTool) and the default
"2D Goal Pose" tool both publish geometry_msgs/PoseStamped on /goal_pose
but do not call the Nav2 action server themselves. nav2_bringup's RViz
preset relies on the "Navigation 2" panel to do that conversion. This
small bridge replicates the conversion without the panel so that
clicking a goal in RViz triggers NavigateToPose.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2GoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav2_goal_bridge")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._on_goal, 10
        )
        self.get_logger().info(
            "nav2_goal_bridge ready: /goal_pose -> /navigate_to_pose"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "navigate_to_pose action server unavailable; goal dropped"
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            "Forwarding goal frame=%s position=(%.2f, %.2f)"
            % (msg.header.frame_id, msg.pose.position.x, msg.pose.position.y)
        )
        self._client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = Nav2GoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
