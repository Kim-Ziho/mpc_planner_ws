#!/usr/bin/env python3
"""Drives the Nav2 NavfnPlanner in jackal_world_test by sending a fixed goal.

The full MPC bringup uses goal_publisher.py to generate a random goal and a
straight-line reference path. For the standalone NavfnPlanner test we want the
real grid-based plan visualized in RViz, so this node:
  - waits for /odometry/filtered to confirm the robot is up
  - calls the planner_server's compute_path_to_pose action with goal
    (NAV_GOAL_X, NAV_GOAL_Y) -- matches goal_publisher.py X_MIN/Y_MIN bounds
  - republishes the resulting Path on /input/reference_path so any downstream
    tooling that consumed the straight-line path keeps working
  - re-fires the action periodically so the plan tracks the moving robot

The planner_server already publishes the latest plan on its own /<ns>/plan
topic, which RViz visualizes directly.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose


# Matches goal_publisher.py X_MIN/Y_MIN — the canonical jackal_world goal.
NAV_GOAL_X = 25.5
NAV_GOAL_Y = 25.5

REPLAN_PERIOD_SEC = 2.0


class NavfnGoalCaller(Node):
    def __init__(self):
        super().__init__("navfn_goal_caller")

        self._action_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )

        self.path_pub = self.create_publisher(Path, "/input/reference_path", 1)

        self._have_odom = False
        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_callback, 10
        )

        self._replan_timer = self.create_timer(REPLAN_PERIOD_SEC, self._send_goal)

        self.get_logger().info(
            f"NavfnGoalCaller ready. Target = ({NAV_GOAL_X:.2f}, {NAV_GOAL_Y:.2f}). "
            f"Waiting for /odometry/filtered + planner_server."
        )

    def _odom_callback(self, _msg: Odometry):
        if not self._have_odom:
            self._have_odom = True
            self.get_logger().info("Got first odom; will start sending goals.")

    def _send_goal(self):
        if not self._have_odom:
            return
        if not self._action_client.server_is_ready():
            self._action_client.wait_for_server(timeout_sec=0.1)
            if not self._action_client.server_is_ready():
                self.get_logger().warn(
                    "compute_path_to_pose action server not ready yet."
                )
                return

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = NAV_GOAL_X
        goal.goal.pose.position.y = NAV_GOAL_Y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False
        goal.planner_id = "GridBased"

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("compute_path_to_pose goal rejected.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        path: Path = result.path
        if not path.poses:
            self.get_logger().warn("Planner returned an empty path.")
            return
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.get_logger().info(
            f"Plan ok: {len(path.poses)} waypoints to "
            f"({NAV_GOAL_X:.2f}, {NAV_GOAL_Y:.2f})."
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavfnGoalCaller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
