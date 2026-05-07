#!/usr/bin/env python3

import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose


# Bounds for random goal generation (matches the ROS1 implementation).
X_MIN, X_MAX, Y_MIN, Y_MAX = 25.5, 25.6, 25.5, 25.6


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__("random_goal_publisher")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.path_pub = self.create_publisher(Path, "/input/reference_path", 1)
        self.create_subscription(Empty, "/lmpcc/reset_environment", self.reset_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        self._planner_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._latest_pose = None
        self._pending_goal = None

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def reset_callback(self, _msg):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self._pending_goal = goal
        self._request_path(goal)

    def _request_path(self, goal: PoseStamped):
        if self._latest_pose is None:
            self.get_logger().warn(
                "No odometry yet; cannot request reference path. Will retry on next reset."
            )
            return

        if not self._planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("planner_server action not available; reference path not requested.")
            return

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose = self._latest_pose.pose

        request = ComputePathToPose.Goal()
        request.goal = goal
        request.start = start
        request.use_start = True
        request.planner_id = "GridBased"

        future = self._planner_client.send_goal_async(request)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("ComputePathToPose goal rejected.")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_path_result)

    def _on_path_result(self, future):
        result_wrapper = future.result()
        if result_wrapper is None:
            self.get_logger().warn("ComputePathToPose returned no result.")
            return
        path = result_wrapper.result.path
        if not path.poses:
            self.get_logger().warn("ComputePathToPose returned an empty path.")
            return
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.get_logger().info(
            f"Published reference path with {len(path.poses)} waypoints."
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
