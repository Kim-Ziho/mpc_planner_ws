#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


# Bounds for random goal generation (matches the ROS1 implementation).
X_MIN, X_MAX, Y_MIN, Y_MAX = 25.5, 25.6, 25.5, 25.6


class RandomGoalPublisher(Node):
    """Publishes a random goal on /move_base_simple/goal.

    The reference path used by the MPC is no longer produced here -- since
    Phase 1 of docs/nav2_planner_integration_plan.md, JackalPlanner requests a
    grid-based plan from Nav2 planner_server via ComputePathToPose on every
    new goal.
    """

    def __init__(self):
        super().__init__("random_goal_publisher")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.create_subscription(Empty, "/lmpcc/reset_environment", self.reset_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        # Without an initial fire the planner sits on "missing Reference Path"
        # until JackalPlanner's 60s timeout publishes /lmpcc/reset_environment.
        # Trigger a goal as soon as the first odom arrives so the planner has
        # a path from t=0.
        self._initial_goal_sent = False

    def odom_callback(self, _msg: Odometry):
        if not self._initial_goal_sent:
            self._initial_goal_sent = True
            self.reset_callback(None)

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
        self.get_logger().info(f"Published goal ({x:.2f}, {y:.2f}).")


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
