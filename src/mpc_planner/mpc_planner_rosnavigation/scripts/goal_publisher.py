#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


# Bounds for random goal generation (matches the ROS1 implementation).
X_MIN, X_MAX, Y_MIN, Y_MAX = 25.5, 25.6, 25.5, 25.6

# Dense straight-line reference path so the MPC's contouring spline stays smooth.
# Nav2 NavfnPlanner produces a grid-quantized path that triggers QP failures in
# the MPC; the test scenario is an open world, so a straight reference is fine.
PATH_RESOLUTION_M = 0.1


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__("random_goal_publisher")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.path_pub = self.create_publisher(Path, "/input/reference_path", 1)
        self.create_subscription(Empty, "/lmpcc/reset_environment", self.reset_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)

        self._latest_pose = None
        # Without an initial fire the planner sits on "missing Reference Path"
        # until JackalPlanner's 60s timeout publishes /lmpcc/reset_environment.
        # Trigger a goal as soon as the first odom arrives so the planner has
        # a path from t=0.
        self._initial_goal_sent = False

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose
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
        self._publish_straight_line_path(goal)

    def _publish_straight_line_path(self, goal: PoseStamped):
        if self._latest_pose is None:
            self.get_logger().warn(
                "No odometry yet; cannot publish reference path. Will retry on next reset."
            )
            return

        sx = self._latest_pose.pose.position.x
        sy = self._latest_pose.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        dx = gx - sx
        dy = gy - sy
        dist = math.hypot(dx, dy)
        n_points = max(2, int(dist / PATH_RESOLUTION_M) + 1)

        yaw = math.atan2(dy, dx)
        qz = math.sin(0.5 * yaw)
        qw = math.cos(0.5 * yaw)

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(n_points):
            t = i / float(n_points - 1)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = sx + t * dx
            ps.pose.position.y = sy + t * dy
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)

        self.path_pub.publish(path)
        self.get_logger().info(
            f"Published straight-line reference path with {n_points} waypoints "
            f"({dist:.2f} m)."
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
