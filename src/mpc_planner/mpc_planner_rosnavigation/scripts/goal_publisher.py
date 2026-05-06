#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped


# Bounds for random goal generation (matches the ROS1 implementation).
X_MIN, X_MAX, Y_MIN, Y_MAX = 25.5, 25.6, 25.5, 25.6


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__("random_goal_publisher")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.create_subscription(Empty, "/lmpcc/reset_environment", self.reset_callback, 10)

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
