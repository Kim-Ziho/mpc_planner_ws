#!/usr/bin/env python3

"""Standalone helper that drives pedestrian_simulator to start without MPC.

ROSPedestrianSimulator only begins ticking once it has received horizon
(/pedestrian_simulator/horizon), integrator step (/pedestrian_simulator/integrator_step)
and clock frequency (/pedestrian_simulator/clock_frequency), and then receives a
call on the /pedestrian_simulator/start service. JackalPlanner does this in
ros2_rosnavigation.cpp::startEnvironment(); for the standalone jackal_world
test bringup we replicate the same handshake here so that markers and
trajectory_predictions are published.
"""

import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty


HORIZON_DEFAULT = 50
INTEGRATOR_STEP_DEFAULT = 0.2
CLOCK_FREQUENCY_DEFAULT = 20.0
SETTINGS_REPEAT = 20
SETTINGS_PERIOD_S = 0.1
SERVICE_WAIT_S = 0.2
SERVICE_RETRY_LIMIT = 50


class PedsimStarter(Node):
    def __init__(self):
        super().__init__("pedsim_starter")

        self.declare_parameter("horizon", HORIZON_DEFAULT)
        self.declare_parameter("integrator_step", INTEGRATOR_STEP_DEFAULT)
        self.declare_parameter("clock_frequency", CLOCK_FREQUENCY_DEFAULT)

        self._horizon = int(self.get_parameter("horizon").value)
        self._integrator_step = float(self.get_parameter("integrator_step").value)
        self._clock_frequency = float(self.get_parameter("clock_frequency").value)

        self._horizon_pub = self.create_publisher(Int32, "/pedestrian_simulator/horizon", 1)
        self._dt_pub = self.create_publisher(Float32, "/pedestrian_simulator/integrator_step", 1)
        self._hz_pub = self.create_publisher(Float32, "/pedestrian_simulator/clock_frequency", 1)
        self._start_client = self.create_client(Empty, "/pedestrian_simulator/start")

        self._publish_count = 0
        self._service_attempts = 0
        self._timer = self.create_timer(SETTINGS_PERIOD_S, self._tick)

        self.get_logger().info(
            f"pedsim_starter: horizon={self._horizon}, dt={self._integrator_step}, "
            f"hz={self._clock_frequency}"
        )

    def _publish_settings(self):
        n = Int32()
        n.data = self._horizon
        self._horizon_pub.publish(n)

        dt = Float32()
        dt.data = self._integrator_step
        self._dt_pub.publish(dt)

        hz = Float32()
        hz.data = self._clock_frequency
        self._hz_pub.publish(hz)

    def _tick(self):
        if self._publish_count < SETTINGS_REPEAT:
            self._publish_settings()
            self._publish_count += 1
            return

        if not self._start_client.service_is_ready():
            if not self._start_client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                self._service_attempts += 1
                if self._service_attempts >= SERVICE_RETRY_LIMIT:
                    self.get_logger().error(
                        "pedsim_starter: /pedestrian_simulator/start unavailable, giving up"
                    )
                    self._timer.cancel()
                return

        # Republish settings once more right before start so the latest values win.
        self._publish_settings()
        self._start_client.call_async(Empty.Request())
        self.get_logger().info("pedsim_starter: /pedestrian_simulator/start called")
        self._timer.cancel()


def main(argv=None):
    rclpy.init(args=argv)
    node = PedsimStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
