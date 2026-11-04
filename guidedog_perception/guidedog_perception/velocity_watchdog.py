# SPDX-License-Identifier: Apache-2.0
"""Independent wall-clock timeout between the veto and the simulated controller."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class VelocityWatchdog(Node):
    """Keep sending zero if the upstream veto stops publishing."""

    def __init__(self):
        """Use a separate process and steady timer, not the simulated clock."""
        super().__init__('guidedog_velocity_watchdog')
        self.timeout = self.declare_parameter('timeout', 0.25).value
        self.max_speed = self.declare_parameter('max_speed', 0.05).value
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.max_speed)):
            raise ValueError('timeout and max_speed must be finite and positive')
        self.received = None
        self.speed = 0.0
        self.publisher = self.create_publisher(Twist, '/guidedog/cmd_vel_checked', 1)
        self.subscription = self.create_subscription(
            Twist, '/guidedog/cmd_vel_candidate', self.on_command, 1)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.clock)

    def on_command(self, msg):
        """Reject invalid or unsupported candidate components independently."""
        values = [msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z]
        self.received = time.monotonic()
        self.speed = (min(values[0], self.max_speed)
                      if all(math.isfinite(v) for v in values) and values[0] >= 0
                      and all(abs(v) <= 1e-6 for v in values[1:]) else 0.0)

    def tick(self):
        """Publish an explicit zero message when the heartbeat is missing."""
        command = Twist()
        if self.received is not None and time.monotonic() - self.received <= self.timeout:
            command.linear.x = self.speed
        self.publisher.publish(command)


def main(args=None):
    """Run the downstream watchdog independently from point-cloud processing."""
    rclpy.init(args=args)
    node = VelocityWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
