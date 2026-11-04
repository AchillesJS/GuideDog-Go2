# SPDX-License-Identifier: Apache-2.0
"""Keyboard requests with a dead-man timeout for the straight-line demo."""

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy


def main():
    """Hold i to move slowly, k/space to stop; never command the controller directly."""
    if not sys.stdin.isatty():
        raise RuntimeError('Run guarded_teleop in an interactive Ubuntu terminal')
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('guidedog_guarded_teleop')
    publisher = node.create_publisher(Twist, '/guidedog/cmd_vel_request', 1)
    last_forward = -1.0
    print('Hold i: forward (0.05 m/s). k/space: stop and rearm. Ctrl+C: exit.\n'
          'After an obstacle/fault, press k then i. Turning/reverse are blocked.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                last_forward = time.monotonic() if key == 'i' else -1.0
            command = Twist()
            if time.monotonic() - last_forward <= 0.6:
                command.linear.x = 0.05
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
