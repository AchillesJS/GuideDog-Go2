# SPDX-License-Identifier: Apache-2.0
"""Forward-only LiDAR command veto for the simulated flat-ground demo."""

from collections import deque
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from guidedog_perception.guard_geometry import (
    cloud_xyz, inspect_corridor, MotionGate, transform_xyz)
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class LidarStop(Node):
    """Reject motion on bad geometry, stale data, invalid TF or missing commands."""

    def __init__(self):
        """Create a latest-scan observer and an independent wall-clock gate."""
        super().__init__('guidedog_lidar_stop')
        defaults = {
            'report_dir': str(Path.home() / '.ros/guidedog/stop'),
            'lidar_topic': '/unitree_lidar/points', 'base_frame': 'base_footprint',
            'max_speed': 0.05, 'stop_distance': 0.80, 'scan_timeout': 1.50,
            'command_timeout': 0.40, 'front_extent': 0.45, 'half_width': 0.40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for name in ('front_extent', 'half_width'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.gate = MotionGate(**{name: self.config[name] for name in (
            'max_speed', 'stop_distance', 'scan_timeout', 'command_timeout')})
        self.directory = Path(self.config['report_dir']).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = deque(maxlen=8)
        self.last_ros_time = None
        self.clock_advanced_at = None
        self.last_report_at = 0.0
        self.last_reason = None
        self.last_status = {}
        self.started = time.monotonic()
        latest_scan = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            PointCloud2, self.config['lidar_topic'], self.on_cloud, latest_scan)
        self.request_sub = self.create_subscription(
            Twist, '/guidedog/cmd_vel_request', self.on_request, 1)
        self.publisher = self.create_publisher(Twist, '/guidedog/cmd_vel_candidate', 1)
        self.status_publisher = self.create_publisher(String, '/guidedog/stop_status', 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)
        self.get_logger().info('Flat-ground guard: forward only, 0.05 m/s cap by default; '
                               'press k then i to rearm after a stop.')

    def on_request(self, msg):
        """Keep the newest request; never forward lateral or turning commands."""
        self.gate.request([msg.linear.x, msg.linear.y, msg.linear.z,
                           msg.angular.x, msg.angular.y, msg.angular.z], time.monotonic())

    def on_cloud(self, msg):
        """Keep a bounded TF wait queue with original wall receipt times."""
        now = time.monotonic()
        if not msg.header.frame_id:
            self.gate.invalidate('empty_frame_id', now)
            self.pending.clear()
            return
        timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.pending.append((msg, now, timestamp))

    def process_pending(self, now):
        """Wait briefly for exact-time TF without blocking its subscription."""
        if not self.pending:
            return
        selected = None
        # A newer scan must not evict an older scan whose exact TF just arrived.
        for index in range(len(self.pending) - 1, -1, -1):
            msg, received, timestamp = self.pending[index]
            if now - received > 0.15:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config['base_frame'], msg.header.frame_id,
                    Time(nanoseconds=timestamp, clock_type=ClockType.ROS_TIME))
                selected = index
                break
            except TransformException:
                continue
        if selected is None:
            if now - self.pending[0][1] > 0.15:
                self.gate.invalidate('tf_unavailable', now)
                while self.pending and now - self.pending[0][1] > 0.15:
                    self.pending.popleft()
            return
        for _ in range(selected + 1):
            self.pending.popleft()
        try:
            points = cloud_xyz(msg)
            points = points[np.isfinite(points).all(axis=1)]
            tf = transform.transform
            points = transform_xyz(
                points, [tf.translation.x, tf.translation.y, tf.translation.z],
                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
            scan = inspect_corridor(points, front=self.config['front_extent'],
                                    half_width=self.config['half_width'])
            self.gate.observe(scan, received, timestamp)
        except (ValueError, TypeError, BufferError) as error:
            self.gate.invalidate(f'invalid_scan:{error}', now)

    def tick(self):
        """Publish a bounded velocity or zero, using a steady 20 Hz timer."""
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        if self.last_ros_time is not None and ros_now < self.last_ros_time:
            self.gate.invalidate('clock_reset', now)
            self.gate.scan_stamp = None
            self.pending.clear()
        if ros_now > 0 and ros_now != self.last_ros_time:
            self.clock_advanced_at = now
        self.last_ros_time = ros_now
        clock_ok = (self.clock_advanced_at is not None
                    and now - self.clock_advanced_at <= self.gate.scan_timeout)
        self.process_pending(now)
        age = ((ros_now - self.gate.scan_stamp) / 1e9
               if self.gate.scan_stamp is not None else None)
        velocity, reason = self.gate.decide(now, clock_ok=clock_ok, stamp_age=age)
        command = Twist()
        command.linear.x = velocity
        self.publisher.publish(command)
        status = {
            'elapsed_wall_sec': now - self.started, 'ros_time_sec': ros_now / 1e9,
            'reason': reason, 'output_speed_mps': velocity, 'latched': self.gate.latched,
            'scan_age_wall_sec': now - self.gate.scan_at if self.gate.scan_at else None,
            'scan_stamp_age_sec': age, 'clock_ok': clock_ok,
            'scan': self.gate.scan, 'parameters': self.config,
        }
        changed = reason != self.last_reason
        if changed:
            self.get_logger().info(f'{reason}: output={velocity:.2f} m/s')
            with (self.directory / 'stop_events.jsonl').open('a') as output:
                output.write(json.dumps(status, allow_nan=False) + '\n')
        if changed or now - self.last_report_at >= 0.2:
            self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))
        if changed or now - self.last_report_at >= 1.0:
            temporary = self.directory / 'stop_report.json.tmp'
            temporary.write_text(json.dumps(status, indent=2, allow_nan=False) + '\n')
            temporary.replace(self.directory / 'stop_report.json')
            self.last_report_at = now
        self.last_status = status
        self.last_reason = reason


def main(args=None):
    """Run the veto; an independent process watches its output heartbeat."""
    rclpy.init(args=args)
    node = LidarStop()
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
