# SPDX-License-Identifier: Apache-2.0
"""Observe simulated sensors and write reproducible timing and TF reports."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import time

from guidedog_perception.sensor_metrics import assess_sensor, native_frequency, StreamMetrics
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_ros import Buffer, TransformListener


def stamp_ns(stamp):
    """Convert a ROS message timestamp to integer nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def git_revision(path):
    """Read a local revision without accessing the network."""
    try:
        return subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'],
            stderr=subprocess.DEVNULL, text=True, timeout=2).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def git_is_dirty(path):
    """Record modified and untracked source files for reproducible reports."""
    try:
        return bool(subprocess.check_output(
            ['git', '-C', str(path), 'status', '--porcelain'],
            stderr=subprocess.DEVNULL, text=True, timeout=2).strip())
    except (OSError, subprocess.SubprocessError):
        return None


class SensorMonitor(Node):
    """Use a steady timer so a paused ROS clock cannot hide stale streams."""

    def __init__(self):
        """Configure bounded observations and create the output session."""
        super().__init__('guidedog_sensor_monitor')
        defaults = {
            'report_dir': str(Path.home() / '.ros/guidedog/reports' /
                              datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')),
            'report_period_sec': 5.0, 'window_sec': 10.0,
            'stale_after_sec': 2.0, 'duration_sec': 0.0, 'base_frame': 'base_link',
            'rgb_topic': '/rgb_image', 'lidar_topic': '/unitree_lidar/points',
            'imu_topic': '/imu/data', 'odom_topic': '/odom',
            'rgb_frame': 'camera_link', 'lidar_frame': 'lidar_l1_link', 'rgb_enabled': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for name in ('report_period_sec', 'window_sec', 'stale_after_sec'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(self.config['duration_sec']) or self.config['duration_sec'] < 0:
            raise ValueError('duration_sec must be finite and nonnegative')
        if not self.config['report_dir'].strip():
            raise ValueError('report_dir must not be empty')
        self.directory = Path(self.config['report_dir']).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        if (self.directory / 'report.json').exists():
            raise ValueError('report_dir already contains a report; choose a new directory')
        self.started = time.monotonic()
        self.finished = False
        self.sim_time = bool(self.get_parameter('use_sim_time').value)
        self.clock_metrics = StreamMetrics(self.config['window_sec'])
        self.streams = {name: StreamMetrics(self.config['window_sec'])
                        for name in ('rgb', 'lidar', 'imu', 'odom')}
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscriptions_owned = [self.create_subscription(
            ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)]
        for name, message_type in (
                ('rgb', Image), ('lidar', PointCloud2), ('imu', Imu), ('odom', Odometry)):
            self.subscriptions_owned.append(self.create_subscription(
                message_type, self.config[f'{name}_topic'],
                lambda msg, sensor=name: self.on_sensor(sensor, msg), qos_profile_sensor_data))
        repo = Path(__file__).resolve().parents[2]
        self.metadata = {
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'architecture': platform.machine(), 'python': platform.python_version(),
            'project_revision': git_revision(repo),
            'project_worktree_dirty': git_is_dirty(repo),
            'simulator_revision': git_revision(repo.parent / 'unitree_go2_ros2'),
            'use_sim_time': self.sim_time, 'parameters': self.config,
            'qos': 'sensor_data: best_effort, volatile, keep_last(5)',
            'rate_note': 'Receipt rates are not publisher rates or proof of packet loss.',
        }
        self.previous_cpu = None
        self.previous_system_metrics = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            self.config['report_period_sec'], self.write_report, clock=self.steady_clock)
        self.get_logger().info(f'Reports: {self.directory}')

    def on_clock(self, msg):
        """Track clock progress independently from ROS timer scheduling."""
        self.clock_metrics.observe(time.monotonic(), stamp_ns(msg.clock))

    def on_sensor(self, name, msg):
        """Retain timing and frame metadata without retaining the payload."""
        timestamp = stamp_ns(msg.header.stamp)
        clock_ready = not self.sim_time or self.clock_metrics.count > 0
        age = ((self.get_clock().now().nanoseconds - timestamp) / 1e9
               if clock_ready else None)
        self.streams[name].observe(
            time.monotonic(), timestamp, msg.header.frame_id, age)

    def system_metrics(self):
        """Sample whole-VM CPU and available RAM from Linux procfs."""
        now = time.monotonic()
        if self.previous_cpu and now - self.previous_cpu[2] < 1:
            cached = dict(self.previous_system_metrics)
            cached['sample_age_wall_sec'] = now - self.previous_cpu[2]
            return cached
        result = {'load_average': list(os.getloadavg())}
        try:
            memory = dict(line.split(':', 1)
                          for line in Path('/proc/meminfo').read_text().splitlines())
            result['memory_total_mib'] = int(memory['MemTotal'].split()[0]) / 1024
            result['memory_available_mib'] = int(memory['MemAvailable'].split()[0]) / 1024
            cpu_values = Path('/proc/stat').read_text().splitlines()[0].split()[1:9]
            counters = [int(value) for value in cpu_values]
            current = (sum(counters), counters[3] + counters[4], now)
            result['cpu_busy_percent'] = None
            result['cpu_measurement_interval_wall_sec'] = None
            if self.previous_cpu:
                total = current[0] - self.previous_cpu[0]
                idle = current[1] - self.previous_cpu[1]
                if total > 0:
                    result['cpu_busy_percent'] = 100 * (total - idle) / total
                    result['cpu_measurement_interval_wall_sec'] = now - self.previous_cpu[2]
            self.previous_cpu = current
        except (OSError, KeyError, ValueError, IndexError):
            result['proc_metrics_unavailable'] = True
        self.previous_system_metrics = dict(result)
        return result

    def gazebo_metrics(self):
        """Read bounded log tails from the two native Gazebo subscribers."""
        result = {}
        for name in ('rgb', 'lidar'):
            path = self.directory / f'gazebo-{name}.log'
            try:
                with path.open('rb') as source:
                    source.seek(0, 2)
                    source.seek(max(0, source.tell() - 8192))
                    rate = native_frequency(source.read().decode('utf-8', errors='replace'))
                result[name] = {
                    'latest_received_hz_wall': rate,
                    'log_age_wall_sec': max(0, time.time() - path.stat().st_mtime),
                    'window_note': 'Latest native CLI window; differs from ROS window.',
                }
            except OSError:
                result[name] = {'latest_received_hz_wall': None, 'log_age_wall_sec': None}
        return result

    def write_report(self, final=False):
        """Write an atomic latest snapshot and append one time-series record."""
        now = time.monotonic()
        clock = self.clock_metrics.snapshot(now)
        clock_ok = not self.sim_time or (
            clock['messages_total'] > 0
            and clock['since_last_message_wall_sec'] <= self.config['stale_after_sec']
            and clock['since_stamp_advanced_wall_sec'] <= self.config['stale_after_sec'])
        sensors = {}
        for name, stream in self.streams.items():
            stats = stream.snapshot(now)
            age = ((self.get_clock().now().nanoseconds - stream.last_stamp_ns) / 1e9
                   if stream.count and (not self.sim_time or clock['messages_total']) else None)
            tf_ok = bool(stream.frame_id and stream.last_stamp_ns is not None and
                         self.tf_buffer.can_transform(
                             self.config['base_frame'], stream.frame_id,
                             Time(nanoseconds=stream.last_stamp_ns,
                                  clock_type=ClockType.ROS_TIME)))
            issues = assess_sensor(
                stats, stale_after=self.config['stale_after_sec'], clock_ok=clock_ok,
                age_sec=age, tf_ok=tf_ok, expected_frame=self.config.get(f'{name}_frame', ''))
            enabled = name != 'rgb' or self.config['rgb_enabled']
            if not enabled:
                issues = []
            stats.update(topic=self.config[f'{name}_topic'], latest_age_sec=age,
                         tf_to_base_at_message_time=tf_ok, issues=issues,
                         enabled=enabled,
                         status=('DISABLED' if not enabled else 'OK' if not issues else 'WARN'))
            sensors[name] = stats
        report = {
            'schema_version': 1, 'metadata': self.metadata,
            'elapsed_wall_sec': now - self.started, 'final': final,
            'clock': dict(clock, healthy=clock_ok),
            'system': self.system_metrics(), 'sensors': sensors, 'gazebo': self.gazebo_metrics(),
        }
        temporary = self.directory / 'report.json.tmp'
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.directory / 'report.json')
        with (self.directory / 'samples.jsonl').open('a') as output:
            output.write(json.dumps(report, allow_nan=False) + '\n')

        def number(value):
            return f'{value:.2f}' if value is not None else 'unknown'

        lines = ['# GuideDog sensor report', '',
                 f'Elapsed: {report["elapsed_wall_sec"]:.1f} wall seconds. '
                 f'Clock healthy: {clock_ok}.',
                 f'Ubuntu CPU busy: {number(report["system"].get("cpu_busy_percent"))}%. '
                 f'Available RAM: {number(report["system"].get("memory_available_mib"))} MiB.',
                 f'Simulation/wall speed: {number(clock["source_time_per_wall_second"])}.', '',
                 '| Sensor | Status | ROS Hz (wall) | GZ Hz (latest) | TF | Issues |',
                 '| --- | --- | --- | --- | --- | --- |']
        for name, stats in sensors.items():
            wall = stats['received_hz_wall']
            source = report['gazebo'].get(name, {}).get('latest_received_hz_wall')
            lines.append(f'| {name} | {stats["status"]} | '
                         f'{number(wall)} | {number(source)} | '
                         f'{stats["tf_to_base_at_message_time"]} | '
                         f'{", ".join(stats["issues"])} |')
        lines.extend(['', 'Receipt rates do not establish publisher rates or message loss.',
                      'GZ and ROS rate windows differ; check JSON log age before comparing.',
                      'Diagnostic thresholds are not validated motion safety limits.',
                      'Gazebo source measurements, when enabled, are in gazebo-*.log.', ''])
        (self.directory / 'report.md').write_text('\n'.join(lines))
        summary = ' | '.join(
            f'{name}: {stats["status"]}, {stats["received_hz_wall"] or 0:.1f} Hz wall'
            for name, stats in sensors.items())
        if rclpy.ok():
            self.get_logger().info(summary)
        if self.config['duration_sec'] and now - self.started >= self.config['duration_sec']:
            self.finished = True


def main(args=None):
    """Observe until interrupted or the requested wall-time duration expires."""
    rclpy.init(args=args)
    node = None
    try:
        node = SensorMonitor()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.write_report(final=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
