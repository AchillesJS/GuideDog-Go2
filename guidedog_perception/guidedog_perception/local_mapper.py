# SPDX-License-Identifier: Apache-2.0
"""Read-only flat-ground mapping with exact-time TF and wall-time evidence expiry."""

from collections import deque
import json
import math
from pathlib import Path
import time

from guidedog_perception.guard_geometry import cloud_xyz, inspect_corridor, transform_xyz
from guidedog_perception.local_grid import LocalGrid
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def apply_tf(points, transform):
    """Apply the transform convention shared with the direct raw-sensor veto."""
    t, q = transform.transform.translation, transform.transform.rotation
    return transform_xyz(points, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def write_svg(path, layers, resolution, robot):
    """Render a compact view using the exact published cells and unknown semantics."""
    grid = layers['occupancy']
    n = len(grid)
    colors = {0: '#78bb9c', 100: '#e45a58'}
    rectangles = []
    for row, values in enumerate(grid):
        boundaries = np.r_[0, np.flatnonzero(values[1:] != values[:-1])+1, n]
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            if values[start] in colors:
                rectangles.append(f'<rect x="{start}" y="{n-row-1}" width="{end-start}" '
                                  f'height="1" fill="{colors[values[start]]}"/>')
    x, y, yaw = robot
    text = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-2 -9 {n+4} {n+20}">'
            '<rect x="-2" y="-9" width="100%" height="100%" fill="#17202b"/>'
            '<text x="0" y="-4" fill="white" font-size="3">GuideDog local map</text>'
            f'<rect width="{n}" height="{n}" fill="#535b66"/>' + ''.join(rectangles) +
            f'<circle cx="{x:.2f}" cy="{n-y:.2f}" r="1.5" fill="#559aff"/>'
            f'<path d="M{x:.2f},{n-y:.2f} l{4*math.cos(yaw):.2f},'
            f'{-4*math.sin(yaw):.2f}" stroke="white" stroke-width="0.5"/>'
            f'<text x="0" y="{n+5}" fill="white" font-size="2.2">'
            f'green: observed floor | red: obstacle | gray: unknown</text>'
            f'<text x="0" y="{n+9}" fill="white" font-size="2.2">'
            f'{n*resolution:.1f} m square | cell {resolution:.2f} m</text></svg>')
    temporary = path.with_suffix('.svg.tmp')
    temporary.write_text(text)
    temporary.replace(path)


class LocalMapper(Node):
    """Publish observations only; neither map nor ground truth controls the robot."""

    def __init__(self):
        """Create bounded subscriptions, storage and a steady publication clock."""
        super().__init__('guidedog_local_mapper')
        defaults = {'lidar_topic': '/unitree_lidar/points', 'map_frame': 'odom',
                    'base_frame': 'base_footprint', 'resolution': 0.1, 'size': 8.0,
                    'free_ttl': 2.0, 'occupied_ttl': 5.0, 'update_period': 0.2,
                    'report_dir': str(Path.home() / '.ros/guidedog/local_map')}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        if not math.isfinite(self.config['update_period']) or self.config['update_period'] < 0.1:
            raise ValueError('update_period_must_be_at_least_0.1_wall_seconds')
        self.grid = LocalGrid(**{k: self.config[k] for k in
                                 ('resolution', 'size', 'free_ttl', 'occupied_ttl')})
        self.directory = Path(self.config['report_dir']).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=8)
        self.costs = deque(maxlen=200)
        self.received = self.accepted = self.rejected = 0
        self.last_source = None
        self.last_seen_source = None
        self.last_received = self.last_update = None
        self.last_ros = self.advanced_at = None
        self.reason = 'waiting_for_scan'
        self.pose = [0.0, 0.0, 0.0]
        self.last_report = 0.0
        self.output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            OccupancyGrid, '/guidedog/local_map', self.output_qos)
        self.status_publisher = self.create_publisher(String, '/guidedog/local_map_status', 1)
        self.subscription = self.create_subscription(
            PointCloud2, self.config['lidar_topic'], self.on_cloud,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)

    def on_cloud(self, msg):
        """Retain the original receipt time; stale/replayed data cannot refresh cells."""
        self.received += 1
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        if self.last_seen_source is not None and stamp <= self.last_seen_source:
            self.rejected += 1
            self.reason = 'replayed_or_out_of_order_scan'
            return
        self.last_seen_source = stamp
        self.pending.append((msg, time.monotonic()))

    def process(self, now, ros_now):
        """Select the newest exactly transformable fresh scan without blocking TF."""
        if self.last_update is not None and now - self.last_update < self.config['update_period']:
            return
        selected = None
        for index in range(len(self.pending)-1, -1, -1):
            msg, received = self.pending[index]
            stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
            if now - received > 0.25:
                continue
            if self.last_source is not None and stamp <= self.last_source:
                continue
            if not msg.header.frame_id or not -0.05 <= (ros_now-stamp)/1e9 <= 0.5:
                continue
            try:
                source_time = Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME)
                local_tf = self.buffer.lookup_transform(
                    self.config['base_frame'], msg.header.frame_id, source_time)
                map_tf = self.buffer.lookup_transform(
                    self.config['map_frame'], self.config['base_frame'], source_time)
                selected = index
                break
            except TransformException:
                continue
        if selected is None:
            while self.pending and now - self.pending[0][1] > 0.25:
                self.pending.popleft()
                self.rejected += 1
                self.reason = 'stale_replayed_invalid_or_missing_tf'
            return
        for _ in range(selected+1):
            self.pending.popleft()
        started = time.monotonic()
        try:
            local = apply_tf(cloud_xyz(msg), local_tf)
            local = local[np.isfinite(local).all(axis=1)]
            plane = inspect_corridor(local)['ground_plane_z_ax_by_c']
            if plane is None:
                raise ValueError('unobserved_flat_support')
            t, q = map_tf.transform.translation, map_tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            self.grid.observe(local, apply_tf(local, map_tf), plane,
                              [t.x, t.y], stamp, received)
            self.pose = [t.x, t.y, yaw]
            self.last_source, self.last_received = stamp, received
            self.accepted += 1
            self.reason = 'observing'
        except (ValueError, TypeError, BufferError) as error:
            self.rejected += 1
            self.reason = f'invalid_scan:{error}'
        self.last_update = now
        self.costs.append((time.monotonic()-started)*1000)

    def tick(self):
        """Keep ageing data while paused or disconnected; never re-stamp old observations."""
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
        if self.last_ros is not None and ros_now < self.last_ros:
            self.grid.reset()
            self.pending.clear()
            self.last_source = self.last_received = None
            self.last_seen_source = None
            self.reason = 'clock_reset'
        if ros_now > 0 and ros_now != self.last_ros:
            self.advanced_at = now
        self.last_ros = ros_now
        clock_ok = self.advanced_at is not None and now - self.advanced_at <= 0.5
        if clock_ok:
            self.process(now, ros_now)
        layers = self.grid.snapshot(now)
        msg = OccupancyGrid()
        msg.header.frame_id = self.config['map_frame']
        stamp = self.last_source or 0
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
        msg.info.resolution = self.grid.resolution
        msg.info.width = msg.info.height = self.grid.n
        msg.info.origin.position.x, msg.info.origin.position.y = self.grid.origin.tolist()
        msg.info.origin.orientation.w = 1.0
        msg.data = layers['occupancy'].reshape(-1).tolist()
        # Publication is bounded at 5 Hz; expiry/reporting never depends on source time.
        if now - getattr(self, 'published_at', -math.inf) < 0.2:
            return
        self.published_at = now
        self.publisher.publish(msg)
        age = now-self.last_received if self.last_received is not None else None
        status = {'schema_version': 1, 'simulation_flat_ground_only': True,
                  'healthy': clock_ok and age is not None and age <= 0.5
                  and self.reason == 'observing',
                  'reason': self.reason if clock_ok else 'clock_not_advancing',
                  'source_stamp_ns': self.last_source, 'source_receipt_age_wall_sec': age,
                  'clock_ok': clock_ok, 'received': self.received, 'accepted': self.accepted,
                  'rejected': self.rejected, 'parameters': self.config,
                  'pose_odom': self.pose, 'origin_odom': self.grid.origin.tolist(),
                  'rolls': self.grid.rolls, 'resets': self.grid.resets,
                  'cell_array_bytes': self.grid.memory_bytes,
                  'update_cost_p95_ms': (float(np.percentile(self.costs, 95))
                                         if self.costs else None),
                  'cells': {name: int((layers['occupancy'] == code).sum())
                            for name, code in (('unknown', -1), ('free', 0), ('occupied', 100))},
                  'evidence_note': 'score is support count, not a calibrated probability'}
        if age is not None and age > 0.5 and clock_ok:
            status['reason'] = 'stale_scan'
        self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))
        if now-self.last_report >= 2.0:
            temporary = self.directory / 'local_map_report.json.tmp'
            temporary.write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')
            temporary.replace(self.directory / 'local_map_report.json')
            with (self.directory / 'local_map.npz.tmp').open('wb') as output:
                np.savez_compressed(output, **layers, origin_odom=self.grid.origin,
                                    resolution=self.grid.resolution, source_stamp_ns=stamp)
            (self.directory / 'local_map.npz.tmp').replace(self.directory / 'local_map.npz')
            robot = ((np.array(self.pose[:2])-self.grid.origin)/self.grid.resolution).tolist()
            write_svg(self.directory / 'local_map.svg', layers, self.grid.resolution,
                      [*robot, self.pose[2]])
            self.last_report = now


def main(args=None):
    """Run an observation-only node with no velocity publishers or truth inputs."""
    rclpy.init(args=args)
    node = LocalMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
