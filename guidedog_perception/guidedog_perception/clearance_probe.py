# SPDX-License-Identifier: Apache-2.0
"""Observe independent clearance references without providing data to motion control."""

from collections import deque
import json
import threading
import time

from guidedog_perception.clearance_metrics import box_clearance, level_points, PoseHistory, yaw_of
from guidedog_perception.guard_geometry import cloud_xyz, inspect_corridor, transform_xyz
import numpy as np
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class ClearanceProbe:
    """Compare one source cloud in estimated and ground-truth reference frames."""

    def __init__(self, node, directory, filename='clearance-samples.jsonl'):
        """Keep bounded histories and a separate audit file, never a command publisher."""
        self.node = node
        self.history = PoseHistory()
        self.pending = deque(maxlen=8)
        self.records = deque(maxlen=10000)
        self.box = None
        self.phase = 'startup'
        self.lock = threading.Lock()
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.output = (directory / filename).open('w')
        self.timer = node.create_timer(0.025, self.process, clock=node.steady_clock)
        self.received = 0
        self.unpaired = 0
        self.invalid = 0
        self.last_received = None
        self.max_receipt_gap = 0.0

    def on_pose(self, msg):
        """Accept only the explicitly configured Gazebo ground-truth body pose."""
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise ValueError('unexpected_ground_truth_frames')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        self.history.add(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def on_cloud(self, msg):
        """Retain actual incoming scans and their original receipt times."""
        now = time.monotonic()
        if self.last_received is not None:
            self.max_receipt_gap = max(self.max_receipt_gap, now - self.last_received)
        self.last_received = now
        self.received += 1
        if len(self.pending) == self.pending.maxlen:
            self.unpaired += 1
        self.pending.append((msg, now))

    def set_box(self, box, size=(0.5, 1.0, 1.0), phase='front_obstacle'):
        """Update the known test fixture; reference values never enter the veto."""
        self.box, self.phase = (box, size), phase

    @staticmethod
    def apply(points, transform):
        """Apply a TF rigid transform using the project cloud decoder conventions."""
        t, q = transform.transform.translation, transform.transform.rotation
        return transform_xyz(points, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def process(self):
        """Wait for bracketed reference poses and exact-time TF, with bounded delay."""
        now = time.monotonic()
        while self.pending:
            msg, received = self.pending[0]
            stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
            reference = self.history.at(stamp)
            try:
                body_tf = self.buffer.lookup_transform(
                    'base_link', msg.header.frame_id,
                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
                footprint_tf = self.buffer.lookup_transform(
                    'base_footprint', msg.header.frame_id,
                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
            except TransformException:
                body_tf = footprint_tf = None
            if reference is None or body_tf is None:
                if now - received < 0.25:
                    return
                self.pending.popleft()
                self.unpaired += 1
                continue
            self.pending.popleft()
            points = cloud_xyz(msg)
            points = points[np.isfinite(points).all(axis=1)]
            position, quaternion, bracket_gap = reference
            try:
                estimated = inspect_corridor(self.apply(points, footprint_tf))
                truth = inspect_corridor(level_points(
                    self.apply(points, body_tf), position, quaternion))
            except ValueError:
                self.invalid += 1
                continue
            box = self.box
            record = {
                'stamp_ns': stamp, 'received_monotonic': received, 'phase': self.phase,
                'pose_bracket_sec': bracket_gap,
                'truth_position': position.tolist(), 'truth_quaternion': quaternion.tolist(),
                'tf_clearance_m': estimated['clearance_m'],
                'truth_cloud_clearance_m': truth['clearance_m'],
                'estimated_ground_plane': estimated['ground_plane_z_ax_by_c'],
                'truth_ground_plane': truth['ground_plane_z_ax_by_c'],
            }
            if box is not None:
                clipped, projected = box_clearance(
                    *box, position, yaw_of(quaternion))
                record.update(box_corridor_clearance_m=clipped,
                              box_projected_clearance_m=projected)
            with self.lock:
                self.records.append(record)
            self.output.write(json.dumps(record, allow_nan=False) + '\n')

    def snapshot(self):
        """Return a stable copy for the scenario thread."""
        self.output.flush()
        with self.lock:
            return list(self.records)

    def close(self):
        """Flush evidence before the ROS node is destroyed."""
        self.timer.cancel()
        self.output.close()
