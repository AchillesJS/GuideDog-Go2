# SPDX-License-Identifier: Apache-2.0
"""Point-cloud geometry for the flat-ground, straight-line simulation demo."""

import math

import numpy as np


def cloud_xyz(msg, max_points=250000):
    """Decode XYZ while validating layout, endian order and padded rows."""
    if not 0 < msg.width * msg.height <= max_points:
        raise ValueError('empty_or_oversized_cloud')
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('invalid_cloud_layout')
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError('truncated_cloud')
    fields = {field.name: field for field in msg.fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('invalid_xyz_fields')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > msg.point_step:
            raise ValueError('invalid_xyz_offset')
        formats.append(('>' if msg.is_bigendian else '<') + f'f{size}')
        offsets.append(field.offset)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.stack([data[name] for name in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)


def transform_xyz(points, translation, quaternion):
    """Apply a finite rigid transform; reject invalid quaternions."""
    translation = np.asarray(translation, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if not np.isfinite(translation).all() or not np.isfinite(q).all():
        raise ValueError('nonfinite_transform')
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError('invalid_quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return points @ rotation.T + translation


def ground_plane(points, min_points):
    """Fit observed nearly horizontal support despite bounded footprint height error."""
    candidates = points[(points[:, 2] >= -0.20) & (points[:, 2] <= 0.15)]
    if len(candidates) < min_points:
        return None
    if len(candidates) > 1024:
        candidates = candidates[np.linspace(0, len(candidates) - 1, 1024, dtype=int)]
    design = np.column_stack((candidates[:, :2], np.ones(len(candidates))))
    random = np.random.default_rng(0)
    best = None
    best_count = 0
    for trial in range(33):
        sample = np.arange(len(candidates)) if trial == 0 else random.choice(
            len(candidates), 3, replace=False)
        plane, _, rank, _ = np.linalg.lstsq(design[sample], candidates[sample, 2], rcond=None)
        if rank < 3 or np.linalg.norm(plane[:2]) > math.tan(math.radians(10)):
            continue
        # A uniformly elevated surface must not be relabelled as clear ground.
        if not -0.15 <= plane[2] <= 0.055:
            continue
        inliers = np.abs(candidates[:, 2] - design @ plane) <= 0.025
        count = int(inliers.sum())
        if count < min_points or count <= best_count:
            continue
        extent = np.ptp(candidates[inliers, :2], axis=0)
        if extent[0] < 0.40 or extent[1] < 0.20:
            continue
        best, best_count = inliers, count
    if best is None:
        return None
    plane = np.linalg.lstsq(design[best], candidates[best, 2], rcond=None)[0]
    if (not -0.15 <= plane[2] <= 0.055
            or np.linalg.norm(plane[:2]) > math.tan(math.radians(10))):
        return None
    return plane


def inspect_corridor(points, *, front=0.45, half_width=0.40,
                     min_height=0.08, max_height=0.85,
                     support_range=3.0, min_ground_points=8):
    """Measure forward clearance and require visible flat ground to report clear."""
    finite = points[np.isfinite(points).all(axis=1)]
    if len(finite) < 32:
        raise ValueError('insufficient_finite_points')
    x, y, z = finite.T
    forward = (x >= front) & (x <= front + support_range) & (np.abs(y) <= half_width)
    plane = ground_plane(finite[forward], min_ground_points)
    height = z if plane is None else z - (plane[0]*x + plane[1]*y + plane[2])
    obstacle = forward & (height >= min_height) & (height <= max_height)
    ground = forward & (np.abs(height) <= 0.025) if plane is not None else np.zeros(
        len(finite), dtype=bool)
    clearance = float(np.min(x[obstacle]) - front) if obstacle.any() else None
    if clearance is not None and not math.isfinite(clearance):
        raise ValueError('nonfinite_clearance')
    return {'clearance_m': clearance, 'obstacle_points': int(obstacle.sum()),
            'ground_points': int(ground.sum()),
            'ground_observed': int(ground.sum()) >= min_ground_points,
            'ground_plane_z_ax_by_c': plane.tolist() if plane is not None else None,
            'forward_points': int(forward.sum()),
            'forward_z_percentiles_m': ([float(v) for v in np.percentile(z[forward], [10, 50, 90])]
                                        if forward.any() else None),
            'finite_points': len(finite)}


class MotionGate:
    """Forward-only command policy with explicit neutral-command rearming."""

    def __init__(self, max_speed=0.10, stop_distance=0.80,
                 scan_timeout=0.80, command_timeout=0.40):
        for value in (max_speed, stop_distance, scan_timeout, command_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('gate parameters must be finite and positive')
        if max_speed * scan_timeout > 0.08 + 1e-9:
            raise ValueError('commanded travel during scan timeout must not exceed 0.08 m')
        self.max_speed = max_speed
        self.stop_distance = stop_distance
        self.scan_timeout = scan_timeout
        self.command_timeout = command_timeout
        self.command = None
        self.command_at = None
        self.scan = None
        self.scan_at = None
        self.scan_stamp = None
        self.fault = 'waiting_for_scan'
        self.latched = True
        self.neutral_after_fault = None
        self.last_fault_at = -math.inf

    def invalidate(self, reason, now):
        """Invalidate the previous clear observation immediately."""
        self.fault = reason
        self.latched = True
        self.last_fault_at = now

    def observe(self, scan, received, stamp):
        """Reject replayed timestamps without refreshing scan freshness."""
        if self.scan_stamp is not None and stamp <= self.scan_stamp:
            self.invalidate('nonadvancing_scan_stamp', received)
            return
        self.scan, self.scan_at, self.scan_stamp = scan, received, stamp
        self.fault = None

    def request(self, components, now):
        """Only a finite, straight, nonnegative request can reach the output."""
        self.command_at = now
        if (len(components) != 6 or not all(math.isfinite(v) for v in components)
                or components[0] < 0 or any(abs(v) > 1e-6 for v in components[1:])):
            self.command = None
            self.invalidate('unsupported_or_invalid_command', now)
            return
        self.command = min(components[0], self.max_speed)
        if self.command == 0:
            self.neutral_after_fault = now

    def decide(self, now, *, clock_ok=True, stamp_age=None):
        """Evaluate all freshness checks in monotonic wall time."""
        reason = self.fault
        if not clock_ok:
            reason = 'clock_stalled'
        elif self.scan_at is None:
            reason = reason or 'waiting_for_scan'
        elif now - self.scan_at > self.scan_timeout:
            reason = 'stale_scan'
        elif stamp_age is None or stamp_age < -0.10 or stamp_age > self.scan_timeout:
            reason = 'invalid_or_old_scan_timestamp'
        elif (self.scan['clearance_m'] is not None
              and self.scan['clearance_m'] <= self.stop_distance):
            reason = 'obstacle'
        elif not self.scan['ground_observed']:
            reason = 'unknown_forward_ground'
        if reason:
            self.latched = True
            self.last_fault_at = now
            return 0.0, reason
        if self.command_at is None or now - self.command_at > self.command_timeout:
            self.latched = True
            self.last_fault_at = now
            return 0.0, 'command_timeout'
        if self.command is None:
            return 0.0, 'unsupported_or_invalid_command'
        if self.latched:
            if (self.command == 0 and self.neutral_after_fault is not None
                    and self.neutral_after_fault > self.last_fault_at):
                self.latched = False
            else:
                return 0.0, 'neutral_command_required'
        return self.command, 'moving' if self.command else 'idle'
