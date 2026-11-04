# SPDX-License-Identifier: Apache-2.0
"""Independent time-aligned references for the flat-world clearance experiment."""

from bisect import bisect_left
from collections import deque
import math

from guidedog_perception.guard_geometry import transform_xyz
import numpy as np


def yaw_of(q):
    """Return planar heading from a finite normalized XYZW quaternion."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError('invalid_reference_quaternion')
    x, y, z, w = q / norm
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


class PoseHistory:
    """Interpolate only bracketed poses with a bounded source-time gap."""

    def __init__(self):
        """Bound memory to recent ground-truth samples."""
        self.values = deque(maxlen=1000)

    def add(self, stamp, position, quaternion):
        """Reject invalid poses and clear history on a source-clock reset."""
        p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
        if not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
            raise ValueError('invalid_reference_pose')
        if self.values and stamp < self.values[-1][0]:
            self.values.clear()
        if self.values and stamp == self.values[-1][0]:
            return
        self.values.append((stamp, p, q / np.linalg.norm(q)))

    def at(self, stamp, max_gap_ns=60000000):
        """Use linear translation and shortest-arc quaternion interpolation."""
        values = list(self.values)
        index = bisect_left([v[0] for v in values], stamp)
        if index < len(values) and values[index][0] == stamp:
            return values[index][1], values[index][2], 0.0
        if index == 0 or index == len(values):
            return None
        a, b = values[index - 1], values[index]
        gap = b[0] - a[0]
        if gap > max_gap_ns:
            return None
        fraction = (stamp - a[0]) / gap
        qa, qb = a[2], b[2]
        dot = float(np.dot(qa, qb))
        if dot < 0:
            qb, dot = -qb, -dot
        dot = min(1.0, dot)
        if dot > 0.9995:
            q = qa + fraction*(qb - qa)
        else:
            angle = math.acos(dot)
            q = (math.sin((1 - fraction)*angle)*qa + math.sin(fraction*angle)*qb) / math.sin(angle)
        return a[1] + fraction*(b[1] - a[1]), q / np.linalg.norm(q), gap / 1e9


def level_points(body_points, position, quaternion):
    """Reproject the same cloud using actual body pose into a horizontal yaw frame."""
    world = transform_xyz(body_points, position, quaternion)
    yaw = yaw_of(quaternion)
    c, s = math.cos(yaw), math.sin(yaw)
    delta = world - np.array([position[0], position[1], 0])
    return np.column_stack((c*delta[:, 0] + s*delta[:, 1],
                            -s*delta[:, 0] + c*delta[:, 1], delta[:, 2]))


def box_clearance(box, size, position, yaw, front=0.45, half_width=0.40):
    """Clip an oriented box to the observed corridor before measuring its front gap."""
    c, s = math.cos(box[2]), math.sin(box[2])
    corners = []
    cy, sy = math.cos(yaw), math.sin(yaw)
    for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x, y = x*size[0]/2, y*size[1]/2
        dx, dy = box[0] + c*x - s*y - position[0], box[1] + s*x + c*y - position[1]
        corners.append(np.array([cy*dx + sy*dy, -sy*dx + cy*dy]))
    projected = min(p[0] for p in corners) - front
    polygon = corners
    for axis, bound, sign in ((1, half_width, 1), (1, -half_width, -1), (0, front, -1)):
        clipped = []
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            da, db = sign*(a[axis] - bound), sign*(b[axis] - bound)
            if da <= 0:
                clipped.append(a)
            if (da <= 0) != (db <= 0):
                clipped.append(a + da/(da - db)*(b - a))
        polygon = clipped
        if not polygon:
            return None, projected
    return max(0.0, min(p[0] for p in polygon) - front), projected
