# SPDX-License-Identifier: Apache-2.0
"""Conservative floor-coverage diagnostics, never an input to motion control."""

import math

from guidedog_perception.map_analysis import validate_grid
import numpy as np


ASSESSMENT_RADIUS = math.hypot(0.45, 0.40) + 0.10


def point_range_masks(points, minimum, maximum):
    """Distinguish missing and saturated range values from candidate surface returns."""
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not math.isfinite(minimum)
            or not math.isfinite(maximum) or not 0 <= minimum < maximum):
        raise ValueError('invalid_range_contract')
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    low = finite & (ranges <= minimum+1e-5)
    high = finite & (ranges >= maximum-1e-5)
    return finite, low, high, finite & ~low & ~high


def support_coverage(data, origin, resolution, robot_xy, *, healthy):
    """
    Count every grid cell intersecting the unchanged diagnostic body circle.

    Free means the mapper's observed flat-floor label, not continuous surface proof.
    Missing cells and unhealthy observations always withhold readiness.
    """
    grid, origin = validate_grid(data, origin, resolution)
    center = np.asarray(robot_xy, dtype=float)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError('invalid_robot_position')
    lo = np.floor((center-ASSESSMENT_RADIUS-origin)/resolution).astype(int)
    hi = np.floor((center+ASSESSMENT_RADIUS-origin)/resolution).astype(int)
    y, x = np.mgrid[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
    lower = origin + np.stack((x, y), axis=-1)*resolution
    distance = np.maximum(np.maximum(lower-center, center-(lower+resolution)), 0)
    required = (distance**2).sum(axis=-1) <= ASSESSMENT_RADIUS**2 + 1e-12
    inside = (x >= 0) & (y >= 0) & (x < grid.shape[1]) & (y < grid.shape[0])
    values = np.full(x.shape, -1, dtype=np.int8)
    values[inside] = grid[y[inside], x[inside]]
    counts = {name: int((required & (values == code) & inside).sum())
              for name, code in [('observed_floor', 0), ('unknown', -1), ('occupied', 100)]}
    counts['outside_map'] = int((required & ~inside).sum())
    count = int(required.sum())
    supported = bool(healthy and counts['observed_floor'] == count)
    return {'schema_version': 1, 'diagnostic_only': True,
            'scope': 'observed flat-floor grid cells; not continuous terrain or motion approval',
            'assessment_radius_m': ASSESSMENT_RADIUS, 'required_cells': count,
            'cells': counts, 'observed_floor_fraction': counts['observed_floor']/count,
            'map_healthy': bool(healthy), 'envelope_observed': supported,
            'reason': ('stale_or_invalid_map' if not healthy else
                       'observed_grid_envelope' if supported else 'incomplete_floor_evidence')}


def scan_floor_coverage(local, plane):
    """
    Measure one scan without turning the fitted plane or body returns into floor.

    The raw candidate count inside the body mask is explicitly untrusted: legs and
    feet can lie near the fitted ground plane. No self-filter exception is made.
    """
    points = np.asarray(local, dtype=float)
    plane = np.asarray(plane, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or plane.shape != (3,)
            or not np.isfinite(plane).all()):
        raise ValueError('invalid_floor_scan')
    points = points[np.isfinite(points).all(axis=1)]
    height = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
    ground = np.abs(height) <= 0.025
    body = (np.abs(points[:, 0]) <= 0.45) & (np.abs(points[:, 1]) <= 0.40)
    near = np.linalg.norm(points[:, :2], axis=1) <= ASSESSMENT_RADIUS
    # Same support threshold and self mask as the existing mapper, in body coordinates.
    n, resolution = 20, 0.1
    origin = np.array([-1., -1.])
    cells = np.floor((points[:, :2]-origin)/resolution).astype(int)
    valid = ~body & ((cells >= 0) & (cells < n)).all(axis=1)
    index = cells[valid, 1]*n+cells[valid, 0]
    floors = np.bincount(index[ground[valid]], minlength=n*n).reshape(n, n)
    other = np.bincount(index[~ground[valid]], minlength=n*n).reshape(n, n)
    grid = np.where((floors >= 3) & (other == 0), 0, -1).astype(np.int8)
    return {'finite_points': len(points), 'near_plane_points': int(ground.sum()),
            'near_plane_points_in_envelope': int((ground & near).sum()),
            'untrusted_near_plane_points_inside_body_mask': int((ground & body).sum()),
            'nearest_trusted_floor_range_xy_m': (
                float(np.linalg.norm(points[ground & ~body, :2], axis=1).min())
                if (ground & ~body).any() else None),
            'single_scan_coverage': support_coverage(grid, origin, resolution, [0, 0],
                                                     healthy=True)}, grid, floors
