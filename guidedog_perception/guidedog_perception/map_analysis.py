# SPDX-License-Identifier: Apache-2.0
"""Read-only geometry and observed-space connectivity checks, without motion commands."""

from collections import deque
import hashlib
import math
import struct

import numpy as np


def validate_grid(data, origin, resolution):
    """Reject malformed class grids and coordinate metadata before interpretation."""
    grid, origin = np.asarray(data), np.asarray(origin, dtype=float)
    if (grid.ndim != 2 or min(grid.shape) == 0 or max(grid.shape) > 200
            or not np.isin(grid, [-1, 0, 100]).all()
            or origin.shape != (2,) or not np.isfinite(origin).all()
            or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError('invalid_map_snapshot')
    return grid.astype(np.int8), origin


def map_digest(data, origin, resolution, stamp_ns):
    """Pair map content and status even when ageing changes cells at the same stamp."""
    grid, origin = validate_grid(data, origin, resolution)
    metadata = struct.pack('!qddfII', stamp_ns, *origin, resolution, *grid.shape)
    return hashlib.sha256(metadata + grid.tobytes(order='C')).hexdigest()


def cell_centers(data, origin, resolution):
    """Return odom coordinates for the actual advertised cell centres."""
    grid, origin = validate_grid(data, origin, resolution)
    row, col = np.indices(grid.shape)
    return np.stack((origin[0]+(col+0.5)*resolution,
                     origin[1]+(row+0.5)*resolution), axis=-1)


def footprint_free(data, resolution, radius):
    """Require observed ground under a circular envelope; unknown/outside are blocked."""
    grid, _ = validate_grid(data, [0, 0], resolution)
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('invalid_assessment_radius')
    free = grid == 0
    if radius == 0:
        return free
    # Include cells whose squares can intersect the circle, not just their centres.
    reach_m = radius + resolution/math.sqrt(2)
    if reach_m >= max(grid.shape)*resolution:
        return np.zeros(grid.shape, dtype=bool)
    reach = math.ceil(reach_m/resolution)
    height, width = grid.shape
    result = free.copy()
    for dy in range(-reach, reach+1):
        for dx in range(-reach, reach+1):
            if math.hypot(dx, dy)*resolution > reach_m:
                continue
            shifted = np.zeros_like(free)
            y0, y1 = max(0, -dy), min(height, height-dy)
            x0, x1 = max(0, -dx), min(width, width-dx)
            if y1 > y0 and x1 > x0:
                shifted[y0:y1, x0:x1] = free[y0+dy:y1+dy, x0+dx:x1+dx]
            result &= shifted
    return result


def flood(mask, start):
    """Use four-neighbour connectivity without diagonal corner cutting."""
    mask = np.asarray(mask, dtype=bool)
    reached = np.zeros_like(mask)
    row, col = start
    if not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1]) or not mask[row, col]:
        return reached
    queue = deque([(row, col)])
    reached[row, col] = True
    while queue:
        row, col = queue.popleft()
        for y, x in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
            if (0 <= y < mask.shape[0] and 0 <= x < mask.shape[1]
                    and mask[y, x] and not reached[y, x]):
                reached[y, x] = True
                queue.append((y, x))
    return reached


def largest_component(mask):
    """Count the largest observed component without seeding blind space as free."""
    visited = np.zeros_like(mask, dtype=bool)
    largest = 0
    for row, col in zip(*np.nonzero(mask)):
        if not visited[row, col]:
            reached = flood(mask & ~visited, (row, col))
            visited |= reached
            largest = max(largest, int(reached.sum()))
    return largest


def assess_connectivity(data, origin, resolution, robot_xy, goal_xy, radius, healthy=True):
    """Report planning readiness separately from visible-box localization success."""
    grid, origin = validate_grid(data, origin, resolution)
    points = np.asarray([robot_xy, goal_xy], dtype=float)
    if points.shape != (2, 2) or not np.isfinite(points).all():
        raise ValueError('invalid_assessment_positions')
    cells = np.floor((points-origin)/resolution).astype(int)
    start, goal = [(int(y), int(x)) for x, y in cells]
    available = footprint_free(grid, resolution, radius)
    reached = flood(available, start)

    def state(cell):
        y, x = cell
        return int(grid[y, x]) if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] else None

    if not healthy:
        reason = 'stale_or_invalid_map'
    elif state(start) is None:
        reason = 'start_outside_map'
    elif state(start) == -1:
        reason = 'start_unknown'
    elif state(start) == 100:
        reason = 'start_occupied'
    elif not reached.any():
        reason = 'start_envelope_not_fully_observed'
    elif state(goal) is None:
        reason = 'goal_outside_map'
    elif not reached[goal]:
        reason = 'no_observed_connection_for_envelope'
    else:
        reason = 'observed_connection'
    return {'planning_ready': reason == 'observed_connection', 'reason': reason,
            'assessment_radius_m': radius, 'start_cell_state': state(start),
            'goal_cell_state': state(goal), 'observed_free_cells': int((grid == 0).sum()),
            'largest_point_free_component_cells': largest_component(grid == 0),
            'envelope_free_cells': int(available.sum()),
            'largest_envelope_free_component_cells': largest_component(available),
            'reachable_envelope_cells_from_start': int(reached.sum()),
            'unknown_cells': int((grid == -1).sum())}


def rectangle_signed_distance(points, pose, size):
    """Measure exact planar distance to a known oriented rectangular fixture."""
    points, pose, size = np.asarray(points), np.asarray(pose), np.asarray(size)
    if (points.ndim < 1 or points.shape[-1] != 2 or pose.shape != (3,)
            or size.ndim != 1 or len(size) < 2
            or not np.isfinite(points).all() or not np.isfinite(pose).all()
            or not np.isfinite(size).all() or (size[:2] <= 0).any()):
        raise ValueError('invalid_reference_rectangle')
    delta = points-pose[:2]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    local = np.stack((c*delta[..., 0]+s*delta[..., 1],
                      -s*delta[..., 0]+c*delta[..., 1]), axis=-1)
    distance = np.abs(local)-size[:2]/2
    return (np.linalg.norm(np.maximum(distance, 0), axis=-1)
            + np.minimum(np.max(distance, axis=-1), 0))


def assess_boxes(data, origin, resolution, boxes, association_gate=0.50):
    """Associate cells once and count free cells wholly inside reference obstacles."""
    grid, _ = validate_grid(data, origin, resolution)
    if not boxes or not math.isfinite(association_gate) or association_gate <= 0:
        raise ValueError('invalid_reference_boxes')
    names = [b['name'] for b in boxes]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError('duplicate_or_empty_reference_name')
    centres = cell_centers(grid, origin, resolution)
    distances = np.stack([rectangle_signed_distance(centres, b['pose'], b['size']) for b in boxes])
    absolute = np.abs(distances)
    owner = np.argmin(absolute, axis=0)
    nearest = np.min(absolute, axis=0)
    result = {}
    for i, box in enumerate(boxes):
        selected = (grid == 100) & (owner == i) & (nearest <= association_gate)
        errors = absolute[i][selected]
        interior = distances[i] < -resolution/math.sqrt(2)
        result[box['name']] = {
            'associated_occupied_cells': int(selected.sum()),
            'boundary_error_p95_m': float(np.percentile(errors, 95)) if len(errors) else None,
            'boundary_error_max_m': float(errors.max()) if len(errors) else None,
            'whole_interior_cells': int(interior.sum()),
            'free_wholly_inside_box_cells': int((interior & (grid == 0)).sum()),
            'unknown_wholly_inside_box_cells': int((interior & (grid == -1)).sum())}
    return {'boxes': result,
            'unmatched_occupied_cells': int(((grid == 100) & (nearest > association_gate)).sum()),
            'association_gate_m': association_gate,
            'free_wholly_inside_boxes_cells': sum(
                b['free_wholly_inside_box_cells'] for b in result.values())}
