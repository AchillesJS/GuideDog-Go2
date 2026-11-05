# SPDX-License-Identifier: Apache-2.0
"""Exercise conservative coverage boundaries and missing-floor evidence."""

from guidedog_perception.support_evidence import (
    point_range_masks, scan_floor_coverage, support_coverage)
import numpy as np
import pytest


def assess(grid, center=(0, 0), healthy=True):
    """Use the same fixed world extent for all coverage assertions."""
    return support_coverage(grid, [-1, -1], 0.1, center, healthy=healthy)


def test_missing_and_saturated_ranges_are_not_surface_observations():
    """Boundary clamping and nonfinite rays must not count as actual nearby floor."""
    raw = [[.8, 0, 0], [.800001, 0, 0], [1, 0, 0], [30, 0, 0],
           [np.nan, 0, 0], [np.inf, 0, 0], [0, 0, 0]]
    finite, low, high, valid = point_range_masks(raw, .8, 30)
    assert finite.tolist() == [True, True, True, True, False, False, True]
    assert low.tolist() == [True, True, False, False, False, False, True]
    assert high.tolist() == [False, False, False, True, False, False, False]
    assert valid.tolist() == [False, False, True, False, False, False, False]


@pytest.mark.parametrize('bounds', [(1, 1), (2, 1), (-1, 1), (np.nan, 30)])
def test_invalid_sensor_range_contract_is_rejected(bounds):
    """An invalid model cannot silently provide new surface evidence."""
    with pytest.raises(ValueError):
        point_range_masks(np.empty((0, 3)), *bounds)


def test_observed_grid_coverage_requires_every_intersecting_cell():
    """A tiny intersection with an unknown boundary cell still withholds support."""
    grid = np.zeros((20, 20), dtype=np.int8)
    full = assess(grid)
    assert full['envelope_observed'] and full['observed_floor_fraction'] == 1
    grid[10, 17] = -1  # x=[.7,.8]: circle intersects despite center x=.75 outside
    partial = assess(grid)
    assert not partial['envelope_observed'] and partial['cells']['unknown'] == 1
    assert partial['required_cells'] == sum(partial['cells'].values())


@pytest.mark.parametrize('healthy', [True, False])
def test_no_returns_never_establish_floor(healthy):
    """A plane estimate elsewhere does not observe missing cells under the robot."""
    result = assess(np.full((20, 20), -1), healthy=healthy)
    assert not result['envelope_observed'] and result['observed_floor_fraction'] == 0


def test_stale_complete_grid_and_map_edges_withhold_support():
    """Previously complete evidence must fail on loss or incomplete map coverage."""
    grid = np.zeros((20, 20))
    assert assess(grid, healthy=False)['reason'] == 'stale_or_invalid_map'
    edge = assess(grid, center=(0.8, 0.8))
    assert edge['cells']['outside_map'] > 0 and not edge['envelope_observed']


def test_occupied_cell_is_not_counted_as_observed_floor():
    """Obstacle evidence inside the body envelope prevents a complete floor label."""
    grid = np.zeros((20, 20))
    grid[10, 10] = 100
    result = assess(grid)
    assert result['cells']['occupied'] == 1 and not result['envelope_observed']


def dense_floor():
    """Place four non-boundary samples in each test cell on known flat floor."""
    y, x = np.indices((20, 20))
    return np.concatenate([np.column_stack((-1+(x.ravel()+dx)*.1,
                                            -1+(y.ravel()+dy)*.1, np.zeros(400)))
                           for dx, dy in ((.2, .2), (.2, .8), (.8, .2), (.8, .8))])


def test_body_plane_candidates_are_untrusted_and_cannot_seed_unknown_start():
    """Even dense feet-like plane points do not override the existing self mask."""
    result, grid, _ = scan_floor_coverage(dense_floor(), [0, 0, 0])
    assert result['untrusted_near_plane_points_inside_body_mask'] > 0
    assert grid[10, 10] == -1
    assert not result['single_scan_coverage']['envelope_observed']


def test_explicit_geometry_mode_reports_measured_floor_below_old_rectangle():
    """The nearest retained floor metric must respect the selected exclusion mode."""
    legacy, _, _ = scan_floor_coverage(dense_floor(), [0, 0, 0])
    geometry, _, _ = scan_floor_coverage(dense_floor(), [0, 0, 0], body_mask=False)
    assert legacy['nearest_trusted_floor_range_xy_m'] > .4
    assert geometry['nearest_trusted_floor_range_xy_m'] < .05


def test_hole_and_one_conflicting_return_remain_unknown_without_interpolation():
    """A fitted plane cannot bridge a missing patch or hide a step in its cell."""
    points = dense_floor()
    hole = (points[:, 0] > .5) & (points[:, 0] < .8) & (np.abs(points[:, 1]) < .2)
    points = np.r_[points[~hole], [[.55, .55, .06]]]
    _, grid, _ = scan_floor_coverage(points, [0, 0, 0])
    assert np.all(grid[8:12, 15:18] == -1)
    assert grid[15, 15] == -1
    assert grid[16, 15] == 0  # positive control outside the hole/conflict


def test_empty_and_nonfinite_cloud_do_not_create_support():
    """Missing rays provide neither floor counts nor a trusted nearest surface."""
    result, grid, counts = scan_floor_coverage([[np.nan]*3, [np.inf]*3], [0, 0, 0])
    assert np.all(grid == -1) and not counts.any()
    assert result['nearest_trusted_floor_range_xy_m'] is None


@pytest.mark.parametrize('plane', [[0, 0, np.nan], [0, 0]])
def test_invalid_plane_cannot_label_floor(plane):
    """Malformed or nonfinite plane metadata must be explicitly rejected."""
    with pytest.raises(ValueError):
        scan_floor_coverage(dense_floor(), plane)
