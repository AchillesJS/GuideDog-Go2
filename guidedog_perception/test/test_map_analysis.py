# SPDX-License-Identifier: Apache-2.0
"""Verify observer pairing and conservative free-space assessment on known geometry."""

import math

from guidedog_perception.map_analysis import (
    assess_boxes, assess_connectivity, cell_centers, flood, footprint_free,
    largest_component, map_digest, rectangle_signed_distance)
import numpy as np
import pytest


def test_content_digest_changes_on_expiry_even_with_unchanged_source_stamp():
    """A status for an expired grid must not pair with its old occupied snapshot."""
    grid = np.full((8, 8), -1, dtype=np.int8)
    before = map_digest(grid, [-1, -2], 0.1, 100)
    grid[2, 3] = 100
    assert map_digest(grid, [-1, -2], 0.1, 100) != before
    assert map_digest(grid, [-1, -2], float(np.float32(0.1)), 100) == map_digest(
        grid, [-1, -2], 0.1, 100)
    assert map_digest(grid, [-1, -2], 0.1, 101) != map_digest(grid, [-1, -2], 0.1, 100)
    assert map_digest(grid, [-1, -2], 0.1, 100) != map_digest(grid, [0, -2], 0.1, 100)


@pytest.mark.parametrize('data', [np.array([[50]]), np.array([]), np.zeros((201, 2))])
def test_invalid_grid_is_rejected(data):
    """Malformed classes or dimensions cannot silently become free space."""
    with pytest.raises(ValueError):
        map_digest(data, [0, 0], 0.1, 100)


def test_diagonal_cells_cannot_connect_through_blocked_corners():
    """A point's diagonal shortcut must not stand in for a traversable corridor."""
    grid = np.eye(5, dtype=bool)
    assert flood(grid, (0, 0)).sum() == 1
    assert largest_component(grid) == 1


def test_unknown_and_map_edges_block_the_complete_envelope():
    """The whole footprint must have observed support, including its boundary."""
    data = np.zeros((20, 20), dtype=np.int8)
    allowed = footprint_free(data, 0.1, 0.25)
    assert allowed[10, 10]
    assert not allowed[0, 0]
    data[10, 12] = -1
    assert not footprint_free(data, 0.1, 0.25)[10, 10]


def test_narrow_passage_is_point_connected_but_not_footprint_connected():
    """A visible gap narrower than the diagnostic body envelope is unavailable."""
    data = np.full((40, 40), 100, dtype=np.int8)
    data[17:23, :] = 0
    assert flood(data == 0, (20, 5))[20, 35]
    allowed = footprint_free(data, 0.1, 0.4)
    assert not allowed.any()


def test_sufficiently_observed_wide_passage_is_connected():
    """The assessor must distinguish an observed wide passage from a narrow one."""
    data = np.full((40, 40), 100, dtype=np.int8)
    data[8:32, :] = 0
    result = assess_connectivity(data, [0, 0], 0.1, [1, 2], [3, 2], 0.4)
    assert result['planning_ready'] and result['reason'] == 'observed_connection'


def test_blind_start_is_not_seeded_to_free():
    """A known passage beyond the sensor blind region does not validate the start."""
    data = np.zeros((40, 40), dtype=np.int8)
    data[19:22, 9:12] = -1
    result = assess_connectivity(data, [0, 0], 0.1, [1.05, 2.05], [3, 2], 0.4)
    assert not result['planning_ready'] and result['reason'] == 'start_unknown'


def test_stale_map_is_never_ready_despite_connected_cells():
    """Connectivity is not permission to use old observations."""
    result = assess_connectivity(np.zeros((40, 40)), [0, 0], 0.1,
                                 [1, 2], [3, 2], 0.4, healthy=False)
    assert not result['planning_ready'] and result['reason'] == 'stale_or_invalid_map'


def test_rectangle_distance_uses_rotation_and_detects_interior():
    """World rotations must preserve signed distances to the known rectangle."""
    p = np.array([[0, 0], [1, 0], [0.25, 0.5], [0.5, 0.75]])
    expected = [-0.25, 0.75, 0, math.sqrt(0.25**2+0.25**2)]
    np.testing.assert_allclose(rectangle_signed_distance(p, [0, 0, 0], [0.5, 1]), expected)
    world = np.stack((-p[:, 1]+10, p[:, 0]+20), axis=1)
    np.testing.assert_allclose(
        rectangle_signed_distance(world, [10, 20, math.pi/2], [0.5, 1]), expected, atol=1e-8)


def test_false_free_inside_known_object_is_measured_without_counting_edge_cells():
    """Report confidently interior free labels separately from grid-edge ambiguity."""
    data = np.full((40, 40), -1, dtype=np.int8)
    centres = cell_centers(data, [-2, -2], 0.1)
    distance = rectangle_signed_distance(centres, [0, 0, 0], [1, 1])
    data[distance < -0.2] = 0
    data[np.abs(distance) < 0.06] = 100
    result = assess_boxes(data, [-2, -2], 0.1,
                          [{'name': 'box', 'pose': [0, 0, 0], 'size': [1, 1, 1]}])
    assert result['free_wholly_inside_boxes_cells'] > 0
    assert result['boxes']['box']['associated_occupied_cells'] >= 3
    assert result['boxes']['box']['boundary_error_p95_m'] < 0.06


def test_large_mislocalization_is_not_hidden_by_good_associated_cells():
    """Report occupied returns outside all association gates instead of dropping them."""
    data = np.full((40, 40), -1, dtype=np.int8)
    data[35, 35] = 100
    result = assess_boxes(data, [-2, -2], 0.1,
                          [{'name': 'box', 'pose': [0, 0, 0], 'size': [0.5, 1, 1]}])
    assert result['unmatched_occupied_cells'] == 1
    assert result['boxes']['box']['associated_occupied_cells'] == 0
