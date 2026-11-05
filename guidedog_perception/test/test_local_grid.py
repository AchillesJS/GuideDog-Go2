# SPDX-License-Identifier: Apache-2.0
"""Check evidence, occlusion, ageing and world-cell preservation before simulation."""

from guidedog_perception.local_grid import LocalGrid
import numpy as np
import pytest


def points(x=1.05, y=0.05, z=0):
    """Provide independent returns in a single known world cell."""
    return np.array([[x, y, z], [x+0.01, y, z], [x, y+0.01, z]])


def value(grid, x=1.05, y=0.05, now=1):
    """Read a world position, independent of the current rolling origin."""
    col, row = np.floor(np.array([x, y])/grid.resolution).astype(int) - grid.origin_cells
    return grid.snapshot(now)['occupancy'][row, col]


def observe(grid, p, stamp=1, now=1, center=(0, 0), odom=None):
    """Use a known flat support plane for isolated evidence-policy tests."""
    return grid.observe(p, p if odom is None else odom, [0, 0, 0], center, stamp, now)


def test_only_observed_ground_is_free():
    """No ray or empty cloud can authorize the intervening or blind cells."""
    g = LocalGrid()
    observe(g, points())
    assert value(g) == 0
    assert value(g, x=0.65) == -1
    assert value(g, x=0.05) == -1
    observe(g, np.empty((0, 3)), 2, 1.1)
    assert value(g, x=0.65, now=1.1) == -1


def test_body_and_sparse_returns_cannot_establish_free_space():
    """Self returns and single ground samples remain unknown."""
    g = LocalGrid()
    observe(g, np.vstack((points(0.05), points(1.05)[:1])))
    assert value(g, x=0.05) == -1
    assert value(g) == -1


def test_geometry_filtered_floor_is_allowed_only_without_projected_excluded_hit():
    """Measured floor below the body can pass, but a removed hit withholds its cell."""
    g = LocalGrid()
    p = points(.05)
    g.observe(p, p, [0, 0, 0], [0, 0], 1, 1, body_mask=False)
    assert value(g, x=.05) == 0
    g.observe(p, p, [0, 0, 0], [0, 0], 2, 1.1, body_mask=False,
              excluded_odom=np.array([[.05, .05, .3]]))
    assert value(g, x=.05, now=1.1) == -1


def test_excluded_hit_never_clears_a_remembered_obstacle():
    """Repeated floor mixed with removed self/boundary points cannot clear occupancy."""
    g = LocalGrid()
    observe(g, points(z=.2))
    for stamp in range(2, 7):
        p = points()
        g.observe(p, p, [0, 0, 0], [0, 0], stamp, 1+stamp*.1,
                  excluded_odom=np.array([[1.05, .05, .3]]))
    assert value(g, now=1.7) == 100


def test_obstacle_dominates_ground_in_same_cell():
    """Visible ground in a mixed cell must not erase its obstacle."""
    g = LocalGrid()
    observe(g, np.vstack((points(), points(z=0.2))))
    assert value(g) == 100


@pytest.mark.parametrize('height', [0.2, 0.04, -0.08, 2.0])
def test_ambiguous_height_invalidates_a_previously_free_cell(height):
    """One nonfloor return cannot coexist with a claim of clear observed floor."""
    g = LocalGrid()
    observe(g, points())
    observe(g, np.vstack((points(), points(z=height)[:1])), 2, 1.1)
    assert value(g, now=1.1) == -1


def test_occlusion_holds_then_expires_to_unknown():
    """A missing observation cannot mark a remembered obstacle free."""
    g = LocalGrid()
    observe(g, points(z=0.2))
    observe(g, np.empty((0, 3)), 2, 2)
    assert value(g, now=2) == 100
    assert value(g, now=6.01) == -1


def test_three_fresh_ground_scans_clear_a_remembered_obstacle():
    """One floor sample or a replay cannot clear an occupied cell."""
    g = LocalGrid()
    observe(g, points(z=0.2))
    observe(g, points(), 2, 1.1)
    assert value(g, now=1.1) == 100
    assert not observe(g, points(), 2, 1.2)
    assert value(g, now=1.2) == 100
    observe(g, points(), 3, 1.3)
    assert value(g, now=1.3) == 100
    observe(g, points(), 4, 1.4)
    assert value(g, now=1.4) == 0


def test_interrupted_clearing_requires_a_new_streak():
    """An occluded scan interrupts consecutive ground evidence."""
    g = LocalGrid()
    observe(g, points(z=0.2))
    observe(g, points(), 2, 1.1)
    observe(g, np.empty((0, 3)), 3, 1.2)
    observe(g, points(), 4, 1.3)
    observe(g, points(), 5, 1.4)
    assert value(g, now=1.4) == 100


def test_ground_expires_without_new_data_even_when_source_clock_is_frozen():
    """Expiry uses wall time and cannot be renewed by duplicate stamps."""
    g = LocalGrid()
    observe(g, points())
    assert not observe(g, points(), 1, 2)
    assert value(g, now=3.01) == -1


@pytest.mark.parametrize('center', [(0.2, 0), (-0.2, 0), (0, 0.2), (0, -0.2)])
def test_roll_preserves_world_cells_without_wraparound(center):
    """Translation changes storage indices, not an obstacle's odom position."""
    g = LocalGrid()
    observe(g, np.vstack((points(z=0.2), points(3.95, z=0.2))))
    g.recenter(center)
    assert value(g) == 100
    assert g.rolls == 1
    assert value(g, x=-3.7) == -1


def test_large_localization_jump_discards_history():
    """Do not attach old obstacles to a discontinuous localization frame."""
    g = LocalGrid()
    observe(g, points(z=0.2))
    g.recenter([2, 0])
    assert value(g) == -1
    assert g.resets == 1


def test_height_and_age_layers_are_bounded_observations():
    """Ground height and scatter have units and become unknown after expiry."""
    g = LocalGrid()
    p = points()
    p[:, 2] = [-0.01, 0, 0.01]
    world = p + [0, 0, 0.3]
    observe(g, p, odom=world)
    snap = g.snapshot(1.5)
    row, col = np.argwhere(snap['occupancy'] == 0)[0]
    assert snap['ground_z_odom_m'][row, col] == pytest.approx(0.3)
    assert snap['ground_rms_m'][row, col] == pytest.approx(np.std(p[:, 2]))
    assert snap['age_sec'][row, col] == pytest.approx(0.5)
    assert np.isnan(g.snapshot(4)['ground_z_odom_m']).all()
    assert g.memory_bytes < 1024*1024


def test_reset_clears_stamp_and_all_layers():
    """A new source-clock epoch cannot reuse earlier evidence."""
    g = LocalGrid()
    observe(g, points(z=0.2), 100)
    g.reset()
    assert observe(g, points(), 1, 2)
    assert not (g.snapshot(2)['occupancy'] == 100).any()
