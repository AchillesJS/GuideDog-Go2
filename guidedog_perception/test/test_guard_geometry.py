# SPDX-License-Identifier: Apache-2.0
"""Check cloud layout, geometry, conservative unknown handling and motion veto."""

from types import SimpleNamespace

from guidedog_perception.guard_geometry import (
    cloud_xyz, inspect_corridor, MotionGate, transform_xyz)
from guidedog_perception.support_evidence import point_range_masks
import numpy as np
import pytest


def scene(obstacle=None):
    """Build observed flat ground and an optional elevated return."""
    ground = np.array([[x, y, 0] for x in np.linspace(0.8, 3, 20)
                       for y in np.linspace(-0.3, 0.3, 5)])
    return np.vstack([ground, [obstacle]]) if obstacle is not None else ground


def test_forward_obstacle_floor_and_side_filtering():
    """Ignore floor, rear/body returns and obstacles outside the swept corridor."""
    assert inspect_corridor(scene([1.0, 0.0, 0.3]))['clearance_m'] == pytest.approx(0.55)
    for point in ([1, 1.0, 0.3], [-1, 0, 0.3], [0.2, 0, 0.3], [1, 0, 1.5]):
        result = inspect_corridor(scene(point))
        assert result['clearance_m'] is None
        assert result['ground_observed']


def test_empty_nan_and_unobserved_ground_are_not_free():
    """Missing data or ground coverage must never become a clear path."""
    with pytest.raises(ValueError):
        inspect_corridor(np.full((100, 3), np.nan))
    result = inspect_corridor(np.tile([2.0, 2.0, 0], (50, 1)))
    assert not result['ground_observed']


def test_excluded_support_cannot_fit_floor_but_remains_an_obstacle_candidate():
    """Removing positive evidence must never remove the same hit from raw obstacle veto."""
    points = scene([1, 0, .3])
    result = inspect_corridor(points, support_valid=np.zeros(len(points), dtype=bool))
    assert not result['ground_observed'] and result['ground_plane_z_ax_by_c'] is None
    assert result['clearance_m'] == pytest.approx(.55)


def test_saturated_near_hit_is_retained_when_other_floor_is_valid():
    """A min-range obstacle must survive exclusion from positive floor support."""
    points = scene([.75, 0, np.sqrt(.8**2-.75**2)])
    _, low, _, valid = point_range_masks(points, .8, 30)
    assert low[-1] and not valid[-1]
    result = inspect_corridor(points, support_valid=valid)
    assert result['ground_observed'] and result['clearance_m'] == pytest.approx(.30)


def test_ground_plane_handles_footprint_height_error_and_retains_obstacle():
    """Use current observed support rather than assuming an exact footprint plane."""
    points = scene([1.0, 0.0, 0.3])
    points[:, 2] += -0.09 + 0.02*points[:, 0] - 0.01*points[:, 1]
    result = inspect_corridor(points)
    assert result['ground_observed']
    assert result['clearance_m'] == pytest.approx(0.55)
    np.testing.assert_allclose(result['ground_plane_z_ax_by_c'], [0.02, -0.01, -0.09])


@pytest.mark.parametrize('offset,slope', [(0.10, 0), (-0.30, 0), (0, 0.30)])
def test_unbounded_or_elevated_plane_is_not_ground(offset, slope):
    """Reject excessive pose error, elevated surfaces and steep observed support."""
    points = scene()
    points[:, 2] += offset + slope*points[:, 0]
    assert not inspect_corridor(points)['ground_observed']


def test_ground_support_requires_spatial_extent():
    """Repeated points or a line cannot establish a support plane."""
    points = np.array([[x, 0, 0] for x in np.linspace(0.8, 3, 50)])
    assert not inspect_corridor(points)['ground_observed']


def test_side_context_fits_plane_when_forward_patch_is_short():
    """A blind zone and box may leave a short but observed forward floor patch."""
    front = np.array([[x, y, 0] for x in np.linspace(0.9, 1.1, 10)
                      for y in (-0.3, 0, 0.3)])
    side = np.array([[x, y, 0] for x in np.linspace(0.8, 3, 20)
                     for y in (-0.8, 0.8)])
    result = inspect_corridor(np.vstack((front, side, [[1.25, 0, 0.3]])))
    assert result['ground_observed']
    assert result['clearance_m'] == pytest.approx(0.8)
    # Side ground alone must not establish forward traversability.
    assert not inspect_corridor(side)['ground_observed']


def test_side_ground_does_not_relabel_an_elevated_forward_patch():
    """Nearby support must retain the height of a small forward obstacle."""
    side = np.array([[x, y, 0] for x in np.linspace(0.8, 3, 20)
                     for y in (-0.8, 0.8)])
    raised = np.array([[x, y, 0.1] for x in np.linspace(0.9, 1.1, 10)
                       for y in (-0.3, 0, 0.3)])
    result = inspect_corridor(np.vstack((side, raised)))
    assert not result['ground_observed']
    assert result['clearance_m'] == pytest.approx(0.45)


def test_transform_and_invalid_quaternion():
    """Use the rigid frame transform before applying floor-height thresholds."""
    points = np.array([[1, 0, 0]])
    transformed = transform_xyz(points, [0, 0, 0.4], [0, 0, np.sqrt(0.5), np.sqrt(0.5)])
    np.testing.assert_allclose(transformed, [[0, 1, 0.4]], atol=1e-6)
    with pytest.raises(ValueError):
        transform_xyz(points, [0, 0, 0], [0, 0, 0, 0])


@pytest.mark.parametrize('big_endian', [False, True])
def test_cloud_endian_and_row_padding(big_endian):
    """Decode organized clouds without interpreting padding as extra points."""
    order = '>' if big_endian else '<'
    raw = bytearray(2 * 32)
    view = np.ndarray((2, 2, 3), dtype=order + 'f4', buffer=raw, strides=(32, 12, 4))
    view[:] = np.arange(12).reshape(2, 2, 3)
    fields = [SimpleNamespace(name=name, offset=i*4, datatype=7, count=1)
              for i, name in enumerate(('x', 'y', 'z'))]
    msg = SimpleNamespace(width=2, height=2, point_step=12, row_step=32,
                          is_bigendian=big_endian, fields=fields, data=raw)
    np.testing.assert_allclose(cloud_xyz(msg), np.arange(12).reshape(4, 3))
    msg.data = raw[:10]
    with pytest.raises(ValueError):
        cloud_xyz(msg)


def ready_gate():
    """Initialize a clear observed path and require an explicit neutral request."""
    gate = MotionGate()
    gate.observe(inspect_corridor(scene()), 1.0, 100)
    gate.request([0, 0, 0, 0, 0, 0], 1.01)
    assert gate.decide(1.02, stamp_age=0.02) == (0, 'idle')
    return gate


def test_cap_stop_latch_and_explicit_rearm():
    """An obstacle stop cannot automatically resume a held forward request."""
    gate = ready_gate()
    gate.request([2, 0, 0, 0, 0, 0], 1.03)
    assert gate.decide(1.04, stamp_age=0.04) == (0.1, 'moving')
    gate.observe(inspect_corridor(scene([1, 0, 0.3])), 1.05, 110)
    assert gate.decide(1.06, stamp_age=0.01) == (0, 'obstacle')
    gate.observe(inspect_corridor(scene()), 1.07, 120)
    gate.request([0.1, 0, 0, 0, 0, 0], 1.08)
    assert gate.decide(1.09, stamp_age=0.02) == (0, 'neutral_command_required')
    gate.request([0, 0, 0, 0, 0, 0], 1.10)
    assert gate.decide(1.11, stamp_age=0.04) == (0, 'idle')


@pytest.mark.parametrize('components', [
    [-0.1, 0, 0, 0, 0, 0], [0, 0.1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0.1], [float('nan'), 0, 0, 0, 0, 0]])
def test_unsupported_commands_fail_closed(components):
    """Reverse, turning, sideways and nonfinite input must remain zero."""
    gate = ready_gate()
    gate.request(components, 1.03)
    assert gate.decide(1.04, stamp_age=0.04)[0] == 0


def test_stale_replayed_data_and_clock_failure():
    """Repeated stamps, frozen clocks and expired scans must veto motion."""
    gate = ready_gate()
    gate.request([0.1, 0, 0, 0, 0, 0], 1.1)
    assert gate.decide(1.2, stamp_age=0.2, clock_ok=False) == (0, 'clock_stalled')
    gate = ready_gate()
    assert gate.decide(2, stamp_age=0) == (0, 'stale_scan')
    gate = ready_gate()
    gate.observe(inspect_corridor(scene()), 1.2, 100)
    assert gate.decide(1.3, stamp_age=0.1)[0] == 0
    assert gate.fault == 'nonadvancing_scan_stamp'


def test_command_timeout_and_unknown_ground():
    """Keep data absence distinct from an obstacle, and expire old requests."""
    gate = ready_gate()
    assert gate.decide(1.5, stamp_age=0.5) == (0, 'command_timeout')
    gate.observe(inspect_corridor(np.tile([2, 2, 0], (40, 1))), 1.6, 200)
    assert gate.decide(1.61, stamp_age=0.01) == (0, 'unknown_forward_ground')


def test_vm_profile_preserves_commanded_stale_travel_budget():
    """A longer wall timeout must be coupled to a lower command speed."""
    with pytest.raises(ValueError):
        MotionGate(max_speed=0.1, scan_timeout=1.5)
    gate = MotionGate(max_speed=0.05, scan_timeout=1.5)
    assert gate.max_speed * gate.scan_timeout <= 0.08
    gate.observe(inspect_corridor(scene()), 1.0, 100)
    gate.request([0, 0, 0, 0, 0, 0], 1.01)
    assert gate.decide(1.02, stamp_age=0.02) == (0, 'idle')
    gate.request([0.1, 0, 0, 0, 0, 0], 2.3)
    assert gate.decide(2.4, stamp_age=1.4) == (0.05, 'moving')
    assert gate.decide(2.51, stamp_age=1.51) == (0, 'stale_scan')


def test_demo_rejects_unvalidated_slow_requests_and_requires_neutral():
    """Do not drive at a lower request where the simulated gait has drifted away."""
    gate = MotionGate(max_speed=0.05, min_speed=0.05, scan_timeout=1.5)
    gate.observe(inspect_corridor(scene()), 1, 100)
    gate.request([0, 0, 0, 0, 0, 0], 1.01)
    assert gate.decide(1.02, stamp_age=0.02) == (0, 'idle')
    gate.request([0.03, 0, 0, 0, 0, 0], 1.03)
    assert gate.decide(1.04, stamp_age=0.04) == (0, 'unsupported_or_invalid_command')
    gate.observe(inspect_corridor(scene()), 1.05, 200)
    gate.request([0.05, 0, 0, 0, 0, 0], 1.06)
    assert gate.decide(1.07, stamp_age=0.02) == (0, 'neutral_command_required')
    gate.request([0, 0, 0, 0, 0, 0], 1.08)
    assert gate.decide(1.09, stamp_age=0.04) == (0, 'idle')
    gate.request([0.05, 0, 0, 0, 0, 0], 1.1)
    assert gate.decide(1.11, stamp_age=0.06) == (0.05, 'moving')
