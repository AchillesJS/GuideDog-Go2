# SPDX-License-Identifier: Apache-2.0
"""Check independent reference geometry, timing and heading conventions."""

import math

from guidedog_perception.clearance_metrics import box_clearance, level_points, PoseHistory, yaw_of
import numpy as np
import pytest


def test_reference_pose_requires_a_time_bracket():
    """Never extrapolate or pair across a large source-time gap."""
    h = PoseHistory()
    h.add(100000000, [0, 0, 0.3], [0, 0, 0, 1])
    h.add(120000000, [0.02, 0, 0.3], [0, 0, 0, 1])
    assert h.at(90000000) is None
    assert h.at(130000000) is None
    p, _, gap = h.at(110000000)
    np.testing.assert_allclose(p, [0.01, 0, 0.3])
    assert gap == pytest.approx(0.02)
    h.add(300000000, [0.2, 0, 0.3], [0, 0, 0, 1])
    assert h.at(200000000) is None
    h.add(1000000, [0, 0, 0.3], [0, 0, 0, 1])
    assert h.at(110000000) is None


def test_reference_rotation_takes_short_arc_across_heading_wrap():
    """An interpolated heading must remain near 180 degrees, not jump to zero."""
    h = PoseHistory()
    for stamp, angle in [(100000000, 170), (120000000, -170)]:
        angle = math.radians(angle)
        h.add(stamp, [0, 0, 0.3], [0, 0, math.sin(angle/2), math.cos(angle/2)])
    _, q, _ = h.at(110000000)
    assert abs(yaw_of(q)) == pytest.approx(math.pi)


def test_reference_cloud_removes_heading_but_retains_body_pitch():
    """Use full body attitude to recover the actual height and horizontal distance."""
    pitch = math.pi/6
    q = [0, math.sin(pitch/2), 0, math.cos(pitch/2)]
    points = level_points(np.array([[1.0, 0, 0]]), [5, 4, 0.5], q)
    np.testing.assert_allclose(points, [[math.cos(pitch), 0, 0]], atol=1e-8)


def test_box_corridor_ignores_side_obstacle_and_checks_front_face():
    """A longitudinal projection of an off-path box is not the corridor clearance."""
    gap, projected = box_clearance([2, 0, 0], [0.5, 1, 1], [0, 0, 0], 0)
    assert gap == pytest.approx(1.3)
    assert projected == pytest.approx(1.3)
    gap, projected = box_clearance([2, 2, 0], [0.5, 1, 1], [0, 0, 0], 0)
    assert gap is None
    assert projected == pytest.approx(1.3)


def test_rotated_box_corner_outside_strip_does_not_shorten_corridor():
    """Clip to the swept strip rather than projecting the entire rotated box."""
    gap, projected = box_clearance([2, 0, math.pi/4], [0.5, 2, 1], [0, 0, 0], 0)
    assert gap > projected + 0.1
    # Intersection with y=0.4 gives x=2-0.4-sqrt(0.125).
    assert gap == pytest.approx(2 - 0.4 - math.sqrt(0.125) - 0.45)


def test_reference_geometry_is_invariant_to_world_translation_and_heading():
    """Known world placement and robot heading must describe the same local scene."""
    expected = box_clearance([2, 0, 0], [0.5, 1, 1], [0, 0, 0], 0)
    observed = box_clearance([10, 22, math.pi/2], [0.5, 1, 1], [10, 20, 0], math.pi/2)
    np.testing.assert_allclose(observed, expected)
