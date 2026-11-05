# SPDX-License-Identifier: Apache-2.0
"""Guard the scope of an explicit simulator gait-profile override."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.gait_profile import apply_profile
from launch import LaunchDescription
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
import pytest


def test_stock_keeps_the_original_description():
    """Default launches must keep the upstream controller action untouched."""
    description = LaunchDescription([])
    assert apply_profile(description, 'stock') is description


@pytest.mark.parametrize('count', [0, 2])
def test_gait_profile_rejects_ambiguous_controller_layout(count):
    """Reject a missing or duplicated controller instead of changing a guessed node."""
    description = LaunchDescription([
        Node(package='champ_base', executable='quadruped_controller_node')
        for _ in range(count)])
    with pytest.raises(RuntimeError, match='exactly one'):
        apply_profile(description, 'low_lift')


def test_only_the_pinned_gait_generator_is_replaced():
    """Keep physics, effort-controller spawning, sensing and estimation actions intact."""
    path = Path(get_package_share_directory('unitree_go2_sim')) / 'launch/unitree_go2_launch.py'
    original = get_launch_description_from_python_launch_file(str(path))
    result = apply_profile(original, 'low_lift')
    assert len(result.entities) == len(original.entities)
    changed = [(a, b) for a, b in zip(original.entities, result.entities) if a is not b]
    assert len(changed) == 1
    a, b = changed[0]
    assert isinstance(a, Node) and isinstance(b, Node)


def test_unknown_profile_fails_before_launch():
    """Never silently substitute stock behaviour for an unsupported configuration."""
    with pytest.raises(ValueError, match='unknown_gait_profile'):
        apply_profile(LaunchDescription([]), 'typo')


def test_calibrated_profile_keeps_sensors_and_ekf_actions():
    """Only construction of the gait generator and raw odometry estimator changes."""
    path = Path(get_package_share_directory('unitree_go2_sim')) / 'launch/unitree_go2_launch.py'
    original = get_launch_description_from_python_launch_file(str(path))
    result = apply_profile(original, 'low_lift_calibrated')
    changed = [(a, b) for a, b in zip(original.entities, result.entities) if a is not b]
    assert len(result.entities) == len(original.entities) and len(changed) == 2
