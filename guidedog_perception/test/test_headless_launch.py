# SPDX-License-Identifier: Apache-2.0
"""Verify that server rendering changes no simulator sensors or controller actions."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.headless_launch import server_only
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file, PythonLaunchDescriptionSource)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


@pytest.mark.parametrize('count', [0, 2])
def test_ambiguous_gazebo_layout_is_rejected(count):
    """An upstream layout change must fail instead of guessing which process to edit."""
    entities = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/unused.py'),
        launch_arguments={'gz_args': 'world -r'}.items())
                for _ in range(count)]
    with pytest.raises(RuntimeError, match='exactly one'):
        server_only(LaunchDescription(entities))


def test_pinned_simulator_keeps_all_other_actions():
    """Exercise the installed upstream description and actual Launch substitutions."""
    path = Path(get_package_share_directory('unitree_go2_sim')) / 'launch/unitree_go2_launch.py'
    original = get_launch_description_from_python_launch_file(str(path))
    sentinel = LogInfo(msg='retained')
    original.add_action(sentinel)
    adapted = server_only(original)
    pairs = list(zip(original.entities, adapted.entities))
    changed = [(a, b) for a, b in pairs if a is not b]
    assert len(adapted.entities) == len(original.entities) and len(changed) == 1
    before, after = changed[0]
    assert before.launch_description_source is after.launch_description_source
    assert original.entities[-1] is adapted.entities[-1] is sentinel
    context = LaunchContext()
    context.launch_configurations['world'] = '/tmp/test world.sdf'
    before_args = perform_substitutions(
        context, normalize_to_list_of_substitutions(dict(before.launch_arguments)['gz_args']))
    after_args = perform_substitutions(context, dict(after.launch_arguments)['gz_args'])
    assert after_args == before_args + ' -s --headless-rendering'
    assert '-s' not in before_args and '--headless-rendering' not in before_args
