# SPDX-License-Identifier: Apache-2.0
"""Compose explicit rendering/gait profiles around the pinned upstream description."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.gait_profile import apply_profile
from guidedog_perception.headless_launch import server_only
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.substitutions import LaunchConfiguration


def prepare(context):
    """Select the declared profile while retaining all unrelated upstream actions."""
    path = (Path(get_package_share_directory('unitree_go2_sim'))
            / 'launch/unitree_go2_launch.py')
    description = get_launch_description_from_python_launch_file(str(path))
    description = apply_profile(description, LaunchConfiguration('gait_profile').perform(context))
    if LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes'):
        description = server_only(description)
    return list(description.entities)


def generate_launch_description():
    """Defer selection until the including launch has supplied its profile values."""
    return LaunchDescription([OpaqueFunction(function=prepare)])
