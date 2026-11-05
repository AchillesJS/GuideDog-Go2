# SPDX-License-Identifier: Apache-2.0
"""Load the pinned upstream launch unchanged and select server-only EGL rendering."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.headless_launch import server_only
from launch.launch_description_sources import get_launch_description_from_python_launch_file


def generate_launch_description():
    """Keep controllers, robot description, bridges and sensors from upstream."""
    path = (Path(get_package_share_directory('unitree_go2_sim'))
            / 'launch/unitree_go2_launch.py')
    return server_only(get_launch_description_from_python_launch_file(str(path)))
