# SPDX-License-Identifier: Apache-2.0
"""Explicit simulation sensor profiles applied to generated URDF, never the dependency."""

import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def near_lidar_urdf(text):
    """Change only the L1 minimum range; reject a different pinned model layout."""
    root = ET.fromstring(text)
    sensors = root.findall("./gazebo[@reference='lidar_l1_link']/sensor[@name='lidar_l1']")
    if len(sensors) != 1 or sensors[0].get('type') != 'gpu_lidar':
        raise ValueError('expected_one_pinned_L1_gpu_lidar')
    ranges = sensors[0].findall('./lidar/range/min')
    if len(ranges) != 1 or float(ranges[0].text) != 0.8:
        raise ValueError('unexpected_pinned_L1_min_range')
    ranges[0].text = '0.05'
    return ET.tostring(root, encoding='unicode')


def prepare_near_model(context):
    """Generate one shared model before any upstream sensor or controller starts."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = value('sensor_profile')
    if profile == 'pinned':
        return []
    if profile != 'l1_near':
        raise ValueError('unknown_sensor_profile')
    args = ['xacro', value('unitree_go2_description_path')]
    names = {'robot_controllers': 'ros_control_file',
             **{name: name for name in ('command_interface', 'disable_camera', 'disable_d455',
                                        'disable_lidar_l1', 'disable_velodyne_lidar')}}
    args += [f'{target}:={value(source)}' for target, source in names.items()]
    result = subprocess.run(args, capture_output=True, text=True, timeout=15, check=True)
    generated = near_lidar_urdf(result.stdout)
    directory = Path(value('report_dir')).expanduser()
    path = directory / 'sensor-profile.urdf'
    path.write_text(generated)
    metadata = {'profile': profile, 'pinned_min_range_m': 0.8, 'min_range_m': 0.05,
                'other_sensor_settings_changed': False,
                'expanded_pinned_urdf_sha256': hashlib.sha256(result.stdout.encode()).hexdigest(),
                'generated_urdf_sha256': hashlib.sha256(generated.encode()).hexdigest(),
                'scope': 'simulation profile, not validated physical Air sensor calibration',
                'reference': 'https://www.unitree.com/mobile/LiDAR/'}
    (directory / 'sensor-profile.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return [SetLaunchConfiguration('unitree_go2_description_path', str(path))]
