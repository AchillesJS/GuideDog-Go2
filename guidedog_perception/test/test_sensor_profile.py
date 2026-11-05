# SPDX-License-Identifier: Apache-2.0
"""Keep the native sensor-profile change explicit and semantically restricted."""

from copy import deepcopy
import xml.etree.ElementTree as ET

from guidedog_perception.sensor_profile import near_lidar_urdf
import pytest


MODEL = ('<robot name="fixture"><link name="body"/><gazebo reference="lidar_l1_link">'
         '<sensor name="lidar_l1" type="gpu_lidar"><update_rate>10</update_rate><lidar>'
         '<scan><vertical><samples>30</samples></vertical></scan>'
         '<range><min>0.8</min><max>30</max></range></lidar></sensor></gazebo></robot>')


def test_near_profile_changes_exactly_one_range_value():
    """Physics, scan pattern, frame, geometry and update rate must remain identical."""
    expected = ET.fromstring(MODEL)
    actual = ET.fromstring(near_lidar_urdf(MODEL))
    expected.find('./gazebo/sensor/lidar/range/min').text = '0.05'
    assert ET.tostring(actual) == ET.tostring(expected)


@pytest.mark.parametrize('kind', ['missing', 'duplicate', 'changed_range'])
def test_changed_upstream_sensor_layout_is_rejected(kind):
    """Do not patch a guessed sensor or silently accept an unrelated model change."""
    root = ET.fromstring(MODEL)
    if kind == 'missing':
        root.remove(root.find('gazebo'))
    elif kind == 'duplicate':
        root.append(deepcopy(root.find('gazebo')))
    else:
        root.find('./gazebo/sensor/lidar/range/min').text = '0.4'
    with pytest.raises(ValueError):
        near_lidar_urdf(ET.tostring(root, encoding='unicode'))
