# SPDX-License-Identifier: Apache-2.0
"""Reject reordered content/status pairs and stale observations at the consumer."""

from collections import deque
import threading
import time
from types import SimpleNamespace

from guidedog_perception.map_analysis import map_digest
from guidedog_perception.map_benchmark import MappingDriver, wait_mapping
from nav_msgs.msg import OccupancyGrid
import numpy as np


def fixture():
    """Build actual ROS grids whose cells change while their source stamp stays fixed."""
    messages = []
    for values in ([0, 0, 100, -1], [-1, -1, -1, -1]):
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp.sec = 'odom', 1
        msg.info.width = msg.info.height = 2
        msg.info.resolution = 0.1
        msg.data = values
        messages.append(msg)
    status = {'parameters': {'map_frame': 'odom'}, 'source_stamp_ns': 1000000000,
              'healthy': True, 'source_receipt_age_wall_sec': 0.1,
              'observer_received_monotonic': time.monotonic(),
              'snapshot_digest': map_digest(np.array(messages[0].data).reshape(2, 2),
                                            [0, 0], messages[0].info.resolution, 1000000000)}
    return SimpleNamespace(lock=threading.Lock(), maps=deque(messages), map_status=status)


def test_same_stamp_cannot_pair_different_cell_contents():
    """Late status for an occupied map must not describe the newer expired cells."""
    driver = fixture()
    paired = MappingDriver.mapping(driver)
    assert paired is not None and paired[0][1, 0] == 100
    driver.maps.popleft()
    assert MappingDriver.mapping(driver) is None


def test_wrong_frame_cannot_pair_by_stamp_and_digest():
    """A matching stamp and byte digest are insufficient across coordinate frames."""
    driver = fixture()
    driver.maps[0].header.frame_id = 'other_odom'
    assert MappingDriver.mapping(driver) is None


def test_consumer_elapsed_time_expires_publisher_health():
    """A previously healthy observation cannot remain usable in the consumer cache."""
    driver = fixture()
    driver.map_status['observer_received_monotonic'] = time.monotonic()-0.6
    paired = MappingDriver.mapping(driver)
    assert paired is not None and paired[3]['reported_healthy']
    assert not paired[3]['healthy']


def test_wait_observer_uses_one_copy_for_each_predicate():
    """An unmatched callback interval must be retried without refetching its state."""
    expected = {'healthy': True}
    copies = iter([None, {'healthy': False}, expected])
    driver = SimpleNamespace(mapping=lambda: next(copies))
    actual = wait_mapping(driver, lambda s: s['healthy'], 1, 'not available')
    assert actual is expected
