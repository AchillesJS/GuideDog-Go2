# SPDX-License-Identifier: Apache-2.0
"""Compare native and ROS LiDAR stamps in shared wall-time observation windows."""

import json
import os
import signal
import subprocess
import threading
import time

from guidedog_perception.stop_benchmark import prepare_scenarios
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.transport13 import Node as GzNode
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class StampObserver:
    """Store header stamps and receipt times only, never cloud payloads."""

    def __init__(self, node):
        """Listen independently on native, existing bridge and experimental bridge paths."""
        self.lock = threading.Lock()
        self.samples = {'native': [], 'original': [], 'reliable': [], 'oneway': []}
        self.gz = GzNode()
        self.gz.subscribe(PointCloudPacked, '/unitree_lidar/points',
                          lambda msg: self.add('native', msg.header.stamp))
        self.subscriptions = [node.create_subscription(
            PointCloud2, topic, lambda msg, key=key: self.add(key, msg.header.stamp),
            qos_profile_sensor_data) for key, topic in (
                ('original', '/unitree_lidar/points'), ('oneway', '/guidedog/lidar_oneway'))]
        self.subscriptions.append(node.create_subscription(
            PointCloud2, '/unitree_lidar/points',
            lambda msg: self.add('reliable', msg.header.stamp),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)))

    def add(self, key, stamp):
        """Capture generation stamps at the actual callback boundary."""
        with self.lock:
            self.samples[key].append((time.monotonic(), stamp.sec*1000000000 + stamp.nsec
                                      if key == 'native'
                                      else stamp.sec*1000000000 + stamp.nanosec))

    def window(self, seconds, channels):
        """Measure identical receipt windows and compare only shared source-time ranges."""
        started = time.monotonic()
        time.sleep(seconds)
        ended = time.monotonic()
        with self.lock:
            rows = {key: [r for r in self.samples[key] if started <= r[0] < ended]
                    for key in channels}
        result = {'window_wall_sec': ended - started, 'channels': {}}
        for key, values in rows.items():
            unique = sorted({stamp for _, stamp in values})
            result['channels'][key] = {
                'received': len(values), 'unique_stamps': len(unique),
                'duplicate_stamps': len(values) - len(unique),
                'received_hz_wall': len(values)/(ended - started),
                'unique_hz_wall': len(unique)/(ended - started),
                'max_gap_wall_sec': max([0.0] + [b[0] - a[0] for a, b in zip(values, values[1:])]),
            }
        if all(rows.values()):
            low = max(min(stamp for _, stamp in values) for values in rows.values())
            high = min(max(stamp for _, stamp in values) for values in rows.values())
            sets = {key: {stamp for _, stamp in values if low <= stamp <= high}
                    for key, values in rows.items()}
            native = sets['native']
            result['shared_stamp_range_ns'] = [low, high]
            result['native_unique_in_shared_range'] = len(native)
            result['matched_native_stamp_fraction'] = {
                key: len(stamps & native)/len(native) if native else None
                for key, stamps in sets.items() if key != 'native'}
        result['passed'] = all(len(values) >= 20 for values in rows.values())
        return result


def run_profile(driver, record, directory):
    """Compare the original bridge with an added one-way bridge without moving."""
    prepare_scenarios(driver, record)
    observer = StampObserver(driver)
    time.sleep(2.0)
    record('original_bridge_window', observer.window(20, ('native', 'original', 'reliable')))
    log = (directory / 'oneway-bridge.log').open('w')
    bridge = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/unitree_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '--ros-args', '-r', '__node:=guidedog_lidar_oneway_probe',
        '-r', '/unitree_lidar/points:=/guidedog/lidar_oneway'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(3.0)
        result = observer.window(20, ('native', 'original', 'reliable', 'oneway'))
        record('parallel_bridge_window', result)
        if not result['passed']:
            raise RuntimeError('Bridge profiling window has insufficient messages')
    finally:
        os.killpg(bridge.pid, signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGTERM)
            bridge.wait(timeout=5)
        log.close()
    with observer.lock:
        (directory / 'bridge-stamps.json').write_text(json.dumps(observer.samples) + '\n')
