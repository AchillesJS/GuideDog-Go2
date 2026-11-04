# SPDX-License-Identifier: Apache-2.0
"""Exercise real ROS subscriptions, TF, report files and a frozen simulation clock."""

import json
import time

from geometry_msgs.msg import TransformStamped
from guidedog_perception.sensor_monitor import SensorMonitor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import StaticTransformBroadcaster


def test_report_and_stale_detection_with_paused_clock(tmp_path):
    """A stopped source must become stale even when its ROS timestamp stays current."""
    rclpy.init(args=[
        '--ros-args', '-p', f'report_dir:={tmp_path}', '-p', 'use_sim_time:=true',
        '-p', 'report_period_sec:=0.2', '-p', 'stale_after_sec:=0.4'], domain_id=77)
    source = Node('diagnostics_test_source', use_global_arguments=False)
    monitor = SensorMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(monitor)
    publishers = {
        'clock': source.create_publisher(Clock, '/clock', qos_profile_sensor_data),
        'rgb': source.create_publisher(Image, '/rgb_image', qos_profile_sensor_data),
        'lidar': source.create_publisher(
            PointCloud2, '/unitree_lidar/points', qos_profile_sensor_data),
    }
    broadcaster = StaticTransformBroadcaster(source)
    transforms = []
    for child in ('camera_link', 'lidar_l1_link'):
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    broadcaster.sendTransform(transforms)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    try:
        spin_for(0.5)
        for i in range(12):
            clock = Clock()
            clock.clock.sec = 100
            clock.clock.nanosec = i * 50_000_000
            publishers['clock'].publish(clock)
            spin_for(0.02)
            image = Image()
            image.header.stamp = clock.clock
            image.header.frame_id = 'camera_link'
            image.width, image.height, image.step = 1, 1, 3
            image.encoding = 'rgb8'
            image.data = [0, 0, 0]
            cloud = PointCloud2()
            cloud.header.stamp = clock.clock
            cloud.header.frame_id = 'lidar_l1_link'
            publishers['rgb'].publish(image)
            publishers['lidar'].publish(cloud)
            spin_for(0.03)
        monitor.write_report()
        healthy = json.loads((tmp_path / 'report.json').read_text())
        assert healthy['clock']['healthy']
        for name in ('rgb', 'lidar'):
            assert healthy['sensors'][name]['messages_total'] >= 8
            assert healthy['sensors'][name]['tf_to_base_at_message_time']
            assert healthy['sensors'][name]['status'] == 'OK'
        spin_for(0.7)
        monitor.write_report(final=True)
        stale = json.loads((tmp_path / 'report.json').read_text())
        assert not stale['clock']['healthy']
        assert 'stale_stream' in stale['sensors']['rgb']['issues']
        assert 'clock_unavailable_or_stalled' in stale['sensors']['lidar']['issues']
        assert stale['final']
        assert (tmp_path / 'report.md').is_file()
        assert len((tmp_path / 'samples.jsonl').read_text().splitlines()) >= 2
    finally:
        executor.shutdown()
        monitor.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_intentionally_disabled_rgb_is_reported_as_disabled(tmp_path):
    """An optional camera must not be reported as a failed required stream."""
    rclpy.init(args=[
        '--ros-args', '-p', f'report_dir:={tmp_path}', '-p', 'rgb_enabled:=false'], domain_id=77)
    monitor = SensorMonitor()
    try:
        monitor.write_report(final=True)
        report = json.loads((tmp_path / 'report.json').read_text())
        assert report['sensors']['rgb']['status'] == 'DISABLED'
        assert report['sensors']['rgb']['issues'] == []
        assert not report['sensors']['rgb']['enabled']
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
