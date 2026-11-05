# SPDX-License-Identifier: Apache-2.0
"""Exercise actual ROS map delivery, delayed TF, frozen time, replay and reset."""

from collections import deque
import json
import time

from geometry_msgs.msg import TransformStamped
from guidedog_perception.local_mapper import LocalMapper
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def test_mapper_time_tf_and_latched_delivery(tmp_path):
    """Old or untransformable data must expire without fresh source timestamps."""
    rclpy.init(args=[
        '--ros-args', '-p', f'report_dir:={tmp_path}',
        '-p', 'use_sim_time:=true', '-p', 'free_ttl:=0.3',
        '-p', 'occupied_ttl:=0.8', '-p', 'self_filter_mode:=geometry'], domain_id=80)
    source = Node('map_test_source', use_global_arguments=False)
    mapper = LocalMapper()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(mapper)
    clock_pub = source.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    model_pub = source.create_publisher(
        String, '/robot_description', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                 durability=DurabilityPolicy.TRANSIENT_LOCAL))
    model = ('<robot name="fixture"><link name="test_body"><visual><origin xyz="0 0 .5"/>'
             '<geometry><sphere radius=".1"/></geometry></visual></link></robot>')
    model_pub.publish(String(data=model))
    scan_pub = source.create_publisher(PointCloud2, '/unitree_lidar/points',
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    broadcaster = TransformBroadcaster(source)
    maps, statuses = [], []
    subscriptions = [source.create_subscription(
        String, '/guidedog/local_map_status',
        lambda msg: statuses.append(json.loads(msg.data)), 1)]
    transforms = deque()
    sim_ns = 1000000000
    last_scan = None
    last_emit = 0

    def advance(seconds, send_scan=True, freeze=False, replay=False, frame='base_footprint',
                body_tf=True):
        nonlocal sim_ns, last_scan, last_emit
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now-last_emit >= 0.04:
                last_emit = now
                if not freeze:
                    sim_ns += 40000000
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(sim_ns, 1000000000)
                clock_pub.publish(clock)
                tf = TransformStamped()
                tf.header.stamp = clock.clock
                tf.header.frame_id, tf.child_frame_id = 'odom', 'base_footprint'
                tf.transform.rotation.w = 1.0
                transforms.append((now, tf))
                if body_tf:
                    body = TransformStamped()
                    body.header.stamp = clock.clock
                    body.header.frame_id, body.child_frame_id = 'base_footprint', 'test_body'
                    body.transform.rotation.w = 1.0
                    transforms.append((now, body))
                # Keep exact TF behind newer clouds to exercise the bounded pending queue.
                while transforms and now-transforms[0][0] >= 0.06:
                    broadcaster.sendTransform(transforms.popleft()[1])
                if send_scan:
                    scan = PointCloud2()
                    scan.header.stamp, scan.header.frame_id = clock.clock, frame
                    p = np.array([[x, y, 0] for x in np.linspace(0.8, 2.8, 40)
                                  for y in np.linspace(-0.8, 0.8, 25)] +
                                 [[1.42, 0.02, 0.2], [1.43, 0.02, 0.2],
                                  [1.42, 0.03, 0.2]], dtype=np.float32)
                    scan.height, scan.width = 1, len(p)
                    scan.fields = [PointField(name=k, offset=i*4, datatype=7, count=1)
                                   for i, k in enumerate(('x', 'y', 'z'))]
                    scan.point_step, scan.row_step = 12, len(p)*12
                    scan.data = p.tobytes()
                    scan_pub.publish(last_scan if replay and last_scan is not None else scan)
                    if not replay:
                        last_scan = scan
            executor.spin_once(timeout_sec=0.005)

    try:
        advance(.4, body_tf=False)
        assert mapper.accepted == 0
        advance(1.5)
        assert mapper.accepted >= 3
        assert statuses[-1]['healthy']
        subscriptions.append(source.create_subscription(
            OccupancyGrid, '/guidedog/local_map', maps.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        advance(0.3, send_scan=False, freeze=True)
        assert maps and 100 in maps[-1].data
        advance(1.1, send_scan=False, freeze=True)
        assert set(maps[-1].data) == {-1}
        assert not statuses[-1]['healthy']
        # Already queued valid clouds may finish before expiry. Once they have
        # drained, advancing the clock without scans must not re-stamp the map.
        stamp = maps[-1].header.stamp
        accepted = mapper.accepted
        advance(0.3, send_scan=False)
        assert maps[-1].header.stamp == stamp
        assert mapper.accepted == accepted
        advance(0.6, replay=True)
        assert mapper.accepted == accepted
        assert set(maps[-1].data) == {-1}
        # A backward ROS clock must clear the old spatial epoch before recovery.
        sim_ns = 100000000
        transforms.clear()
        advance(0.25, send_scan=False)
        assert mapper.grid.resets >= 1
        assert mapper.last_source is None
        advance(0.9)
        assert mapper.accepted > accepted
        assert 100 in maps[-1].data
        last_valid_stamp = last_scan.header.stamp.sec*1000000000 + last_scan.header.stamp.nanosec
        advance(1.2, frame='frame_without_tf')
        # A valid cloud already waiting for TF may still complete; no missing-frame
        # cloud can advance the accepted source stamp or refresh expired cells.
        assert mapper.last_source <= last_valid_stamp
        assert set(maps[-1].data) == {-1}
        assert not statuses[-1]['healthy']
        advance(.7)
        assert statuses[-1]['healthy']
        last_body_stamp = sim_ns  # last emitted body pose, including the delayed TF queue
        advance(1.2, body_tf=False)
        assert mapper.last_source <= last_body_stamp
        assert not statuses[-1]['healthy'] and set(maps[-1].data) == {-1}
        resets = mapper.grid.resets
        model_pub.publish(String(data='<robot invalid'))
        advance(.4)
        assert mapper.body.model is None and mapper.grid.resets > resets
        assert not statuses[-1]['healthy'] and set(maps[-1].data) == {-1}
        model_pub.publish(String(data=model))
        advance(.8)
        assert mapper.body.model is not None and statuses[-1]['healthy']
        assert (tmp_path / 'local_map.svg').exists()
        assert (tmp_path / 'local_map.npz').exists()
    finally:
        for subscription in subscriptions:
            source.destroy_subscription(subscription)
        executor.shutdown()
        mapper.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
