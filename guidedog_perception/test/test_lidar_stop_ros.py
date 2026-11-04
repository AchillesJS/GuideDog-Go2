# SPDX-License-Identifier: Apache-2.0
"""Exercise cloud input, TF, rearming and the independent ROS watchdog."""

from collections import deque
import time

from geometry_msgs.msg import TransformStamped, Twist
from guidedog_perception.lidar_stop import LidarStop
from guidedog_perception.velocity_watchdog import VelocityWatchdog
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def test_ros_veto_and_downstream_timeout(tmp_path):
    """A real ROS chain must veto obstacles, stale scans and a stopped gate."""
    rclpy.init(args=[
        '--ros-args', '-p', f'report_dir:={tmp_path}', '-p', 'use_sim_time:=true',
        '-p', 'scan_timeout:=0.4'], domain_id=78)
    source = Node('stop_test_source', use_global_arguments=False)
    gate, watchdog = LidarStop(), VelocityWatchdog()
    executor = SingleThreadedExecutor()
    for node in (source, gate, watchdog):
        executor.add_node(node)
    clock_pub = source.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    scan_pub = source.create_publisher(PointCloud2, '/unitree_lidar/points',
                                       qos_profile_sensor_data)
    command_pub = source.create_publisher(Twist, '/guidedog/cmd_vel_request', 1)
    outputs = []
    subscription = source.create_subscription(
        Twist, '/guidedog/cmd_vel_checked', lambda msg: outputs.append(msg.linear.x), 1)
    broadcaster = StaticTransformBroadcaster(source)
    transform = TransformStamped()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'test_lidar'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    started = time.monotonic()

    def run(seconds, speed=0.0, obstacle=False, scans=True, requests=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            timestamp = int((100 + time.monotonic() - started) * 1e9)
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(timestamp, 1_000_000_000)
            clock_pub.publish(clock)
            if scans:
                points = [[x, y, 0] for x in np.linspace(1, 3, 20)
                          for y in (-0.2, 0, 0.2)]
                if obstacle:
                    points.append([1, 0, 0.3])
                cloud = PointCloud2()
                cloud.header.frame_id = 'test_lidar'
                cloud.header.stamp = clock.clock
                cloud.width, cloud.height = len(points), 1
                cloud.point_step, cloud.row_step = 12, len(points) * 12
                cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                                for i, name in enumerate(('x', 'y', 'z'))]
                cloud.data = np.asarray(points, dtype='<f4').tobytes()
                scan_pub.publish(cloud)
            if requests:
                command = Twist()
                command.linear.x = speed
                command_pub.publish(command)
            until = time.monotonic() + 0.025
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)

    try:
        run(0.7)
        run(0.4, speed=0.1)
        assert outputs and outputs[-1] > 0
        run(0.4, speed=0.1, obstacle=True)
        assert outputs[-1] == 0 and gate.last_reason == 'obstacle'
        run(0.3, speed=0.1)
        assert outputs[-1] == 0 and gate.last_reason == 'neutral_command_required'
        run(0.2)
        run(0.3, speed=0.1)
        assert outputs[-1] > 0
        run(0.7, speed=0.1, scans=False)
        assert outputs[-1] == 0 and gate.last_reason == 'stale_scan'
        run(0.3)
        run(0.3, speed=0.1)
        assert outputs[-1] > 0
        executor.remove_node(gate)
        run(0.45, speed=0.1)
        assert outputs[-1] == 0
        assert (tmp_path / 'stop_events.jsonl').is_file()
        assert subscription is not None
    finally:
        executor.shutdown()
        for node in (gate, watchdog, source):
            node.destroy_node()
        rclpy.shutdown()


def test_delayed_tf_does_not_starve_cloud_processing(tmp_path):
    """Frequent newer scans must not evict scans waiting for delayed exact TF."""
    rclpy.init(args=[
        '--ros-args', '-p', f'report_dir:={tmp_path}', '-p', 'use_sim_time:=true',
        '-p', 'scan_timeout:=0.4'], domain_id=79)
    source = Node('delayed_tf_source', use_global_arguments=False)
    gate = LidarStop()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(gate)
    clock_pub = source.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    scan_pub = source.create_publisher(PointCloud2, '/unitree_lidar/points',
                                       qos_profile_sensor_data)
    command_pub = source.create_publisher(Twist, '/guidedog/cmd_vel_request', 1)
    broadcaster = TransformBroadcaster(source)
    transforms = deque()
    points = np.asarray([[x, y, 0] for x in np.linspace(1, 3, 20)
                         for y in (-0.2, 0, 0.2)], dtype='<f4')
    started = time.monotonic()
    moving_samples = []
    try:
        while time.monotonic() - started < 2.5:
            now = time.monotonic()
            timestamp = int((100 + now - started) * 1e9)
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(timestamp, 1_000_000_000)
            clock_pub.publish(clock)
            cloud = PointCloud2()
            cloud.header.frame_id = 'delayed_lidar'
            cloud.header.stamp = clock.clock
            cloud.width, cloud.height = len(points), 1
            cloud.point_step, cloud.row_step = 12, len(points) * 12
            cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.data = points.tobytes()
            scan_pub.publish(cloud)
            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.header.stamp = clock.clock
            transform.child_frame_id = 'delayed_lidar'
            transform.transform.rotation.w = 1.0
            transforms.append((now, transform))
            while transforms and now - transforms[0][0] >= 0.10:
                broadcaster.sendTransform(transforms.popleft()[1])
            command = Twist()
            command.linear.x = 0.05 if now - started > 1.0 else 0.0
            command_pub.publish(command)
            until = time.monotonic() + 0.025
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)
            if now - started > 1.5:
                moving_samples.append(gate.last_status.get('output_speed_mps', 0))
        assert moving_samples and all(value > 0 for value in moving_samples)
        assert gate.gate.scan_at > started + 2.0
        assert len(gate.pending) <= 8
    finally:
        executor.shutdown()
        gate.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
