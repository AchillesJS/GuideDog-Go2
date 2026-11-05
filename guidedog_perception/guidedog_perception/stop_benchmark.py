# SPDX-License-Identifier: Apache-2.0
"""Repeatable robot-motion and fault scenarios in the dedicated Gazebo world."""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from guidedog_perception.clearance_metrics import box_clearance
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class ScenarioDriver(Node):
    """Relay real scans, record ground truth and publish expiring test requests."""

    def __init__(self):
        """Keep sensor payloads out of test reports and bound measurement history."""
        super().__init__('guidedog_scenario_driver')
        self.desired = 0.0
        self.audit = None
        self.commands_enabled = True
        self.relay_enabled = True
        self.pose = None
        self.status = {}
        self.poses = deque(maxlen=20000)
        self.outputs = deque(maxlen=20000)
        self.contact_messages = 0
        self.probe_contacts = 0
        self.contacts = []
        self.lock = threading.Lock()
        self.request_pub = self.create_publisher(Twist, '/guidedog/cmd_vel_request', 1)
        lidar_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.relay_pub = self.create_publisher(
            PointCloud2, '/guidedog/test_lidar', lidar_qos)
        self.create_subscription(PointCloud2, '/unitree_lidar/points', self.relay,
                                 lidar_qos)
        self.create_subscription(Odometry, '/odom/ground_truth', self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/guidedog/cmd_vel_checked', self.on_output, 1)
        self.create_subscription(String, '/guidedog/stop_status', self.on_status, 1)
        self.create_subscription(Contacts, '/guidedog/obstacle_contacts', self.on_contact, 10)
        self.controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.publish_request, clock=self.steady_clock)

    def publish_request(self):
        """Continue the commanded heartbeat while the test thread observes motion."""
        if self.commands_enabled:
            msg = Twist()
            msg.linear.x = self.desired
            self.request_pub.publish(msg)

    def relay(self, msg):
        """Inject scan loss by withholding the real stream, never synthetic geometry."""
        if self.relay_enabled:
            self.relay_pub.publish(msg)
        if self.audit is not None:
            self.audit.on_cloud(msg)

    def on_pose(self, msg):
        """Record physical Gazebo position rather than commanded distance."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self.lock:
            self.pose = (time.monotonic(), p.x, p.y, p.z, yaw)
            self.poses.append(self.pose)
        if self.audit is not None:
            self.audit.on_pose(msg)

    def on_output(self, msg):
        """Observe the final watchdog-to-controller command topic."""
        with self.lock:
            self.outputs.append((time.monotonic(), msg.linear.x))

    def on_status(self, msg):
        """Retain the current veto reason and its observer receipt timestamp."""
        with self.lock:
            self.status = json.loads(msg.data)
            self.status['received_monotonic'] = time.monotonic()

    def on_contact(self, msg):
        """Treat unexpected obstacle contacts conservatively as a failed scenario."""
        with self.lock:
            self.contact_messages += 1
            for contact in msg.contacts:
                names = [contact.collision1.name, contact.collision2.name]
                if any('contact_probe' in name for name in names):
                    self.probe_contacts += 1
                    continue
                if not any('ground_plane' in name for name in names):
                    self.contacts.append({'time': time.monotonic(), 'collisions': names})

    def snapshot(self):
        """Read a consistent snapshot shared with the ROS callback thread."""
        with self.lock:
            return self.pose, dict(self.status), list(self.poses), list(self.outputs)

    def place_obstacle(self, forward, side, yaw_delta=0.0):
        """Move the test box relative to current robot ground-truth heading."""
        pose = self.snapshot()[0]
        if pose is None:
            raise RuntimeError('No ground-truth pose')
        _, x, y, _, yaw = pose
        x += math.cos(yaw)*forward - math.sin(yaw)*side
        y += math.sin(yaw)*forward + math.cos(yaw)*side
        yaw += yaw_delta
        request = (f'name: "test_obstacle" position {{ x: {x} y: {y} z: 0.5 }} '
                   f'orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}')
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/guidedog_stop/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=6)
        if result.returncode or 'true' not in result.stdout:
            raise RuntimeError(f'Obstacle placement failed: {result.stdout} {result.stderr}')
        return x, y, yaw

    def verify_contact_sensor(self, box):
        """Drop an isolated probe onto the distant box to verify the whole contact path."""
        previous_contacts = self.probe_contacts
        sdf = (f'<sdf version="1.8"><model name="contact_probe">'
               f'<pose>{box[0]} {box[1]} 1.2 0 0 0</pose><link name="probe">'
               '<inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx>'
               '<iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>'
               '<collision name="probe_collision"><geometry><sphere><radius>0.05</radius>'
               '</sphere></geometry></collision></link></model></sdf>')
        request = f'sdf: {json.dumps(sdf)} allow_renaming: false'
        created = subprocess.run(
            ['gz', 'service', '-s', '/world/guidedog_stop/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=6)
        if created.returncode or 'true' not in created.stdout:
            raise RuntimeError(f'Contact probe creation failed: {created.stdout} {created.stderr}')
        try:
            wait_until(lambda: self.probe_contacts > previous_contacts, 8,
                       'Contact positive control failed: no probe/box contact received')
        finally:
            removed = subprocess.run(
                ['gz', 'service', '-s', '/world/guidedog_stop/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000', '--req', 'name: "contact_probe" type: MODEL'],
                capture_output=True, text=True, timeout=6)
            if removed.returncode or 'true' not in removed.stdout:
                raise RuntimeError('Contact probe cleanup failed')


def wait_until(predicate, seconds, message):
    """Fail within a bounded wall-time interval."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError(message)


def displacement(a, b):
    """Project ground-truth displacement onto the initial forward heading."""
    return (b[1] - a[1])*math.cos(a[4]) + (b[2] - a[2])*math.sin(a[4])


def settled_stop(driver, started):
    """Measure post-zero travel over two wall seconds using actual pose samples."""
    wait_until(lambda: driver.snapshot()[3] and driver.snapshot()[3][-1][1] == 0,
               1.5, 'Final command did not become zero')
    pose_at_zero = driver.snapshot()[0]
    time.sleep(2.0)
    pose, _, poses, outputs = driver.snapshot()
    relevant = [p for p in poses if p[0] >= pose_at_zero[0]]
    travel = max([0.0] + [displacement(pose_at_zero, p) for p in relevant])
    zeros = [t for t, value in outputs if t >= started and value == 0]
    recent = [p for p in poses if p[0] >= pose[0] - 0.8]
    spread = max([0.0] + [abs(displacement(recent[0], p)) for p in recent])
    if travel > 0.20 or spread > 0.03:
        raise RuntimeError(f'Robot did not settle: travel={travel:.3f}, spread={spread:.3f}')
    return {'observed_zero_delay_wall_sec': zeros[0] - started if zeros else None,
            'post_zero_forward_travel_m': travel, 'settled_position_spread_m': spread}


def prepare_scenarios(driver, record):
    """Wait for active controllers and positively verify the contact sensor path."""
    wait_until(lambda: driver.snapshot()[0] is not None, 60, 'No Gazebo ground truth')
    distant_box = driver.place_obstacle(20, 20)
    wait_until(lambda: driver.snapshot()[1].get('reason') == 'idle', 45,
               'Guard never became ready on observed flat ground')
    wait_until(lambda: driver.controllers.service_is_ready(), 30, 'No controller manager')
    active = set()
    deadline = time.monotonic() + 60
    expected = {'joint_states_controller', 'joint_group_effort_controller'}
    while time.monotonic() < deadline and not expected <= active:
        future = driver.controllers.call_async(ListControllers.Request())
        wait_until(future.done, 5, 'Controller query timed out')
        active = {c.name for c in future.result().controller if c.state == 'active'}
        time.sleep(0.2)
    if not expected <= active:
        raise RuntimeError(f'Controllers are not active: {active}')
    checked = driver.get_subscriptions_info_by_topic('/guidedog/cmd_vel_checked')
    raw = driver.get_subscriptions_info_by_topic('/cmd_vel')
    if not any(e.node_name == 'quadruped_controller_node' for e in checked):
        raise RuntimeError('Controller is not connected to the checked topic')
    if any(e.node_name == 'quadruped_controller_node' for e in raw):
        raise RuntimeError('Controller has a bypass subscription')
    record('controller_route', {
        'passed': True, 'controller_input': '/guidedog/cmd_vel_checked',
        'raw_cmd_vel_bypass': False,
        'guard_parameters': driver.snapshot()[1]['parameters']})
    settled_from = driver.snapshot()[1]['ros_time_sec']
    wait_until(lambda: driver.snapshot()[1].get('ros_time_sec', 0) >= settled_from + 2.0,
               20, 'Simulation did not advance while settling the robot')
    driver.verify_contact_sensor(distant_box)
    record('contact_positive_control', {'passed': True, 'probe_contacts': driver.probe_contacts})


def run_scenarios(driver, record):
    """Test free/side/front paths, rearm latch and two independent data timeouts."""
    prepare_scenarios(driver, record)

    def arm():
        driver.commands_enabled = True
        driver.desired = 0.0
        wait_until(lambda: driver.snapshot()[1].get('reason') == 'idle', 4,
                   'Explicit neutral rearm failed')
        time.sleep(0.3)

    for name, side, seconds in (('free_path', 20, 6.0), ('side_obstacle', 1.2, 5.0)):
        driver.place_obstacle(1.6 if side < 2 else 20, side)
        arm()
        before = driver.snapshot()[0]
        sim_start = driver.snapshot()[1]['ros_time_sec']
        driver.desired = 0.1
        wait_until(lambda: driver.snapshot()[1].get('ros_time_sec', 0) >= sim_start + seconds,
                   35, 'Simulation stalled during a motion scenario')
        progress = displacement(before, driver.snapshot()[0])
        if progress < 0.08:
            raise RuntimeError(f'{name}: insufficient forward progress {progress:.3f} m; '
                               f'guard={driver.snapshot()[1].get("reason")}')
        driver.desired = 0.0
        started = time.monotonic()
        stopping = settled_stop(driver, started)
        record(name, {'passed': True, 'forward_progress_m': progress, **stopping})

    arm()
    box = driver.place_obstacle(1.75, 0)
    if driver.audit is not None:
        driver.audit.set_box(box)
    time.sleep(0.7)
    before = driver.snapshot()[0]
    driver.desired = 0.1

    def stopped_before_contact():
        pose, status, _, _ = driver.snapshot()
        distance = ((box[0]-pose[1])*math.cos(pose[4])
                    + (box[1]-pose[2])*math.sin(pose[4]) - 0.25 - 0.45)
        if distance < 0.20:
            driver.desired = 0.0
            raise RuntimeError('Intervened before collision: guard missed the stop')
        return status.get('reason') == 'obstacle'

    wait_until(stopped_before_contact, 20,
               'Front obstacle did not trigger a stop')
    detected = driver.snapshot()[1]
    detection_time = detected['received_monotonic']
    stopping = settled_stop(driver, detection_time)
    pose, status, _, _ = driver.snapshot()
    clearance, legacy_projected = box_clearance(box, (0.5, 1, 1), pose[1:4], pose[4])
    progress = displacement(before, pose)
    if progress < 0.03 or clearance is None or clearance < 0.20 or driver.contacts:
        raise RuntimeError(f'Front stop failed: progress={progress}, clearance={clearance}')
    record('front_obstacle', {
        'passed': True, 'forward_progress_m': progress,
        'ground_truth_clearance_m': clearance,
        'legacy_projected_clearance_m': legacy_projected,
        'clearance_metric_version': 2,
        'first_detected_clearance_m': detected['scan']['clearance_m'],
        'stopped_scan_clearance_m': status['scan']['clearance_m'],
        'detection_ros_time_sec': detected['ros_time_sec'], **stopping})
    if driver.audit is not None:
        records = driver.audit.snapshot()
        matched = [r for r in records if r['stamp_ns'] == detected['scan_stamp_ns']]
        if not matched:
            raise RuntimeError('Detection cloud has no time-aligned reference sample')
        record('clearance_audit', {'passed': bool(matched),
                                   'detection_sample': matched[-1] if matched else None,
                                   'source_clouds': driver.audit.received,
                                   'unpaired_clouds': driver.audit.unpaired,
                                   'max_receipt_gap_sec': driver.audit.max_receipt_gap})
    cleared_at = time.monotonic()
    cleared_pose = driver.snapshot()[0]
    driver.place_obstacle(20, 20)
    time.sleep(1.0)
    pose, _, _, outputs = driver.snapshot()
    after_clear = [value for t, value in outputs if t >= cleared_at]
    held_travel = displacement(cleared_pose, pose)
    if not after_clear or any(value != 0 for value in after_clear) or held_travel > 0.03:
        raise RuntimeError('Held command restarted after obstacle removal')
    record('explicit_rearm', {'passed': True, 'held_command_remained_stopped': True,
                              'forward_travel_after_clear_m': held_travel})

    arm()
    before = driver.snapshot()[0]
    driver.desired = 0.1
    time.sleep(1.5)
    progress = displacement(before, driver.snapshot()[0])
    if driver.snapshot()[3][-1][1] <= 0 or progress < 0.005:
        raise RuntimeError('Robot was not moving before scan-loss injection')
    injected = time.monotonic()
    driver.relay_enabled = False
    scan_deadline = driver.snapshot()[1]['parameters']['scan_timeout'] + 0.4
    wait_until(lambda: driver.snapshot()[1].get('reason') == 'stale_scan', scan_deadline,
               'Scan-loss stop missed its deadline')
    stopping = settled_stop(driver, injected)
    if (stopping['observed_zero_delay_wall_sec'] is None
            or stopping['observed_zero_delay_wall_sec'] > scan_deadline + 0.1):
        raise RuntimeError('Final scan-loss output missed its deadline')
    record('scan_loss', {'passed': True, 'fault': 'real_scan_relay_disabled',
                         'pre_fault_forward_progress_m': progress, **stopping})
    driver.relay_enabled = True
    arm()
    before = driver.snapshot()[0]
    driver.desired = 0.1
    time.sleep(1.5)
    progress = displacement(before, driver.snapshot()[0])
    if driver.snapshot()[3][-1][1] <= 0 or progress < 0.005:
        raise RuntimeError('Robot was not moving before command-loss injection')
    injected = time.monotonic()
    driver.commands_enabled = False
    wait_until(lambda: driver.snapshot()[1].get('reason') == 'command_timeout', 0.8,
               'Command-loss stop missed its deadline')
    stopping = settled_stop(driver, injected)
    if (stopping['observed_zero_delay_wall_sec'] is None
            or stopping['observed_zero_delay_wall_sec'] > 0.9):
        raise RuntimeError('Final command-loss output missed its deadline')
    record('command_loss', {'passed': True, 'pre_fault_forward_progress_m': progress, **stopping})
    if driver.contacts:
        raise RuntimeError(f'Obstacle contacts detected: {driver.contacts}')
    record('contact_monitor', {'passed': driver.contact_messages > 0,
                               'messages_received': driver.contact_messages,
                               'robot_obstacle_contacts': len(driver.contacts)})
    if not driver.contact_messages:
        raise RuntimeError('No contact sensor messages: collision check is unverified')


def main():
    """Start an owned simulation, execute scenarios and always stop all owned work."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', default=str(Path.home() / '.ros/guidedog/reports' /
                        datetime.now(timezone.utc).strftime('stop_test_%Y%m%dT%H%M%S_%fZ')))
    parser.add_argument('--audit-clearance', action='store_true')
    parser.add_argument('--audit-motion', action='store_true')
    parser.add_argument('--record-gazebo', action='store_true')
    parser.add_argument('--clearance-matrix', action='store_true')
    parser.add_argument('--bridge-profile', action='store_true')
    parser.add_argument('--map-benchmark', action='store_true')
    parser.add_argument('--map-validation', action='store_true')
    parser.add_argument('--motion-only', action='store_true',
                        help='Run only the unchanged longer-motion/map criteria')
    parser.add_argument('--floor-audit', action='store_true',
                        help='Stationary observed-floor, finite-hole and scan-loss audit')
    parser.add_argument('--gait-profile', choices=['stock', 'low_lift', 'low_lift_calibrated'],
                        default='low_lift_calibrated')
    parser.add_argument('--sensor-profile', choices=['pinned', 'l1_near'], default='pinned')
    parser.add_argument('--self-filter-mode', choices=['geometry', 'legacy_body'],
                        default='geometry')
    parser.add_argument('--headless', action='store_true',
                        help='Use Gazebo server-only EGL rendering, with the same sensors')
    parser.add_argument('--with-local-map', action='store_true')
    args = parser.parse_args()
    if args.floor_audit and any((args.motion_only, args.map_validation, args.map_benchmark,
                                 args.bridge_profile, args.clearance_matrix,
                                 args.audit_clearance)):
        parser.error('--floor-audit must run separately from other scenario modes')
    directory = Path(args.report_dir).expanduser()
    if directory.exists() and any(directory.iterdir()):
        parser.error('report directory must be empty')
    directory.mkdir(parents=True, exist_ok=True)
    result = {'passed': False, 'cases': {}, 'simulation_only': True,
              'render_profile': 'server_egl' if args.headless else 'desktop_gui',
              'gait_profile': args.gait_profile,
              'sensor_profile': args.sensor_profile,
              'self_filter_mode': args.self_filter_mode,
              'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
              'gz_partition': os.environ.get('GZ_PARTITION')}

    def record(name, values):
        result['cases'][name] = values
        (directory / 'benchmark.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f'{name}: {values}', flush=True)

    launch_log = (directory / 'launch.log').open('w')
    mapping_enabled = (args.map_benchmark or args.with_local_map or args.map_validation
                       or args.motion_only or args.floor_audit)
    process = subprocess.Popen(
        [sys.executable, '-m', 'guidedog_perception.launch_supervisor',
         'guidedog_perception', 'obstacle_stop.launch.py',
         f'report_dir:={directory / "session"}', 'lidar_topic:=/guidedog/test_lidar',
         f'enable_local_map:={str(mapping_enabled).lower()}',
         f'headless:={str(args.headless).lower()}',
         f'gait_profile:={args.gait_profile}',
         f'sensor_profile:={args.sensor_profile}',
         f'self_filter_mode:={args.self_filter_mode}',
         f'record_gazebo:={str(args.record_gazebo).lower()}'],
        stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    if args.map_validation or args.motion_only or args.floor_audit:
        from guidedog_perception.map_validation import ValidationDriver
        driver = ValidationDriver()
    elif args.map_benchmark:
        from guidedog_perception.map_benchmark import MappingDriver
        driver = MappingDriver()
    else:
        driver = ScenarioDriver()
    if args.audit_clearance or args.clearance_matrix:
        from guidedog_perception.clearance_probe import ClearanceProbe
        driver.audit = ClearanceProbe(driver, directory)
    if args.floor_audit:
        from guidedog_perception.floor_audit import FloorProbe
        driver.audit = FloorProbe(driver, directory, mode=args.self_filter_mode)
    motion_audit = None
    if args.audit_motion:
        from guidedog_perception.motion_audit import MotionAudit
        motion_audit = MotionAudit(driver, directory)

    def spin():
        try:
            rclpy.spin(driver)
        except ExternalShutdownException:
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        if args.floor_audit:
            from guidedog_perception.floor_audit import run_floor_audit
            run_floor_audit(driver, record, directory, args.sensor_profile)
        elif args.map_validation or args.motion_only:
            from guidedog_perception.map_validation import run_validation
            run_validation(driver, record, directory, motion_only=args.motion_only)
        elif args.map_benchmark:
            from guidedog_perception.map_benchmark import run_mapping
            run_mapping(driver, record, directory)
        elif args.bridge_profile:
            from guidedog_perception.bridge_profile import run_profile
            run_profile(driver, record, directory)
        elif args.clearance_matrix:
            from guidedog_perception.clearance_matrix import run_matrix
            run_matrix(driver, record)
        else:
            run_scenarios(driver, record)
        result['passed'] = True
    except (RuntimeError, subprocess.SubprocessError) as error:
        result['error'] = str(error)
        pose, status, poses, _ = driver.snapshot()
        result['failure_observation'] = {
            'status': status, 'last_ground_truth_pose': pose,
            'recent_ground_truth_poses': [p for p in poses if p[0] > time.monotonic() - 8],
        }
        print(f'FAILED: {error}', flush=True)
    finally:
        driver.desired = 0.0
        driver.commands_enabled = True
        time.sleep(0.5)
        rclpy.shutdown()
        spinner.join(timeout=3)
        if motion_audit is not None:
            motion_audit.close(directory)
        if driver.audit is not None:
            driver.audit.close()
        driver.destroy_node()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        result['launch_exit_code'] = process.returncode
        launch_log.close()
        (directory / 'benchmark.json').write_text(json.dumps(result, indent=2) + '\n')
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
