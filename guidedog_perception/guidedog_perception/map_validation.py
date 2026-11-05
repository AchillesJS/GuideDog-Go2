# SPDX-License-Identifier: Apache-2.0
"""Multi-fixture and bounded longer-motion assessment of real Gazebo map evidence."""

import json
import math
import subprocess
import time

from guidedog_perception.clearance_metrics import PoseHistory, yaw_of
from guidedog_perception.local_mapper import write_svg
from guidedog_perception.map_analysis import assess_boxes, assess_connectivity
from guidedog_perception.map_benchmark import MappingDriver
from guidedog_perception.stop_benchmark import displacement, prepare_scenarios, settled_stop
import numpy as np


LIMITS = {
    'min_associated_cells_per_box': 3, 'boundary_error_p95_m': 0.20,
    'boundary_error_max_m': 0.30, 'free_wholly_inside_boxes_cells': 0,
    'max_update_cost_p95_ms': 100.0, 'max_cell_array_bytes': 1024*1024,
    'configured_front_rear_m': 0.45, 'configured_half_width_m': 0.40,
    'assessment_margin_m': 0.10, 'target_forward_motion_m': 2.0,
    'max_lateral_motion_m': 0.30, 'max_heading_excursion_deg': 15.0,
    'max_odom_position_error_m': 0.20, 'max_motion_wall_sec': 600.0,
    'max_motion_source_sec': 180.0, 'request_speed_mps': 0.05,
    'min_body_height_m': 0.15, 'max_body_height_m': 0.40,
    'min_reference_fraction': 0.80, 'min_motion_evaluations': 8,
}
SCENES = {
    'two_boxes': [(2.0, -0.9, 0, 0.5, 0.5), (2.7, 0.9, 10, 0.5, 0.7)],
    'wide_passage': [(2.0, -1.25, 0, 0.5, 0.5), (2.0, 1.25, 0, 0.5, 0.5)],
    'narrow_passage': [(2.0, -0.55, 0, 0.5, 0.5), (2.0, 0.55, 0, 0.5, 0.5)],
    'blocked_passage': [(2.0, -0.55, 0, 0.5, 0.5), (2.0, 0, 0, 0.5, 0.5),
                        (2.0, 0.55, 0, 0.5, 0.5)],
    'long_motion': [(1.0, -1.2, 0, 0.5, 0.5), (2.0, 1.2, 0, 0.5, 0.5),
                    (3.0, -1.2, 0, 0.5, 0.5)],
}


class ValidationDriver(MappingDriver):
    """Observe stamped body truth without providing it to the mapper or veto."""

    def __init__(self):
        """Bound truth history to the same source-time interpolation policy as the audit."""
        super().__init__()
        self.truth_history = PoseHistory()
        self.probe_sequence = 0
        self.probe_expected = None
        self.fixture_probe_hits = 0

    def on_contact(self, msg):
        """Match each positive control to both its unique probe and its own box."""
        super().on_contact(msg)
        with self.lock:
            if self.probe_expected is None:
                return
            probe, box = self.probe_expected
            for contact in msg.contacts:
                names = [contact.collision1.name, contact.collision2.name]
                if (any(probe+'::' in n for n in names)
                        and any(box+'::' in n for n in names)):
                    self.fixture_probe_hits += 1

    def verify_fixture(self, name, position):
        """Require an actual contact for this fixture, not a late previous probe message."""
        self.probe_sequence += 1
        probe = f'contact_probe_validation_{self.probe_sequence}'
        self.probe_expected, self.fixture_probe_hits = (probe, name), 0
        sdf = (f'<sdf version="1.8"><model name="{probe}">'
               f'<pose>{position[0]} {position[1]} 1.2 0 0 0</pose><link name="probe">'
               '<inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx>'
               '<iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>'
               '<collision name="probe_collision"><geometry><sphere><radius>0.05</radius>'
               '</sphere></geometry></collision></link></model></sdf>')
        service('create', 'gz.msgs.EntityFactory', f'sdf: {json.dumps(sdf)} allow_renaming: false')
        try:
            end = time.monotonic()+8
            while time.monotonic() < end and not self.fixture_probe_hits:
                time.sleep(0.05)
            if not self.fixture_probe_hits:
                raise RuntimeError(f'No positive probe contact for {name}')
            return self.fixture_probe_hits
        finally:
            service('remove', 'gz.msgs.Entity', f'name: "{probe}" type: MODEL')
            self.probe_expected = None

    def on_pose(self, msg):
        """Retain full source-time pose in addition to existing wall-time motion samples."""
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise ValueError('unexpected_ground_truth_frames')
        super().on_pose(msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        with self.lock:
            self.truth_history.add(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def paired(self, healthy=True):
        """Require exact map/status content and a bracketed truth pose at that source stamp."""
        snapshot = self.mapping()
        if snapshot is None or (healthy and not snapshot[3]['healthy']):
            return None
        with self.lock:
            truth = self.truth_history.at(snapshot[3]['source_stamp_ns'])
        return (snapshot, truth) if truth is not None else None


def take(driver, seconds=12):
    """Obtain one internally consistent snapshot, without refetching it between checks."""
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        paired = driver.paired()
        if paired is not None:
            return paired
        time.sleep(0.05)
    raise RuntimeError('No fresh content-paired map with source-time ground truth')


def collect(driver, count=3, seconds=15):
    """Require several distinct stamped snapshots rather than selecting one lucky frame."""
    samples, seen = [], set()
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        pair = driver.paired()
        if pair is not None and pair[0][3]['source_stamp_ns'] not in seen:
            samples.append(pair)
            seen.add(pair[0][3]['source_stamp_ns'])
            if len(samples) == count:
                return samples
        time.sleep(0.1)
    raise RuntimeError('Insufficient distinct fresh time-aligned map observations')


def service(action, kind, request):
    """Use only the dedicated test world's existing Gazebo fixture services."""
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/guidedog_stop/{action}', '--reqtype', kind,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
        capture_output=True, text=True, timeout=6)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(f'Fixture {action} failed: {result.stdout} {result.stderr}')


class Fixtures:
    """Own named static boxes and positively verify every new contact sensor."""

    def __init__(self, driver, anchor, rotation, translation):
        """Fix one truth-to-odom reference for the entire series."""
        self.driver, self.anchor = driver, anchor
        self.rotation, self.translation = rotation, translation
        self.names, self.boxes = [], []

    def clear(self):
        """Remove only boxes created by this benchmark."""
        while self.names:
            name = self.names[-1]
            service('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
            self.names.pop()
        self.boxes = []

    def create(self, specs):
        """Probe each box while distant, then place it in the fixed test frame."""
        self.clear()
        x, y, yaw = self.anchor
        c, s = math.cos(yaw), math.sin(yaw)
        for i, (forward, side, degrees, depth, width) in enumerate(specs):
            name = f'map_validation_box_{i}'
            far_x, far_y = x+20+2*i, y+20
            sdf = (f'<sdf version="1.8"><model name="{name}"><static>true</static>'
                   f'<pose>{far_x} {far_y} 0.5 0 0 0</pose><link name="box">'
                   f'<collision name="collision"><geometry><box><size>{depth} {width} 1</size>'
                   '</box></geometry></collision><sensor name="contact" type="contact">'
                   '<contact><collision>collision</collision>'
                   '<topic>/guidedog/obstacle_contacts</topic></contact></sensor>'
                   f'<visual name="visual"><geometry><box><size>{depth} {width} 1</size>'
                   '</box></geometry><material><ambient>0.8 0.15 0.1 1</ambient>'
                   '</material></visual></link></model></sdf>')
            service('create', 'gz.msgs.EntityFactory',
                    f'sdf: {json.dumps(sdf)} allow_renaming: false')
            self.names.append(name)
            probe_hits = self.driver.verify_fixture(name, (far_x, far_y))
            position = np.array([x+c*forward-s*side, y+s*forward+c*side])
            angle = yaw+math.radians(degrees)
            service('set_pose', 'gz.msgs.Pose',
                    f'name: "{name}" position {{ x: {position[0]} y: {position[1]} z: 0.5 }} '
                    f'orientation {{ z: {math.sin(angle/2)} w: {math.cos(angle/2)} }}')
            mapped = self.rotation @ position+self.translation
            heading = angle+math.atan2(self.rotation[1, 0], self.rotation[0, 0])
            self.boxes.append({'name': name, 'pose': [*mapped.tolist(), heading],
                               'size': [depth, width, 1.0],
                               'positive_probe_contacts': probe_hits,
                               'truth_world_pose': [*position.tolist(), angle]})
        return list(self.boxes)


def measure(paired, fixtures, goal, radius):
    """Keep localization, free-space connectivity and state-estimation error separate."""
    snapshot, truth = paired
    data, origin, resolution, status = snapshot
    position, quaternion, bracket = truth
    expected = fixtures.rotation @ position[:2]+fixtures.translation
    actual = np.array(status['pose_odom'][:2])
    expected_yaw = yaw_of(quaternion)+math.atan2(fixtures.rotation[1, 0], fixtures.rotation[0, 0])
    yaw_error = math.atan2(math.sin(status['pose_odom'][2]-expected_yaw),
                           math.cos(status['pose_odom'][2]-expected_yaw))
    return {'source_stamp_ns': status['source_stamp_ns'], 'pose_bracket_sec': bracket,
            'healthy': status['healthy'],
            'observer_age_sec': status['observer_status_age_wall_sec'],
            'odom_position_error_m': float(np.linalg.norm(actual-expected)),
            'odom_heading_error_deg': math.degrees(yaw_error),
            'boxes': assess_boxes(data, origin, resolution, fixtures.boxes),
            'connectivity': assess_connectivity(data, origin, resolution,
                                                actual, goal, radius, status['healthy']),
            'update_cost_p95_ms': status['update_cost_p95_ms'],
            'cell_array_bytes': status['cell_array_bytes'], 'rolls': status['rolls']}


def geometry_passed(metrics):
    """Apply the fixed localization, unsupported-free and resource criteria."""
    boxes = metrics['boxes']
    return (all(b['associated_occupied_cells'] >= LIMITS['min_associated_cells_per_box']
                and b['boundary_error_p95_m'] <= LIMITS['boundary_error_p95_m']
                and b['boundary_error_max_m'] <= LIMITS['boundary_error_max_m']
                for b in boxes['boxes'].values())
            and boxes['free_wholly_inside_boxes_cells'] == 0
            and metrics['odom_position_error_m'] <= LIMITS['max_odom_position_error_m']
            and metrics['update_cost_p95_ms'] <= LIMITS['max_update_cost_p95_ms']
            and metrics['cell_array_bytes'] <= LIMITS['max_cell_array_bytes'])


def save(directory, name, paired, metrics):
    """Save exact measured cells and their source-time references, without raw payloads."""
    snapshot, truth = paired
    data, origin, resolution, status = snapshot
    (directory / f'map-{name}.json').write_text(json.dumps({
        'occupancy': data.tolist(), 'origin': origin.tolist(), 'resolution': resolution,
        'status': status, 'truth_position': truth[0].tolist(),
        'truth_quaternion': truth[1].tolist(), 'metrics': metrics}, indent=2)+'\n')
    robot = (np.array(status['pose_odom'][:2])-origin)/resolution
    write_svg(directory / f'map-{name}.svg', {'occupancy': data}, resolution,
              [*robot.tolist(), status['pose_odom'][2]])


def long_motion(driver, fixtures, goal, radius, directory, record):
    """Attempt 2 m using the unchanged veto; preserve every early abort as a failure."""
    fixtures.create(SCENES['long_motion'])
    time.sleep(5.6)
    before = driver.snapshot()[0]
    paired = take(driver)
    baseline_rolls = paired[0][3]['rolls']
    if driver.snapshot()[1].get('reason') != 'idle':
        raise RuntimeError('Veto is not idle before the longer-motion attempt')
    source_start = driver.snapshot()[1]['ros_time_sec']
    started = time.monotonic()
    driver.desired = LIMITS['request_speed_mps']
    next_checkpoint, next_report = 0.25, started
    reached, reason, error = False, None, None
    progress = lateral = heading = 0.0
    max_lateral = max_heading = 0.0
    evaluations, attempts = [], 0
    try:
        while True:
            pose, status, _, _ = driver.snapshot()
            progress = displacement(before, pose)
            lateral = (-(pose[1]-before[1])*math.sin(before[4])
                       + (pose[2]-before[2])*math.cos(before[4]))
            heading = math.degrees(math.atan2(math.sin(pose[4]-before[4]),
                                              math.cos(pose[4]-before[4])))
            max_lateral = max(max_lateral, abs(lateral))
            max_heading = max(max_heading, abs(heading))
            if (abs(lateral) > LIMITS['max_lateral_motion_m']
                    or abs(heading) > LIMITS['max_heading_excursion_deg']
                    or not LIMITS['min_body_height_m'] <= pose[3] <= LIMITS['max_body_height_m']):
                reason = 'motion_envelope_intervention'
                break
            if driver.contacts:
                reason = 'robot_obstacle_contact'
                break
            if status.get('reason') not in ('moving', 'idle'):
                reason = 'veto_stop:'+str(status.get('reason'))
                break
            if progress >= LIMITS['target_forward_motion_m']:
                reached = True
                break
            if (time.monotonic()-started > LIMITS['max_motion_wall_sec']
                    or status['ros_time_sec']-source_start > LIMITS['max_motion_source_sec']):
                reason = 'motion_deadline'
                break
            if time.monotonic() >= next_report:
                attempts += 1
                pair = driver.paired()
                if pair is not None:
                    metrics = measure(pair, fixtures, goal, radius)
                    evaluations.append(metrics)
                    if progress >= next_checkpoint:
                        save(directory, f'long-{progress:.2f}m', pair, metrics)
                        next_checkpoint += 0.25
                with (directory / 'motion-trace.jsonl').open('a') as output:
                    output.write(json.dumps({'elapsed_wall_sec': time.monotonic()-started,
                                             'progress_m': progress, 'lateral_m': lateral,
                                             'heading_excursion_deg': heading,
                                             'veto_reason': status.get('reason')})+'\n')
                print(f'long motion: {progress:.3f} m; side={lateral:.3f}; '
                      f'heading={heading:.1f} deg', flush=True)
                next_report = time.monotonic()+5
            time.sleep(0.05)
    except (RuntimeError, subprocess.SubprocessError, ValueError) as failure:
        reason, error = 'measurement_or_fixture_failure', str(failure)
    finally:
        driver.desired = 0.0
    try:
        stopped = settled_stop(driver, time.monotonic())
    except RuntimeError as failure:
        stopped = {'settle_error': str(failure)}
        reason = reason or 'did_not_settle'
    paired = driver.paired(healthy=False)
    final = None
    if paired is not None:
        final = measure(paired, fixtures, goal, radius)
        save(directory, 'long-final', paired, final)
    fraction = len(evaluations)/max(1, attempts)
    passed = (reached and len(evaluations) >= LIMITS['min_motion_evaluations']
              and fraction >= LIMITS['min_reference_fraction']
              and all(geometry_passed(m) for m in evaluations)
              and final is not None and final['healthy'] and geometry_passed(final)
              and final['rolls'] > baseline_rolls and 'settle_error' not in stopped
              and not driver.contacts)
    result = {'passed': passed, 'target_reached': reached, 'stop_reason': reason,
              'measurement_error': error,
              'progress_m': progress, 'lateral_m': lateral, 'heading_excursion_deg': heading,
              'max_abs_lateral_motion_m': max_lateral,
              'max_abs_heading_excursion_deg': max_heading,
              'final_forward_progress_m': displacement(before, driver.snapshot()[0]),
              'elapsed_wall_sec': time.monotonic()-started,
              'elapsed_source_sec': driver.snapshot()[1]['ros_time_sec']-source_start,
              'reference_boxes': fixtures.boxes, 'evaluation_count': len(evaluations),
              'reference_fraction': fraction, 'attempted_evaluations': attempts,
              'evaluations': evaluations, 'final': final, **stopped}
    record('long_motion', result)
    return result


def run_validation(driver, record, directory):
    """Run all scenes and expose failed readiness rather than paint unknown space free."""
    radius = math.hypot(LIMITS['configured_front_rear_m'], LIMITS['configured_half_width_m'])
    radius += LIMITS['assessment_margin_m']
    record('map_readiness_limits', {'passed': True, **LIMITS,
                                    'assessment_radius_m': radius,
                                    'assessment_only_not_a_motion_safety_limit': True})
    prepare_scenarios(driver, record)
    snapshot, truth = take(driver)
    position, quaternion, _ = truth
    angle = snapshot[3]['pose_odom'][2]-yaw_of(quaternion)
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array(snapshot[3]['pose_odom'][:2])-rotation @ position[:2]
    anchor = [*position[:2].tolist(), yaw_of(quaternion)]
    fixtures = Fixtures(driver, anchor, rotation, translation)
    goal_world = position[:2]+3*np.array([math.cos(anchor[2]), math.sin(anchor[2])])
    goal = rotation @ goal_world+translation
    failures, readiness = [], {}
    try:
        for name in ('two_boxes', 'wide_passage', 'narrow_passage', 'blocked_passage'):
            try:
                boxes = fixtures.create(SCENES[name])
                time.sleep(5.6)
                pairs = collect(driver)
                observations = [measure(pair, fixtures, goal, radius) for pair in pairs]
                metrics = observations[-1]
                save(directory, name, pairs[-1], metrics)
                passed = all(geometry_passed(m) for m in observations) and not driver.contacts
                gap = None
                if name.endswith('_passage'):
                    intervals = sorted((b[1]-b[4]/2, b[1]+b[4]/2) for b in SCENES[name])
                    gap = max(b[0]-a[1] for a, b in zip(intervals, intervals[1:]))
                record(name, {'passed': passed, 'reference_boxes': boxes,
                              'observations': observations, 'reference_gap_width_m': gap,
                              'reference_gap_fits_assessment_circle': (
                                  gap >= 2*radius if gap is not None else None), **metrics})
                readiness[name] = all(m['connectivity']['planning_ready'] for m in observations)
                if not passed:
                    failures.append(name)
            except (RuntimeError, subprocess.SubprocessError) as error:
                driver.desired = 0.0
                record(name, {'passed': False, 'error': str(error)})
                failures.append(name)
            if driver.contacts:
                raise RuntimeError('Robot contact in a stationary fixture scene')
        try:
            motion = long_motion(driver, fixtures, goal, radius, directory, record)
            if not motion['passed']:
                failures.append('long_motion')
        except (RuntimeError, subprocess.SubprocessError) as error:
            driver.desired = 0.0
            record('long_motion', {'passed': False, 'error': str(error)})
            failures.append('long_motion')
        record('contact_monitor', {
            'passed': driver.contact_messages > 0 and not driver.contacts,
            'messages_received': driver.contact_messages,
            'positive_probe_contact_observations': driver.probe_contacts,
            'robot_obstacle_contacts': list(driver.contacts),
            'robot_obstacle_contact_count': len(driver.contacts)})
        ready = (not failures and readiness.get('two_boxes', False)
                 and readiness.get('wide_passage', False))
        record('planning_readiness', {
            'passed': ready, 'planning_ready': ready, 'failed_validation_cases': failures,
            'reason': ('observed_map_assessment_passed' if ready else
                       'map_or_motion_not_ready_for_planning'),
            'unknown_space_was_not_relabelled': True, 'automatic_turns_enabled': False})
        if not ready:
            raise RuntimeError(f'Map readiness not established; failed cases: {failures}')
    finally:
        driver.desired = 0.0
        fixtures.clear()
