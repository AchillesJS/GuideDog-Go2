# SPDX-License-Identifier: Apache-2.0
"""Stationary native-cloud floor audit; no truth data enters the deployed mapper."""

import hashlib
import json
import time
import xml.etree.ElementTree as ET

from guidedog_perception.clearance_probe import ClearanceProbe
from guidedog_perception.guard_geometry import cloud_xyz, inspect_corridor, transform_xyz
from guidedog_perception.map_benchmark import save_snapshot, wait_mapping
from guidedog_perception.map_validation import service, take
from guidedog_perception.stop_benchmark import prepare_scenarios, wait_until
from guidedog_perception.support_evidence import point_range_masks, scan_floor_coverage
import numpy as np
from rcl_interfaces.srv import GetParameters
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import TransformException


def transform_values(transform):
    """Retain the actual exact-time transform alongside the raw cloud artifact."""
    t, q = transform.transform.translation, transform.transform.rotation
    return {'translation': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}


class FloorProbe(ClearanceProbe):
    """Reuse bounded exact-time reference histories, with independent floor records."""

    def __init__(self, node, directory):
        """Use advancing source time and retain one reconstructable cloud per phase."""
        node.set_parameters([Parameter('use_sim_time', value=True)])
        super().__init__(node, directory, filename='floor-samples.jsonl')
        self.directory = directory
        self.last_stamp = None
        self.last_clock = self.clock_at = None
        self.saved = set()
        self.phase_min_stamp = 0
        self.range_bounds = None

    def process(self):
        """Reject stale, duplicate, untransformable and unbracketed native scans."""
        now = time.monotonic()
        if self.range_bounds is None:
            return
        clock = self.node.get_clock().now().nanoseconds
        if self.last_clock is not None and clock < self.last_clock:
            self.pending.clear()
            self.last_stamp = None
        if clock > 0 and clock != self.last_clock:
            self.clock_at = now
        self.last_clock = clock
        while self.pending:
            msg, received = self.pending[0]
            stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if (now-received > 0.25 or not -0.05 <= (clock-stamp)/1e9 <= 0.5
                    or self.clock_at is None or now-self.clock_at > 0.5
                    or (self.last_stamp is not None and stamp <= self.last_stamp)):
                self.pending.popleft()
                self.unpaired += 1
                continue
            reference = self.history.at(stamp)
            try:
                source = Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME)
                body_tf = self.buffer.lookup_transform('base_link', msg.header.frame_id, source)
                footprint_tf = self.buffer.lookup_transform(
                    'base_footprint', msg.header.frame_id, source)
            except TransformException:
                body_tf = footprint_tf = None
            if reference is None or body_tf is None:
                return  # bounded by the original receipt deadline above
            self.pending.popleft()
            self.last_stamp = stamp
            try:
                raw = cloud_xyz(msg)
                minimum, maximum = self.range_bounds
                finite, saturated_min, saturated_max, valid_range = point_range_masks(
                    raw, minimum, maximum)
                points = raw[valid_range]
                local = self.apply(points, footprint_tf)
                plane = inspect_corridor(local)['ground_plane_z_ax_by_c']
                if plane is None:
                    raise ValueError('unobserved_flat_ground')
                coverage, grid, counts = scan_floor_coverage(local, plane)
            except (ValueError, TypeError, BufferError):
                self.invalid += 1
                continue
            position, quaternion, bracket = reference
            world = transform_xyz(self.apply(points, body_tf), position, quaternion)
            hole = ((world[:, 0] > 0.75) & (world[:, 0] < 1.25)
                    & (np.abs(world[:, 1]) < 0.25))
            record = {'stamp_ns': stamp, 'phase': self.phase,
                      'receipt_monotonic': received, 'pose_bracket_sec': bracket,
                      'frame_id': msg.header.frame_id,
                      'source_age_sec': (clock-stamp)/1e9,
                      'processing_receipt_age_wall_sec': now-received,
                      'tf_cloud_to_footprint': transform_values(footprint_tf),
                      'tf_cloud_to_body': transform_values(body_tf),
                      'width': msg.width, 'height': msg.height,
                      'finite_native_points': int(finite.sum()),
                      'range_saturated_min_points': int(saturated_min.sum()),
                      'range_saturated_max_points': int(saturated_max.sum()),
                      'valid_range_points': int(valid_range.sum()),
                      'min_native_range_m': float(np.linalg.norm(
                          raw[finite], axis=1).min()),
                      'estimated_plane': plane, 'truth_position': position.tolist(),
                      'truth_quaternion': quaternion.tolist(),
                      'hole_interior_ground_candidates': int((hole & (np.abs(world[:, 2])
                                                                      <= 0.025)).sum()),
                      **coverage}
            if self.phase not in self.saved and stamp >= self.phase_min_stamp:
                np.savez_compressed(self.directory / f'floor-{self.phase}.npz',
                                    raw_xyz=raw, local_xyz=local, world_xyz=world,
                                    finite_native_mask=finite,
                                    valid_range_mask=valid_range,
                                    plane=plane, grid=grid, floor_counts=counts,
                                    source_stamp_ns=stamp, width=msg.width, height=msg.height)
                self.saved.add(self.phase)
            with self.lock:
                self.records.append(record)
            self.output.write(json.dumps(record, allow_nan=False)+'\n')


def collect_floor(driver, phase, count=15):
    """Collect all qualifying scans in a fixed window; no selection by good coverage."""
    audit = driver.audit
    after = driver.get_clock().now().nanoseconds + 1000000000
    audit.phase_min_stamp = after
    audit.phase = phase
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        records = [r for r in audit.snapshot() if r['phase'] == phase and r['stamp_ns'] >= after]
        if len(records) >= count:
            return records[:count]
        time.sleep(0.05)
    raise RuntimeError(f'Insufficient fresh native floor scans for {phase}: {len(records)}')


def verify_sensor_model(driver, directory, profile):
    """Read the running publisher's model, not just the launch profile label."""
    client = driver.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    try:
        wait_until(client.service_is_ready, 5, 'No live robot model parameter service')
        future = client.call_async(GetParameters.Request(names=['robot_description']))
        wait_until(future.done, 5, 'Live robot model query timed out')
        values = future.result().values
        if len(values) != 1 or values[0].type != 4:
            raise RuntimeError('Live robot description is not a string')
        text = values[0].string_value
        root = ET.fromstring(text)
        sensors = root.findall("./gazebo[@reference='lidar_l1_link']/sensor[@name='lidar_l1']")
        if len(sensors) != 1:
            raise RuntimeError('Unexpected live L1 sensor count')
        sensor = sensors[0]
        minimum = float(sensor.findtext('./lidar/range/min'))
        expected = {'pinned': .8, 'l1_near': .05}[profile]
        if minimum != expected:
            raise RuntimeError(f'Live LiDAR minimum {minimum} differs from {expected}')
        maximum = float(sensor.findtext('./lidar/range/max'))
        point_range_masks(np.empty((0, 3)), minimum, maximum)
        (directory / 'live-robot-description.urdf').write_text(text)
        return {'passed': True, 'profile': profile, 'min_range_m': minimum,
                'max_range_m': maximum,
                'horizontal_samples': int(sensor.findtext('./lidar/scan/horizontal/samples')),
                'vertical_samples': int(sensor.findtext('./lidar/scan/vertical/samples')),
                'update_rate_hz': float(sensor.findtext('./update_rate')),
                'robot_description_sha256': hashlib.sha256(text.encode()).hexdigest()}
    except (ET.ParseError, ValueError, TypeError) as error:
        raise RuntimeError(f'Invalid live sensor model: {error}') from error
    finally:
        driver.destroy_client(client)


def run_floor_audit(driver, record, directory, sensor_profile):
    """Compare observed floor and a real finite-floor hole without requesting motion."""
    prepare_scenarios(driver, record)
    model = verify_sensor_model(driver, directory, sensor_profile)
    driver.audit.range_bounds = (model['min_range_m'], model['max_range_m'])
    record('live_sensor_model', model)
    driver.desired = 0.0
    started = time.monotonic()
    native = collect_floor(driver, 'flat')
    if any(r['hole_interior_ground_candidates'] < 3 for r in native):
        raise RuntimeError('Future hole area was not observed in the flat-floor control')
    flat = save_snapshot(driver, directory, 'floor-flat', take(driver)[0])
    record('floor_flat', {'passed': True, 'samples': native,
                          'support': flat[3]['support_evidence']})
    # Four independently rendered/collidable slabs leave a 0.6 x 0.6 m hole.
    # Feet remain on the left slab. These world dimensions are observer fixtures,
    # never a source of floor labels for the mapper or forward veto.
    rectangles = [(-5, 0.7, -5, 5), (1.3, 5, -5, 5),
                  (0.7, 1.3, -5, -0.3), (0.7, 1.3, 0.3, 5)]
    links = []
    for i, (x0, x1, y0, y1) in enumerate(rectangles):
        geometry = f'<geometry><box><size>{x1-x0} {y1-y0} 0.2</size></box></geometry>'
        links.append(f'<link name="floor_{i}"><pose>{(x0+x1)/2} {(y0+y1)/2} -0.1 '
                     f'0 0 0</pose><collision name="collision">{geometry}</collision>'
                     f'<visual name="visual">{geometry}</visual></link>')
    sdf = ('<sdf version="1.8"><model name="floor_audit_slabs"><static>true</static>'
           + ''.join(links) + '</model></sdf>')
    (directory / 'floor-hole.sdf').write_text(sdf+'\n')
    service('create', 'gz.msgs.EntityFactory', f'sdf: {json.dumps(sdf)} allow_renaming: false')
    service('remove', 'gz.msgs.Entity', 'name: "ground_plane" type: MODEL')
    samples = collect_floor(driver, 'hole')
    # Wait out the unchanged 2 s wall-time floor TTL after removing the plane.
    time.sleep(2.2)
    pair = take(driver)
    hole_map = save_snapshot(driver, directory, 'floor-hole', pair[0])
    data, origin, resolution, status = hole_map
    y, x = np.indices(data.shape)
    centers = origin + np.stack((x+0.5, y+0.5), axis=-1)*resolution
    # World and odom are not assumed identical: align via the paired body reference.
    _, (position, quaternion, _) = pair
    pose = status['pose_odom']
    from guidedog_perception.clearance_metrics import yaw_of
    delta_yaw = yaw_of(quaternion)-pose[2]
    c, s = np.cos(delta_yaw), np.sin(delta_yaw)
    rotation = np.array([[c, -s], [s, c]])
    centers = (centers-np.array(pose[:2])) @ rotation.T + position[:2]
    inside = ((centers[..., 0] > 0.75+resolution/2)
              & (centers[..., 0] < 1.25-resolution/2)
              & (np.abs(centers[..., 1]) < 0.25-resolution/2))
    false_free = int(((data == 0) & inside).sum())
    if (not inside.any() or false_free
            or any(r['hole_interior_ground_candidates'] for r in samples)):
        raise RuntimeError('Floor-hole negative control did not remain unobserved')
    record('floor_hole', {'passed': True, 'hole_world_bounds_m': [0.7, 1.3, -0.3, 0.3],
                          'interior_cells': int(inside.sum()), 'false_free_cells': false_free,
                          'samples': samples, 'support': status['support_evidence']})
    driver.relay_enabled = False
    try:
        wait_mapping(driver, lambda s: not s[3]['support_evidence']['map_healthy'],
                     5, 'Support diagnostic did not invalidate on scan loss')
        time.sleep(2.2)
        expired = driver.mapping()
        if expired is None or expired[3]['support_evidence']['envelope_observed']:
            raise RuntimeError('Expired floor evidence was approved')
        record('floor_dropout', {'passed': True, 'support': expired[3]['support_evidence']})
    finally:
        driver.relay_enabled = True
    wait_mapping(driver, lambda s: s[3]['healthy'], 10, 'Floor observations did not recover')
    _, _, poses, outputs = driver.snapshot()
    if any(abs(speed) > 1e-9 for stamp, speed in outputs if stamp >= started):
        raise RuntimeError('Stationary floor audit emitted a nonzero checked command')
    heights = [p[3] for p in poses if p[0] >= started]
    if not heights or min(heights) < 0.15 or max(heights) > 0.40:
        raise RuntimeError('Robot did not remain standing on the finite floor')
    record('floor_audit_scope', {'passed': True, 'stationary_checked_commands_only': True,
                                 'body_height_range_m': [min(heights), max(heights)],
                                 'planning_ready': False,
                                 'note': 'Audit completion is not successful planning readiness'})
