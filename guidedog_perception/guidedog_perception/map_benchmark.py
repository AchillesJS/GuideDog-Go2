# SPDX-License-Identifier: Apache-2.0
"""Observe actual Gazebo mapping, motion, blackout, expiry and box removal."""

from collections import deque
import json
import math
import time

from guidedog_perception.local_mapper import write_svg
from guidedog_perception.stop_benchmark import (
    displacement, prepare_scenarios, ScenarioDriver, settled_stop, wait_until)
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MappingDriver(ScenarioDriver):
    """Use ground truth only for acceptance measurements, never the mapper's input."""

    def __init__(self):
        """Observe the mapper independently of the established command/scan relay."""
        super().__init__()
        self.maps = deque(maxlen=10)
        self.map_status = {}
        self.create_subscription(OccupancyGrid, '/guidedog/local_map', self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/guidedog/local_map_status', self.on_map_status, 1)

    def on_map(self, msg):
        """Bound stored snapshots and record the advertised source observation stamp."""
        with self.lock:
            self.maps.append(msg)

    def on_map_status(self, msg):
        """Keep the observation status separate from the motion veto status."""
        with self.lock:
            self.map_status = json.loads(msg.data)

    def mapping(self):
        """Pair the current map and status by source time, without invented restamping."""
        with self.lock:
            status = dict(self.map_status)
            for msg in reversed(self.maps):
                stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                if stamp == status.get('source_stamp_ns'):
                    data = np.array(msg.data, dtype=np.int8).reshape(
                        msg.info.height, msg.info.width)
                    origin = [msg.info.origin.position.x, msg.info.origin.position.y]
                    return data, np.array(origin), msg.info.resolution, status
        return None


def world_centers(snapshot):
    """Return the world coordinates encoded by cell indices and grid origin."""
    data, origin, resolution, _ = snapshot
    y, x = np.indices(data.shape)
    return np.stack((origin[0]+(x+0.5)*resolution,
                     origin[1]+(y+0.5)*resolution), axis=-1)


def box_observation(snapshot, box):
    """Measure occupied-cell boundary error against the known static rectangle."""
    data, _, _, _ = snapshot
    delta = world_centers(snapshot)-box[:2]
    c, s = math.cos(box[2]), math.sin(box[2])
    xy = np.stack((c*delta[..., 0]+s*delta[..., 1],
                   -s*delta[..., 0]+c*delta[..., 1]), axis=-1)
    selected = (data == 100) & (np.abs(xy[..., 0]) < 0.5) & (np.abs(xy[..., 1]) < 0.75)
    points = np.abs(xy[selected])
    outside = np.maximum(points-[0.25, 0.5], 0)
    inside = np.minimum(0.25-points[:, 0], 0.5-points[:, 1])
    errors = np.where(inside >= 0, inside, np.linalg.norm(outside, axis=1))
    return {'occupied_cells_near_box': int(selected.sum()),
            'box_boundary_error_p95_m': float(np.percentile(errors, 95)) if len(errors) else None}


def save_snapshot(driver, directory, name):
    """Keep each measured map and its picture, not just the final expired snapshot."""
    snapshot = driver.mapping()
    data, origin, resolution, status = snapshot
    (directory / f'map-{name}.json').write_text(json.dumps({
        'status': status, 'occupancy': data.tolist(), 'origin': origin.tolist(),
        'resolution': resolution}, indent=2)+'\n')
    robot = (np.array(status['pose_odom'][:2])-origin)/resolution
    write_svg(directory / f'map-{name}.svg', {'occupancy': data}, resolution,
              [*robot.tolist(), status['pose_odom'][2]])
    return snapshot


def run_mapping(driver, record, directory):
    """Test predefined criteria with real scans and explicit zero-command intervals."""
    limits = {'max_box_boundary_error_p95_m': 0.20, 'min_obstacle_cells': 3,
              'max_update_cost_p95_ms': 100.0, 'max_cell_storage_bytes': 1024*1024,
              'min_motion_m': 0.05, 'simulation_flat_ground_only': True}
    record('mapping_limits', {'passed': True, **limits})
    prepare_scenarios(driver, record)
    wait_until(lambda: driver.mapping() is not None
               and driver.mapping()[3]['healthy']
               and driver.mapping()[3]['accepted'] >= 10, 20, 'Mapper did not become ready')
    free = driver.mapping()
    if (free[0] == 0).sum() < 20 or (free[0] == -1).sum() < 100:
        raise RuntimeError('Initial map has insufficient observed floor or unknown space')
    truth = driver.snapshot()[0]
    estimated = free[3]['pose_odom']
    angle = estimated[2]-truth[4]
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array(estimated[:2])-rotation @ np.array(truth[1:3])
    box = driver.place_obstacle(1.75, 0)
    box_reference = np.r_[rotation @ np.array(box[:2])+translation, box[2]+angle]
    wait_until(lambda: box_observation(driver.mapping(), box_reference)[
        'occupied_cells_near_box'] >= limits['min_obstacle_cells'], 10,
        'No occupied cells at the actual box')
    # Let the initially observed floor behind the newly placed box expire.
    time.sleep(2.5)
    before = save_snapshot(driver, directory, 'before-motion')
    first = box_observation(before, box_reference)
    if first['box_boundary_error_p95_m'] > limits['max_box_boundary_error_p95_m']:
        raise RuntimeError(f'Initial box localization error: {first}')
    record('box_localization', {
        'passed': True, **first, 'initial_gt_to_odom_rotation_rad': angle,
        'unknown_cells': int((before[0] == -1).sum()),
        'free_cells': int((before[0] == 0).sum())})
    delta = world_centers(before)-box_reference[:2]
    c, s = math.cos(box_reference[2]), math.sin(box_reference[2])
    forward, side = c*delta[..., 0]+s*delta[..., 1], -s*delta[..., 0]+c*delta[..., 1]
    shadow = (forward > 0.45) & (forward < 1.05) & (np.abs(side) < 0.15)
    unknown_fraction = float((before[0][shadow] == -1).mean())
    if not np.any(shadow) or unknown_fraction < 0.80:
        raise RuntimeError(f'Occluded ground was incorrectly labelled: {unknown_fraction=}')
    record('occluded_ground', {'passed': True, 'unknown_fraction': unknown_fraction,
                               'sampled_cells': int(shadow.sum())})
    pose_before = driver.snapshot()[0]
    driver.desired = 0.05
    wait_until(lambda: displacement(pose_before, driver.snapshot()[0]) >= 0.10
               or driver.snapshot()[1].get('reason') == 'obstacle', 18,
               'No measured motion toward the box')
    driver.desired = 0.0
    stopping = settled_stop(driver, time.monotonic())
    after = save_snapshot(driver, directory, 'after-motion')
    measured = box_observation(after, box_reference)
    progress = displacement(pose_before, driver.snapshot()[0])
    passed = (progress >= limits['min_motion_m']
              and measured['occupied_cells_near_box'] >= limits['min_obstacle_cells']
              and measured['box_boundary_error_p95_m'] <= limits['max_box_boundary_error_p95_m']
              and after[3]['rolls'] > before[3]['rolls'] and not driver.contacts)
    record('motion_and_roll', {
        'passed': passed, 'ground_truth_progress_m': progress,
        'rolls_before': before[3]['rolls'], 'rolls_after': after[3]['rolls'],
        **measured, **stopping})
    if not passed:
        raise RuntimeError('Moving grid failed its predefined localization/progress criteria')
    driver.relay_enabled = False
    time.sleep(0.9)
    held = save_snapshot(driver, directory, 'blackout-held')
    held_values = box_observation(held, box_reference)
    if held[3]['healthy'] or held_values['occupied_cells_near_box'] < limits['min_obstacle_cells']:
        raise RuntimeError('Blackout did not report stale data and retain the obstacle')
    stamp = held[3]['source_stamp_ns']
    record('short_blackout', {'passed': True, **held_values, 'healthy': False,
                              'source_stamp_ns': stamp})
    wait_until(lambda: (driver.mapping()[0] == -1).all(), 7,
               'Missing observations did not expire to unknown')
    expired = save_snapshot(driver, directory, 'blackout-expired')
    if expired[3]['source_stamp_ns'] != stamp:
        raise RuntimeError('Expired map was falsely stamped as a new observation')
    record('expiry_to_unknown', {'passed': True, 'source_stamp_unchanged': True,
                                 'unknown_cells': int((expired[0] == -1).sum())})
    driver.relay_enabled = True
    wait_until(lambda: driver.mapping()[3]['healthy']
               and box_observation(driver.mapping(), box_reference)[
                   'occupied_cells_near_box'] >= limits['min_obstacle_cells'], 10,
               'Map did not recover from restored real scans')
    restored = save_snapshot(driver, directory, 'restored')
    previous_hits = world_centers(restored)[restored[0] == 100]
    driver.place_obstacle(20, 20)

    def observed_clearing():
        current = driver.mapping()
        cells = np.floor((previous_hits-current[1])/current[2]).astype(int)
        valid = ((cells >= 0) & (cells < current[0].shape[0])).all(axis=1)
        cells = cells[valid]
        return int((current[0][cells[:, 1], cells[:, 0]] == 0).sum())

    wait_until(lambda: observed_clearing() >= 1, 8,
               'No former obstacle cell was cleared by newly observed ground')
    cleared = save_snapshot(driver, directory, 'box-removed')
    record('fresh_ground_clearing', {
        'passed': True, 'formerly_occupied_now_observed_free': observed_clearing()})
    status = cleared[3]
    passed = (status['cell_array_bytes'] <= limits['max_cell_storage_bytes']
              and status['update_cost_p95_ms'] <= limits['max_update_cost_p95_ms'])
    record('mapping_budget', {'passed': passed, 'cell_array_bytes': status['cell_array_bytes'],
                              'update_cost_p95_ms': status['update_cost_p95_ms'],
                              'accepted_clouds': status['accepted'],
                              'robot_obstacle_contacts': len(driver.contacts)})
    if not passed or driver.contacts:
        raise RuntimeError('Mapping budget or contact criterion failed')
