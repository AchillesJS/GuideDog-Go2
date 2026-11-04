# SPDX-License-Identifier: Apache-2.0
"""Predefined reference-error and stopping criteria across actual Gazebo approaches."""

import json
import math
import subprocess
import time

from guidedog_perception.clearance_metrics import box_clearance
from guidedog_perception.stop_benchmark import (
    displacement, prepare_scenarios, settled_stop, wait_until)
import numpy as np


def replace_box(driver, width):
    """Resize only the test fixture; preserve a contact sensor on every new box."""
    removed = subprocess.run(
        ['gz', 'service', '-s', '/world/guidedog_stop/remove',
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', 'name: "test_obstacle" type: MODEL'],
        capture_output=True, text=True, timeout=6)
    if removed.returncode or 'true' not in removed.stdout:
        raise RuntimeError('Could not remove the previous test box')
    time.sleep(0.25)
    pose = driver.snapshot()[0]
    sdf = (f'<sdf version="1.8"><model name="test_obstacle"><static>true</static>'
           f'<pose>{pose[1]+20} {pose[2]+20} 0.5 0 0 0</pose><link name="box">'
           f'<collision name="collision"><geometry><box><size>0.5 {width} 1</size>'
           '</box></geometry></collision><sensor name="obstacle_contact" type="contact">'
           '<contact><collision>collision</collision><topic>/guidedog/obstacle_contacts</topic>'
           '</contact></sensor>'
           f'<visual name="visual"><geometry><box><size>0.5 {width} 1</size></box></geometry>'
           '</visual></link></model></sdf>')
    created = subprocess.run(
        ['gz', 'service', '-s', '/world/guidedog_stop/create',
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', f'sdf: {json.dumps(sdf)}'],
        capture_output=True, text=True, timeout=6)
    if created.returncode or 'true' not in created.stdout:
        raise RuntimeError(f'Could not create test box: {created.stdout} {created.stderr}')
    before = driver.probe_contacts
    driver.verify_contact_sensor((pose[1]+20, pose[2]+20, 0))
    wait_until(lambda: driver.probe_contacts > before, 5,
               'Resized box contact positive control failed')


def run_matrix(driver, record):
    """Record every attempt and reject any predeclared accuracy or stopping failure."""
    prepare_scenarios(driver, record)
    limits = {'min_final_corridor_clearance_m': 0.70,
              'max_frame_error_p95_m': 0.05, 'max_sampling_error_p95_m': 0.05,
              'max_post_zero_travel_m': 0.03, 'min_reference_fraction': 0.80}
    record('matrix_limits', {'passed': True, **limits, 'simulation_only': True})
    cases = [('straight', 1.0, 0, 0.05), ('angled_left', 1.0, 15, 0.05),
             ('angled_right', 1.0, -15, 0.05), ('narrow', 0.4, 0, 0.05),
             ('wide', 1.4, 0, 0.05), ('below_supported_speed', 1.0, 0, 0.03)]
    failures = []
    for name, width, degrees, speed in cases:
        driver.desired = 0.0
        driver.audit.phase = 'setup'
        try:
            replace_box(driver, width)
            wait_until(lambda: driver.snapshot()[1].get('reason') == 'idle', 5,
                       'Guard did not rearm with the distant box')
            if speed < driver.snapshot()[1]['parameters']['min_speed']:
                before = driver.snapshot()[0]
                started = time.monotonic()
                driver.desired = speed
                wait_until(lambda: driver.snapshot()[1].get('reason') ==
                           'unsupported_or_invalid_command', 1,
                           'Unsupported low speed was not rejected')
                time.sleep(0.5)
                final, _, _, outputs = driver.snapshot()
                emitted = [v for t, v in outputs if t >= started]
                progress = abs(displacement(before, final))
                passed = bool(emitted) and all(v == 0 for v in emitted) and progress < 0.01
                record(name, {'passed': passed, 'request_speed_mps': speed,
                              'rejected': True, 'absolute_forward_motion_m': progress})
                driver.desired = 0.0
                if not passed:
                    failures.append(name)
                continue
            angle = math.radians(degrees)
            distance = (0.45 + 0.8 + 0.25*abs(math.cos(angle))
                        + width/2*abs(math.sin(angle)) + 0.22)
            box = driver.place_obstacle(distance, 0, angle)
            time.sleep(0.8)
            driver.audit.set_box(box, (0.5, width, 1.0), name)
            received_before = driver.audit.received
            wait_until(lambda: driver.snapshot()[1].get('reason') == 'idle', 5,
                       'Guard not ready before the approach')
            before = driver.snapshot()[0]
            driver.desired = speed

            def stopped():
                pose, status, _, _ = driver.snapshot()
                gap, _ = box_clearance(box, (0.5, width, 1), pose[1:4], pose[4])
                if gap is None or gap < 0.35:
                    driver.desired = 0.0
                    raise RuntimeError('Approach left corridor or crossed intervention gap')
                if status.get('reason') == 'obstacle':
                    return True
                if status.get('reason') not in ('moving', 'idle'):
                    driver.desired = 0.0
                    raise RuntimeError(f'Non-obstacle stop: {status.get("reason")}')
                return False

            wait_until(stopped, 35, 'No obstacle stop within the approach deadline')
            detected = driver.snapshot()[1]
            stopping = settled_stop(driver, detected['received_monotonic'])
            driver.desired = 0.0
            final = driver.snapshot()[0]
            gap, _ = box_clearance(box, (0.5, width, 1), final[1:4], final[4])
            records = [r for r in driver.audit.snapshot() if r['phase'] == name]
            paired = [r for r in records if all(r.get(k) is not None for k in (
                'tf_clearance_m', 'truth_cloud_clearance_m', 'box_corridor_clearance_m'))]
            selected = [r for r in paired if r['stamp_ns'] == detected['scan_stamp_ns']]
            if len(paired) < 8 or not selected:
                raise RuntimeError('Insufficient synchronized clearance evidence')
            fraction = len(paired) / max(1, driver.audit.received - received_before)
            frame = [abs(r['tf_clearance_m'] - r['truth_cloud_clearance_m']) for r in paired]
            sampling = [abs(r['truth_cloud_clearance_m'] - r['box_corridor_clearance_m'])
                        for r in paired]
            frame95 = float(np.percentile(frame, 95))
            sampling95 = float(np.percentile(sampling, 95))
            travel_ok = stopping['post_zero_forward_travel_m'] <= limits['max_post_zero_travel_m']
            passed = (gap is not None and gap >= limits['min_final_corridor_clearance_m']
                      and frame95 <= limits['max_frame_error_p95_m']
                      and sampling95 <= limits['max_sampling_error_p95_m']
                      and fraction >= limits['min_reference_fraction']
                      and travel_ok
                      and displacement(before, final) >= 0.02 and not driver.contacts)
            values = {'passed': passed, 'width_m': width, 'box_angle_deg': degrees,
                      'request_speed_mps': speed,
                      'forward_progress_m': displacement(before, final),
                      'final_corridor_clearance_m': gap, 'frame_error_p95_m': frame95,
                      'sampling_error_p95_m': sampling95, 'reference_fraction': fraction,
                      'detection_sample': selected[-1], **stopping}
            record(name, values)
            if not passed:
                failures.append(name)
        except (RuntimeError, subprocess.SubprocessError) as error:
            driver.desired = 0.0
            record(name, {'passed': False, 'error': str(error),
                          'guard_status': driver.snapshot()[1]})
            failures.append(name)
            time.sleep(1.0)
        if driver.contacts:
            raise RuntimeError('Contact recorded during clearance matrix')
    if failures:
        raise RuntimeError(f'Clearance matrix failed: {failures}')
