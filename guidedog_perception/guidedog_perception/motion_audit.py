# SPDX-License-Identifier: Apache-2.0
"""Record commands and measured motion without changing the command path."""

from collections import Counter
import json
import time

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory


def vector(value):
    """Encode an explicitly named three-axis vector."""
    return [value.x, value.y, value.z]


def quaternion(value):
    """Keep the measured quaternion without a heading correction or reset."""
    return [value.x, value.y, value.z, value.w]


class MotionAudit:
    """Keep bounded-duration telemetry with original stamps and observer receipt time."""

    def __init__(self, node, directory):
        """Observe real ROS streams; never subscribe to synthetic substitute poses."""
        self.output = (directory / 'motion-audit.jsonl').open('w')
        self.started = time.monotonic()
        self.counts = Counter()
        self.subscriptions = []
        self.parameter_clients = {}
        self.requested_parameters = set()
        profiles = {
            'quadruped_controller_node': ['use_sim_time', 'gait.swing_height',
                                          'gait.stance_depth', 'gait.stance_duration',
                                          'gait.nominal_height', 'loop_rate'],
            'controller_manager': ['use_sim_time', 'update_rate'],
            'state_estimation_node': ['use_sim_time', 'gait.odom_scaler'],
            'joint_group_effort_controller': ['gains.lf_hip_joint.p', 'gains.lf_hip_joint.d',
                                              'state_publish_rate'],
        }
        for name, names in profiles.items():
            self.parameter_clients[name] = (
                node.create_client(GetParameters, f'/{name}/get_parameters'), names)
        self.parameter_timer = node.create_timer(
            0.5, self.parameters, clock=node.steady_clock)
        for topic in ('/guidedog/cmd_vel_request', '/guidedog/cmd_vel_candidate',
                      '/guidedog/cmd_vel_checked'):
            self.subscriptions.append(node.create_subscription(
                Twist, topic, lambda msg, name=topic: self.command(name, msg), 10))
        for topic in ('/odom', '/odom/ground_truth'):
            self.subscriptions.append(node.create_subscription(
                Odometry, topic, lambda msg, name=topic: self.odom(name, msg),
                qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(
            Imu, '/imu/data', self.imu, qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(
            JointTrajectoryControllerState, '/joint_group_effort_controller/controller_state',
            self.joints, qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(
            JointTrajectory, '/joint_group_effort_controller/joint_trajectory',
            self.trajectory, 10))

    def write(self, topic, values, header=None):
        """Write one callback observation, keeping stamp-less commands explicitly so."""
        now = time.monotonic()
        self.counts[topic] += 1
        stamp = None if header is None else header.stamp.sec*1000000000+header.stamp.nanosec
        self.output.write(json.dumps({'topic': topic, 'receipt_wall_sec': now-self.started,
                                      'source_stamp_ns': stamp, **values}, allow_nan=False)+'\n')

    def command(self, topic, msg):
        """Retain all six command components rather than assuming angular zero."""
        self.write(topic, {'linear': vector(msg.linear), 'angular': vector(msg.angular)})

    def parameters(self):
        """Record actual constructor parameters instead of trusting a profile label."""
        for name, (client, names) in self.parameter_clients.items():
            if name in self.requested_parameters or not client.service_is_ready():
                continue
            self.requested_parameters.add(name)
            request = GetParameters.Request(names=names)

            def received(future, node_name=name, keys=names):
                fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value',
                          4: 'string_value'}
                values = {k: getattr(v, fields[v.type]) if v.type in fields else None
                          for k, v in zip(keys, future.result().values)}
                values['requested_names'] = keys
                values['response_count'] = len(future.result().values)
                self.write('/'+node_name+'/parameters', values)

            client.call_async(request).add_done_callback(received)

    def odom(self, topic, msg):
        """Record the estimator and independent truth using their own source frames."""
        pose, twist = msg.pose.pose, msg.twist.twist
        self.write(topic, {'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
                           'position': vector(pose.position),
                           'quaternion': quaternion(pose.orientation),
                           'linear': vector(twist.linear), 'angular': vector(twist.angular)},
                   msg.header)

    def imu(self, msg):
        """Keep the orientation and angular velocity, not an inferred controller target."""
        self.write('/imu/data', {
            'frame': msg.header.frame_id, 'quaternion': quaternion(msg.orientation),
            'angular': vector(msg.angular_velocity)}, msg.header)

    def joints(self, msg):
        """Capture set points, tracking errors and actual effort-controller output."""
        values = {'joint_names': list(msg.joint_names)}
        for name in ('reference', 'feedback', 'error', 'output'):
            point = getattr(msg, name)
            values[name] = {k: list(getattr(point, k))
                            for k in ('positions', 'velocities', 'effort')}
        self.write('/joint_group_effort_controller/controller_state', values, msg.header)

    def trajectory(self, msg):
        """Record the real controller target and its advertised interpolation horizon."""
        values = {'joint_names': list(msg.joint_names), 'points': []}
        for p in msg.points:
            values['points'].append({'positions': list(p.positions),
                                     'time_from_start_sec': p.time_from_start.sec
                                     + p.time_from_start.nanosec/1e9})
        self.write('/joint_group_effort_controller/joint_trajectory', values, msg.header)

    def close(self, directory):
        """Close after the executor stops so the audit is a complete immutable record."""
        self.output.close()
        (directory / 'motion-audit-counts.json').write_text(
            json.dumps(dict(self.counts), indent=2)+'\n')
