# SPDX-License-Identifier: Apache-2.0
"""Observe the actual published model and gather complete exact-time self transforms."""

import hashlib
from xml.etree.ElementTree import ParseError

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.robot_body import RobotBody
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def stamped_body_transforms(buffer, model, source_frame, source_time):
    """Ask TF for every link at the requested time; do not request latest transforms."""
    transforms = {}
    for name in model.links:
        tf = buffer.lookup_transform(name, source_frame, source_time).transform
        t, q = tf.translation, tf.rotation
        transforms[name] = ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
    return transforms


class BodyObserver:
    """Invalidate old geometry on any model change, including a malformed update."""

    def __init__(self, node):
        """Use the transient model topic; missing description never means an empty robot."""
        self.model = None
        self.epoch = 0
        self.digest = None
        self.error = 'waiting_for_robot_description'
        self.mesh_root = (get_package_share_directory('unitree_go2_description')+'/meshes')
        self.subscription = node.create_subscription(
            String, '/robot_description', self.on_model,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_model(self, msg):
        """Parse one model once; changed or unsupported assets cannot refresh floor."""
        digest = hashlib.sha256(msg.data.encode()).hexdigest()
        if digest == self.digest:
            return
        self.digest, self.model = digest, None
        self.epoch += 1
        try:
            self.model = RobotBody(msg.data, self.mesh_root)
            self.error = None
        except (ValueError, TypeError, OSError, ParseError) as error:
            self.error = 'invalid_robot_model:'+str(error)
