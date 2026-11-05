# SPDX-License-Identifier: Apache-2.0
"""Explicit simulator-only CHAMP profiles without editing the pinned dependency."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


PROFILES = {'stock': {}, 'low_lift': {'gait.swing_height': 0.015},
            'low_lift_calibrated': {'gait.swing_height': 0.015}}
ODOM_SCALERS = {'low_lift_calibrated': 1.60}


def model_command():
    """Use exactly the upstream model arguments for both CHAMP nodes."""
    return Command([
        'xacro ', LaunchConfiguration('unitree_go2_description_path'),
        ' robot_controllers:=', LaunchConfiguration('ros_control_file'),
        ' command_interface:=', LaunchConfiguration('command_interface'),
        ' disable_camera:=', LaunchConfiguration('disable_camera'),
        ' disable_d455:=', LaunchConfiguration('disable_d455'),
        ' disable_lidar_l1:=', LaunchConfiguration('disable_lidar_l1'),
        ' disable_velodyne_lidar:=', LaunchConfiguration('disable_velodyne_lidar')])


def profiled_controller(profile):
    """Keep the pinned controller, effort interface, geometry and all other gait values."""
    simulator = Path(get_package_share_directory('unitree_go2_sim'))
    return Node(
        package='champ_base', executable='quadruped_controller_node', output='screen',
        parameters=[{'gazebo': True, 'publish_joint_states': True,
                     'publish_joint_control': True, 'publish_foot_contacts': False,
                     'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
                     'urdf': model_command()},
                    str(simulator / 'config/joints/joints.yaml'),
                    str(simulator / 'config/links/links.yaml'),
                    str(simulator / 'config/gait/gait.yaml'),
                    {'hardware_connected': False, 'close_loop_odom': True}, PROFILES[profile]],
        remappings=[('/cmd_vel/smooth', '/cmd_vel')])


def profiled_estimator(profile):
    """Apply the independently calibrated scalar at construction, never fit it online."""
    simulator = Path(get_package_share_directory('unitree_go2_sim'))
    return Node(
        package='champ_base', executable='state_estimation_node', output='screen',
        parameters=[{'orientation_from_imu': True, 'urdf': model_command()},
                    str(simulator / 'config/joints/joints.yaml'),
                    str(simulator / 'config/links/links.yaml'),
                    str(simulator / 'config/gait/gait.yaml'),
                    {'gait.odom_scaler': ODOM_SCALERS[profile]}])


def apply_profile(description, profile):
    """Change exactly the gait-generator node, rejecting an unknown upstream layout."""
    if profile not in PROFILES:
        raise ValueError('unknown_gait_profile')
    if profile == 'stock':
        return description
    context = LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    matches = [e for e in description.entities if isinstance(e, Node)
               and text(e.node_package) == 'champ_base'
               and text(e.node_executable) == 'quadruped_controller_node']
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one pinned CHAMP gait-generator node')
    replacements = {matches[0]: profiled_controller(profile)}
    if profile in ODOM_SCALERS:
        estimators = [e for e in description.entities if isinstance(e, Node)
                      and text(e.node_package) == 'champ_base'
                      and text(e.node_executable) == 'state_estimation_node']
        if len(estimators) != 1:
            raise RuntimeError('Expected exactly one pinned CHAMP state estimator')
        replacements[estimators[0]] = profiled_estimator(profile)
    return LaunchDescription([replacements.get(e, e) for e in description.entities])
