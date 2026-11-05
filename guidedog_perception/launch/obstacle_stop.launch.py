# SPDX-License-Identifier: Apache-2.0
"""Protected forward-motion demo using the pinned simulator unchanged."""

from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Isolate controller commands from keyboard requests and add two veto stages."""
    package = Path(get_package_share_directory('guidedog_perception'))
    directory = str(Path.home() / '.ros/guidedog/reports' /
                    datetime.now(timezone.utc).strftime('stop_%Y%m%dT%H%M%S_%fZ'))
    defaults = {
        'report_dir': directory,
        'world': str(package / 'worlds/obstacle_stop.sdf'),
        'lidar_topic': '/unitree_lidar/points',
        'lidar_reliability': 'reliable',
        'record_gazebo': 'false',
        'headless': 'false',
        'gait_profile': 'low_lift_calibrated',
        'sensor_profile': 'pinned',
        'self_filter_mode': 'geometry',
        'lidar_min_range': PythonExpression([
            "0.05 if '", LaunchConfiguration('sensor_profile'), "' == 'l1_near' else 0.8"]),
        'lidar_max_range': '30.0',
        'enable_rgb': 'false',
        'enable_local_map': 'false',
        'max_speed': '0.05', 'min_speed': '0.05',
        'stop_distance': '0.80', 'scan_timeout': '1.50',
    }
    declarations = [DeclareLaunchArgument(key, default_value=value)
                    for key, value in defaults.items()]
    simulation = GroupAction(actions=[
        SetRemap(src='/cmd_vel/smooth', dst='/guidedog/cmd_vel_checked'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/simulation.launch.py')),
            launch_arguments={
                'report_dir': LaunchConfiguration('report_dir'),
                'world': LaunchConfiguration('world'),
                'record_gazebo': LaunchConfiguration('record_gazebo'),
                'headless': LaunchConfiguration('headless'),
                'gait_profile': LaunchConfiguration('gait_profile'),
                'sensor_profile': LaunchConfiguration('sensor_profile'),
                'enable_rgb': LaunchConfiguration('enable_rgb'),
                'lidar_reliability': LaunchConfiguration('lidar_reliability'),
            }.items()),
    ])
    guard = Node(
        package='guidedog_perception', executable='lidar_stop', output='screen',
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
        parameters=[{
            'use_sim_time': True,
            'report_dir': ParameterValue(LaunchConfiguration('report_dir'), value_type=str),
            'lidar_topic': ParameterValue(LaunchConfiguration('lidar_topic'), value_type=str),
            'lidar_reliability': ParameterValue(
                LaunchConfiguration('lidar_reliability'), value_type=str),
            **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
               for name in ('max_speed', 'min_speed', 'stop_distance', 'scan_timeout',
                            'lidar_min_range', 'lidar_max_range')},
        }])
    watchdog = Node(
        package='guidedog_perception', executable='velocity_watchdog', output='screen',
        parameters=[{
            'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
        }])
    contacts = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/guidedog/obstacle_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'])
    mapper = Node(
        package='guidedog_perception', executable='local_mapper', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_local_map')),
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
        parameters=[{
            'use_sim_time': True,
            'lidar_topic': ParameterValue(LaunchConfiguration('lidar_topic'), value_type=str),
            'self_filter_mode': ParameterValue(
                LaunchConfiguration('self_filter_mode'), value_type=str),
            **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
               for name in ('lidar_min_range', 'lidar_max_range')},
            'report_dir': ParameterValue(
                [LaunchConfiguration('report_dir'), '/mapping'], value_type=str),
        }])
    return LaunchDescription(declarations + [simulation, guard, watchdog, contacts, mapper])
