# SPDX-License-Identifier: Apache-2.0
"""Protected forward-motion demo using the pinned simulator unchanged."""

from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
               for name in ('max_speed', 'min_speed', 'stop_distance', 'scan_timeout')},
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
            'report_dir': ParameterValue(
                [LaunchConfiguration('report_dir'), '/mapping'], value_type=str),
        }])
    return LaunchDescription(declarations + [simulation, guard, watchdog, contacts, mapper])
