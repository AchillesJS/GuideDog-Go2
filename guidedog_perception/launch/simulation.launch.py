# SPDX-License-Identifier: Apache-2.0
"""Launch the pinned Go2 simulation and GuideDog sensor diagnostics together."""

from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from guidedog_perception.sim_lifecycle import stop_when_gazebo_exits
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_report_directory(context):
    """Reject reused report directories before starting any simulation process."""
    directory = Path(LaunchConfiguration('report_dir').perform(context)).expanduser()
    if directory.exists() and any(directory.iterdir()):
        raise ValueError(f'Report directory is not empty: {directory}')
    directory.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    """Compose the existing simulation and an independent sensor observer."""
    report_dir = str(Path.home() / '.ros/guidedog/reports' /
                     datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    simulator = get_package_share_directory('unitree_go2_sim')
    description = get_package_share_directory('unitree_go2_description')
    package = get_package_share_directory('guidedog_perception')
    defaults = {
        'report_dir': report_dir,
        'start_simulator': 'true',
        'headless': 'false',
        'gait_profile': 'stock',
        'record_gazebo': 'true',
        'enable_rgb': 'true',
        'lidar_reliability': 'reliable',
        'probe_duration_sec': '60.0',
        'duration_sec': '0.0',
        'report_period_sec': '5.0',
        'stale_after_sec': '2.0',
        'window_sec': '10.0',
        'world': str(Path(description) / 'worlds/default.sdf'),
    }
    declarations = [DeclareLaunchArgument(name, default_value=value)
                    for name, value in defaults.items()]
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PythonExpression([
            "'", str(Path(package) / 'launch/profiled_simulation.launch.py'),
            "' if '", LaunchConfiguration('headless'),
            "'.lower() in ('true', '1', 'yes') or '", LaunchConfiguration('gait_profile'),
            "' != 'stock' else '",
            str(Path(simulator) / 'launch/unitree_go2_launch.py'), "'",
        ])),
        condition=IfCondition(LaunchConfiguration('start_simulator')),
        launch_arguments={
            'world': LaunchConfiguration('world'), 'rviz': 'false',
            'use_sim_time': 'true', 'disable_d455': 'true',
            'disable_velodyne_lidar': 'true',
            'disable_camera': PythonExpression([
                "'", LaunchConfiguration('enable_rgb'), "'.lower() not in ('true', '1', 'yes')"]),
            'disable_lidar_l1': 'false',
        }.items(),
    )
    monitor = Node(
        package='guidedog_perception', executable='sensor_monitor', output='screen',
        parameters=[{
            'use_sim_time': True,
            'rgb_enabled': ParameterValue(LaunchConfiguration('enable_rgb'), value_type=bool),
            'lidar_reliability': ParameterValue(
                LaunchConfiguration('lidar_reliability'), value_type=str),
            'report_dir': ParameterValue(LaunchConfiguration('report_dir'), value_type=str),
            **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
               for name in ('duration_sec', 'report_period_sec', 'stale_after_sec', 'window_sec')},
        }],
    )
    probes = ExecuteProcess(
        cmd=['python3', '-m', 'guidedog_perception.gazebo_probe',
             '--directory', LaunchConfiguration('report_dir'),
             '--duration', LaunchConfiguration('probe_duration_sec')],
        condition=IfCondition(LaunchConfiguration('record_gazebo')), output='screen',
    )
    return LaunchDescription(declarations + [
        OpaqueFunction(function=prepare_report_directory),
        RegisterEventHandler(OnProcessExit(on_exit=stop_when_gazebo_exits)),
        LogInfo(msg=['GuideDog reports: ', LaunchConfiguration('report_dir')]),
        include, monitor, probes,
    ])
