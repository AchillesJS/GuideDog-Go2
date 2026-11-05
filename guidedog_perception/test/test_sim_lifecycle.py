# SPDX-License-Identifier: Apache-2.0
"""Check actual ROS Launch shutdown after closing the included Gazebo process."""

import subprocess
import sys
import textwrap


def test_gazebo_exit_stops_its_launch(tmp_path):
    """A clean GUI-process exit cannot leave the other launch children running."""
    code = textwrap.dedent("""
        import sys
        from launch import LaunchDescription, LaunchService
        from launch.actions import ExecuteProcess, RegisterEventHandler
        from launch.event_handlers import OnProcessExit
        from guidedog_perception.sim_lifecycle import stop_when_gazebo_exits
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            RegisterEventHandler(OnProcessExit(on_exit=stop_when_gazebo_exits)),
            ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(0.3)'],
                           name='gazebo'),
            ExecuteProcess(cmd=['sleep', '60'], name='fake_controller'),
        ]))
        raise SystemExit(service.run())
    """)
    result = subprocess.run([sys.executable, '-c', code], capture_output=True,
                            text=True, timeout=10)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'SIGINT' in result.stdout+result.stderr
