# SPDX-License-Identifier: Apache-2.0
"""End a project launch when its included Gazebo process has exited."""

from launch.actions import EmitEvent
from launch.events import Shutdown


def stop_when_gazebo_exits(event, context):
    """Prevent orphan ROS controllers after the user closes or quits Gazebo."""
    # ros_gz_sim names its ExecuteProcess 'gazebo'; launch adds the numeric suffix.
    if event.process_name.rsplit('-', 1)[0] == 'gazebo':
        return [EmitEvent(event=Shutdown(reason='Gazebo exited; stopping this GuideDog launch'))]
    return []
