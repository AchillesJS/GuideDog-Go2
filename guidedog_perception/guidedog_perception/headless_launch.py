# SPDX-License-Identifier: Apache-2.0
"""Reuse the simulator description, changing only the Gazebo rendering profile."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions


def server_only(description):
    """Append server/EGL flags to the one Gazebo include; reject an ambiguous layout."""
    matches = [entity for entity in description.entities
               if isinstance(entity, IncludeLaunchDescription)
               and any(key == 'gz_args' for key, _ in entity.launch_arguments)]
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one upstream Gazebo include with gz_args')
    original = matches[0]
    arguments = [(key, normalize_to_list_of_substitutions(value)
                  + [TextSubstitution(text=' -s --headless-rendering')]
                  if key == 'gz_args' else value)
                 for key, value in original.launch_arguments]
    replacement = IncludeLaunchDescription(
        original.launch_description_source, launch_arguments=arguments,
        condition=original.condition)
    return LaunchDescription([replacement if entity is original else entity
                              for entity in description.entities])
