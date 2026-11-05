#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$repo_dir/../.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is missing: /opt/ros/jazzy/setup.bash' >&2
  exit 1
fi
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Build the ROS workspace first: $workspace_dir" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
if ! ros2 pkg executables guidedog_perception | grep -q ' sensor_monitor$'; then
  echo 'Build guidedog_perception first: colcon build --packages-select guidedog_perception --symlink-install' >&2
  exit 1
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec python3 -m guidedog_perception.launch_supervisor \
  guidedog_perception simulation.launch.py "$@"
