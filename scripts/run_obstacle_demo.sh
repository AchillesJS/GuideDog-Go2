#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$repo_dir/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID="${GUIDEDOG_ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${GUIDEDOG_GZ_PARTITION:-guidedog_stop}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec python3 -m guidedog_perception.launch_supervisor \
  guidedog_perception obstacle_stop.launch.py "$@"
