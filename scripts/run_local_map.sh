#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$script_dir/run_obstacle_demo.sh" enable_local_map:=true "$@"
