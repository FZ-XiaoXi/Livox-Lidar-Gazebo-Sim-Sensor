#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
source "${workspace_dir}/install/setup.bash"

exec ros2 launch livox_lidar_example livox_example.launch.py "$@"
