#!/usr/bin/env bash
# Starts the full braking_system stack (perception + adapter + visualizer +
# behavior_planner + plan_long_emergency + ctrl_long_emergency) with one command.
# Run this directly on the car after the workspace has been built once with
# `colcon build`. Extra args are forwarded to the launch file, e.g.:
#   ./start.sh safety_distance:=8.0
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

exec ros2 launch behavior_planner braking_system.launch.py "$@"
