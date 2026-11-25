#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

cd /workspace/ros2_ws
# A 3 külső message-repo (crp_msgs, autoware_msgs, tier4_autoware_msgs)
# már lefordítva van image-buildkor (lásd Dockerfile) - ez itt csak a
# braking_system-et (bind-mountolva, élőben szerkeszthető a host-on)
# fordítja hozzá, colcon incrementálisan.
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
echo "Build kész. Futtatás: source install/setup.bash && ./src/braking_system/start.sh"
