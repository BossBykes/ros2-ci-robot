#!/usr/bin/env bash
# Sources ROS 2 Jazzy and (if built) the project workspace overlay,
# then executes whatever command the container was started with.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi

exec "$@"
