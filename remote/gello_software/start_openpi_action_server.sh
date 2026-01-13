#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO=humble
FRANKA_WS=/home/ubuntu/franka_ros2_ws
GELLO_WS=/home/ubuntu/gello_software/ros2

if [ -f /opt/ros//setup.bash ]; then
  # shellcheck disable=SC1090
  source /opt/ros//setup.bash
fi
if [ -f /install/setup.bash ]; then
  # shellcheck disable=SC1090
  source /install/setup.bash
fi
if [ -f /install/setup.bash ]; then
  # shellcheck disable=SC1090
  source /install/setup.bash
fi

exec python3 /home/rsj/gello_software/openpi_action_server.py 
