#!/bin/bash
set -euo pipefail

echo "Killing stale ROS2/Franka/GELLO processes..."

if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi
pkill -f 'ros2cli.daemon' 2>/dev/null || true
pkill -f 'franka_gello_state_publisher|gello_publisher' 2>/dev/null || true
pkill -f 'franka_fr3_arm_controllers|ros2_control_node' 2>/dev/null || true
pkill -f 'franka_gripper_manager|franka_gripper_client|franka_gripper_node' 2>/dev/null || true
pkill -f 'robot_state_publisher|joint_state_publisher' 2>/dev/null || true
pkill -f 'ros2 launch franka_gello_state_publisher|ros2 launch franka_fr3_arm_controllers|ros2 launch franka_gripper_manager' 2>/dev/null || true

sleep 1

echo "Done. Remaining related processes:"
ps -eo pid,ppid,pgid,sid,stat,cmd \
    | grep -Ei 'ros2|franka|gello|robot_state_publisher|joint_state_publisher' \
    | grep -Ev 'kill_ros2_stale.sh|grep -Ei' || true
