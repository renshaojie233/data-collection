#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
OPEN_WIDTH="${2:-0.08}"
SPEED="${3:-0.1}"
FORCE="${4:-20}"

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"

exec /home/rsj/franka_cpp_control/build/franka_gripper_control \
  "${ROBOT_IP}" \
  pulse \
  "${OPEN_WIDTH}" \
  "${SPEED}" \
  "${FORCE}"
