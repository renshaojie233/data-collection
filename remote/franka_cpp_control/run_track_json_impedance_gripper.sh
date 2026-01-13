#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
JSON_PATH="${2:-}"
JSON_DIR="/home/rsj/franka_cpp_control/replay_data/action"

if [[ -z "${JSON_PATH}" ]]; then
  JSON_PATH="$(find "${JSON_DIR}" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi

if [[ -z "${JSON_PATH}" ]]; then
  echo "No JSON files found in ${JSON_DIR}"
  exit 1
fi

TIME_SCALE="${TIME_SCALE:-1.5}"
START_SPEED="${START_SPEED:-0.1}"
K_GAIN="${K_GAIN:-30}"
D_GAIN="${D_GAIN:-5}"
K_ALPHA="${K_ALPHA:-0.3}"
Q_ALPHA="${Q_ALPHA:-0.2}"
GRIPPER_SPEED="${GRIPPER_SPEED:-0.1}"
GRIPPER_FORCE="${GRIPPER_FORCE:-20.0}"
GRIPPER_START_DELAY="${GRIPPER_START_DELAY:-0.5}"

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"

/home/rsj/franka_cpp_control/build/franka_track_json_gripper_only \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${GRIPPER_SPEED}" \
  "${GRIPPER_START_DELAY}" &
GRIPPER_PID=$!

/home/rsj/franka_cpp_control/build/franka_track_json_impedance_smooth \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${K_GAIN}" \
  "${D_GAIN}" \
  "${K_ALPHA}" \
  "${Q_ALPHA}"

wait "${GRIPPER_PID}"
