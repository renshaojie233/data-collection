#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
JSON_PATH="${2:-/home/rsj/franka_cpp_control/replay_data/action/action_data_20251224_045112.json}"

TIME_SCALE="${TIME_SCALE:-1.5}"
START_SPEED="${START_SPEED:-0.1}"
K_GAIN="${K_GAIN:-30}"
D_GAIN="${D_GAIN:-5}"
K_ALPHA="${K_ALPHA:-0.3}"
Q_ALPHA="${Q_ALPHA:-0.2}"

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"

exec /home/rsj/franka_cpp_control/build/franka_track_json_impedance_smooth \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${K_GAIN}" \
  "${D_GAIN}" \
  "${K_ALPHA}" \
  "${Q_ALPHA}"
