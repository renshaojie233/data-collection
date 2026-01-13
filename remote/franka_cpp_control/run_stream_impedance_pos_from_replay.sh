#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
JSON_PATH="${2:-}"
JSON_DIR="/home/rsj/franka_cpp_control/replay_data/action"

ACTION_HOST="${ACTION_HOST:-127.0.0.1}"
ACTION_PORT="${ACTION_PORT:-15123}"
STATE_HOST="${STATE_HOST:-127.0.0.1}"
STATE_PORT="${STATE_PORT:-15124}"
RATE_HZ="${RATE_HZ:-30}"
TIME_SCALE="${TIME_SCALE:-1.0}"
BLEND_S="${BLEND_S:-2}"
MOVE_TO_START="${MOVE_TO_START:-0}"
START_SPEED="${START_SPEED:-0.1}"

ACTION_ORDER="${ACTION_ORDER:-fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,fr3_joint5,fr3_joint6,fr3_joint7}"
GRIPPER_MAX_WIDTH="${GRIPPER_MAX_WIDTH:-}"
GRIPPER_INVERT_SEND="${GRIPPER_INVERT_SEND:-0}"

if [[ -z "${JSON_PATH}" ]]; then
  JSON_PATH="$(find "${JSON_DIR}" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi

if [[ -z "${JSON_PATH}" ]]; then
  echo "No JSON files found in ${JSON_DIR}" >&2
  exit 1
fi

MOVE_BIN="/home/rsj/franka_cpp_control/build/franka_move_to_json_start"
if [[ "${MOVE_TO_START}" = "1" ]]; then
  if [[ ! -x "${MOVE_BIN}" ]]; then
    echo "Missing binary: ${MOVE_BIN}" >&2
    exit 1
  fi
  echo "Moving to start pose from ${JSON_PATH}"
  "${MOVE_BIN}" "${ROBOT_IP}" "${JSON_PATH}" "${START_SPEED}"
fi

export ACTION_HOST ACTION_PORT STATE_HOST STATE_PORT RATE_HZ TIME_SCALE BLEND_S
export ACTION_ORDER GRIPPER_MAX_WIDTH GRIPPER_INVERT_SEND

exec /home/rsj/franka_cpp_control/stream_json_positions.py "${JSON_PATH}"
