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
K_GAIN="${K_GAIN:-100}"
D_GAIN="${D_GAIN:-10}"
K_ALPHA="${K_ALPHA:-0.1}"
GRIPPER_SPEED="${GRIPPER_SPEED:-0.1}"
GRIPPER_FORCE="${GRIPPER_FORCE:-20.0}"
ROBOTIQ_PORT="${ROBOTIQ_PORT:-}"
ROBOTIQ_BAUD="${ROBOTIQ_BAUD:-115200}"
ROBOTIQ_SLAVE_ID="${ROBOTIQ_SLAVE_ID:-9}"
ROBOTIQ_OUT_START="${ROBOTIQ_OUT_START:-0x03E8}"
ROBOTIQ_IN_START="${ROBOTIQ_IN_START:-0x07D0}"
ROBOTIQ_SOURCE_MAX_WIDTH="${ROBOTIQ_SOURCE_MAX_WIDTH:-0.08}"
ROBOTIQ_POSITION_EPS="${ROBOTIQ_POSITION_EPS:-0.02}"
ROBOTIQ_MIN_CMD_MS="${ROBOTIQ_MIN_CMD_MS:-100}"
ROBOTIQ_TIMEOUT_MS="${ROBOTIQ_TIMEOUT_MS:-200}"
ROBOTIQ_SPEED="${ROBOTIQ_SPEED:-${GRIPPER_SPEED}}"
ROBOTIQ_FORCE="${ROBOTIQ_FORCE:-${GRIPPER_FORCE}}"
ROBOTIQ_LAYOUT="${ROBOTIQ_LAYOUT:-codex}"
ROBOTIQ_ACTIVATE="${ROBOTIQ_ACTIVATE:-auto}"

detect_robotiq_port() {
  local matches=()
  local pattern
  if [ -e /dev/robotiq ]; then
    echo "/dev/robotiq"
    return 0
  fi
  for pattern in /dev/serial/by-id/usb-FTDI_USB_TO_RS-485* /dev/serial/by-id/usb-FTDI_FT232* /dev/serial/by-id/usb-ROBOTIQ*; do
    [ -e "$pattern" ] && matches+=("$pattern")
  done
  if [ "${#matches[@]}" -eq 1 ]; then
    echo "${matches[0]}"
    return 0
  fi
  matches=()
  for pattern in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$pattern" ] && matches+=("$pattern")
  done
  if [ "${#matches[@]}" -eq 1 ]; then
    echo "${matches[0]}"
    return 0
  fi
  return 1
}

if [ -z "$ROBOTIQ_PORT" ]; then
  ROBOTIQ_PORT="$(detect_robotiq_port || true)"
fi

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"
export ROBOTIQ_PORT ROBOTIQ_BAUD ROBOTIQ_SLAVE_ID ROBOTIQ_OUT_START ROBOTIQ_IN_START
export ROBOTIQ_SOURCE_MAX_WIDTH ROBOTIQ_POSITION_EPS
export ROBOTIQ_MIN_CMD_MS ROBOTIQ_TIMEOUT_MS ROBOTIQ_SPEED ROBOTIQ_FORCE ROBOTIQ_LAYOUT ROBOTIQ_ACTIVATE

echo "Running raw impedance control (no target smoothing) with Robotiq 2F-85"
echo "Robot IP: ${ROBOT_IP}"
echo "JSON: ${JSON_PATH}"
echo "Time scale: ${TIME_SCALE}"
echo "K gain: ${K_GAIN}, D gain: ${D_GAIN}"
echo "K alpha (velocity filter): ${K_ALPHA}"
echo "Robotiq port: ${ROBOTIQ_PORT:-/dev/ttyUSB0}"
echo "Robotiq baud: ${ROBOTIQ_BAUD}, slave_id: ${ROBOTIQ_SLAVE_ID}, out_start: ${ROBOTIQ_OUT_START}"
echo "Robotiq width source_max: ${ROBOTIQ_SOURCE_MAX_WIDTH}"
echo "Robotiq speed: ${ROBOTIQ_SPEED}, force: ${ROBOTIQ_FORCE}"
echo "Note: Higher gains for better tracking, no target smoothing for maximum accuracy"

/home/rsj/franka_cpp_control/build/franka_track_json_impedance_gripper_raw \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${K_GAIN}" \
  "${D_GAIN}" \
  "${K_ALPHA}" \
  "${GRIPPER_SPEED}" \
  "${GRIPPER_FORCE}"
