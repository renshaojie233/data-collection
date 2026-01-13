#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
JSON_PATH="${2:-}"
JSON_DIR="/home/rsj/franka_cpp_control/replay_data/action"

pick_latest_json_with_gripper() {
  python3 - <<'PY'
import json
import os
import sys

json_dir = "/home/rsj/franka_cpp_control/replay_data/action"
paths = []
for name in os.listdir(json_dir):
    if not name.endswith(".json"):
        continue
    path = os.path.join(json_dir, name)
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        continue
    paths.append((mtime, path))

paths.sort(reverse=True)
for _, path in paths:
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except Exception:
        continue
    entries = data.get("data", [])
    for entry in entries[:50]:
        gripper = entry.get("gripper_joints")
        if not isinstance(gripper, dict):
            continue
        names = gripper.get("names")
        pos = gripper.get("position")
        if isinstance(names, list) and isinstance(pos, list) and names and pos:
            print(path)
            sys.exit(0)
    # fallback: scan more if first entries missing
    for entry in entries[50:200]:
        gripper = entry.get("gripper_joints")
        if not isinstance(gripper, dict):
            continue
        names = gripper.get("names")
        pos = gripper.get("position")
        if isinstance(names, list) and isinstance(pos, list) and names and pos:
            print(path)
            sys.exit(0)
sys.exit(0)
PY
}

if [[ -z "${JSON_PATH}" ]]; then
  JSON_PATH="$(pick_latest_json_with_gripper)"
fi

if [[ -z "${JSON_PATH}" ]]; then
  JSON_PATH="$(find "${JSON_DIR}" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi

if [[ -z "${JSON_PATH}" ]]; then
  echo "No JSON files found in ${JSON_DIR}"
  exit 1
fi

TIME_SCALE="${TIME_SCALE:-1.5}"
START_SPEED="${START_SPEED:-0.1}"
K_GAIN="${K_GAIN:-240}"
D_GAIN="${D_GAIN:-20}"
K_ALPHA="${K_ALPHA:-0.99}"
Q_ALPHA="${Q_ALPHA:-1.0}"
GRIPPER_SPEED="${GRIPPER_SPEED:-0.1}"
GRIPPER_FORCE="${GRIPPER_FORCE:-20.0}"
GRIPPER_START_DELAY="${GRIPPER_START_DELAY:-0.5}"

ROBOTIQ_PORT="${ROBOTIQ_PORT:-}"
ROBOTIQ_BAUD="${ROBOTIQ_BAUD:-115200}"
ROBOTIQ_SLAVE_ID="${ROBOTIQ_SLAVE_ID:-9}"
ROBOTIQ_OUT_START="${ROBOTIQ_OUT_START:-0x03E8}"
ROBOTIQ_IN_START="${ROBOTIQ_IN_START:-0x07D0}"
ROBOTIQ_MIN_CMD_MS="${ROBOTIQ_MIN_CMD_MS:-200}"
ROBOTIQ_TIMEOUT_MS="${ROBOTIQ_TIMEOUT_MS:-200}"
ROBOTIQ_SOURCE_MAX_WIDTH="${ROBOTIQ_SOURCE_MAX_WIDTH:-0.08}"
ROBOTIQ_POSITION_EPS="${ROBOTIQ_POSITION_EPS:-0.02}"
ROBOTIQ_SPEED="${ROBOTIQ_SPEED:-${GRIPPER_SPEED}}"
ROBOTIQ_FORCE="${ROBOTIQ_FORCE:-${GRIPPER_FORCE}}"
ROBOTIQ_LAYOUT="${ROBOTIQ_LAYOUT:-codex}"
ROBOTIQ_START_MODE="${ROBOTIQ_START_MODE:-hold}"
ROBOTIQ_ACTIVATE="${ROBOTIQ_ACTIVATE:-auto}"
USE_GRAVITY_COMP="${USE_GRAVITY_COMP:-0}"
LOAD_MASS="${LOAD_MASS:-0.92086427304}"
LOAD_COM="${LOAD_COM:--0.0000093915,-0.0000429355,0.0474194011}"
LOAD_INERTIA="${LOAD_INERTIA:-0.00124612931,0.00000407086,0.000000108194,0.00000407086,0.00203808113,0.00000257136,0.000000108194,0.00000257136,0.00115342433}"
LOAD_SCALE="${LOAD_SCALE:-0.7}"
K_GAINS="${K_GAINS:-240,240,240,240,100,60,20}"
D_GAINS="${D_GAINS:-20,20,20,10,10,10,5}"
STOP_CONFLICTS="${STOP_CONFLICTS:-1}"

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

stop_conflicts() {
  local patterns=(
    "/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh"
    "franka_fr3_arm_controllers"
    "franka_gripper_manager"
    "robotiq_modbus_gripper"
    "robotiq_gripper_client"
    "franka_gello_state_publisher"
  )
  local -a pids=()
  declare -A seen=()
  for pat in "${patterns[@]}"; do
    while read -r pid _; do
      if [ -z "$pid" ]; then
        continue
      fi
      if [ "$pid" -eq "$$" ] || [ "$pid" -eq "$PPID" ]; then
        continue
      fi
      if [ -n "${seen[$pid]+x}" ]; then
        continue
      fi
      seen[$pid]=1
      pids+=("$pid")
    done < <(pgrep -af "$pat" || true)
  done
  if [ "${#pids[@]}" -gt 0 ]; then
    echo "Stopping conflicting ROS2/robotiq processes: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 2
  fi
}

if [ "${STOP_CONFLICTS}" = "1" ]; then
  stop_conflicts
fi

if [ -n "$ROBOTIQ_PORT" ] && [ ! -e "$ROBOTIQ_PORT" ]; then
  echo "ROBOTIQ_PORT not found: $ROBOTIQ_PORT"
  ROBOTIQ_PORT=""
fi

if [ -z "$ROBOTIQ_PORT" ]; then
  ROBOTIQ_PORT="$(detect_robotiq_port || true)"
fi

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"
export ROBOTIQ_PORT ROBOTIQ_BAUD ROBOTIQ_SLAVE_ID ROBOTIQ_OUT_START ROBOTIQ_IN_START
export ROBOTIQ_MIN_CMD_MS ROBOTIQ_TIMEOUT_MS ROBOTIQ_SOURCE_MAX_WIDTH ROBOTIQ_POSITION_EPS
export ROBOTIQ_SPEED ROBOTIQ_FORCE ROBOTIQ_LAYOUT ROBOTIQ_START_MODE ROBOTIQ_ACTIVATE
export USE_GRAVITY_COMP LOAD_MASS LOAD_COM LOAD_INERTIA LOAD_SCALE
export K_GAINS D_GAINS

echo "JSON: ${JSON_PATH}"
echo "Robotiq port: ${ROBOTIQ_PORT:-<not found>}"

if [ -n "$ROBOTIQ_PORT" ]; then
  set +e
  python3 - <<'PY'
import os
import errno
import sys
port = os.environ.get("ROBOTIQ_PORT")
if not port:
    sys.exit(0)
try:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    os.close(fd)
except OSError as exc:
    if exc.errno == errno.EACCES:
        print(f"Error: cannot open {port} for read/write: {exc}")
        sys.exit(3)
    print(f"Warning: cannot open {port}: {exc}")
    sys.exit(1)
sys.exit(0)
PY
  rc=$?
  set -e
  if [ "$rc" -eq 3 ]; then
    cat <<'EOF'
Error: Robotiq serial port is not writable.
Common fixes:
  - Stop ModemManager: sudo systemctl stop ModemManager
  - Add a udev rule to ignore the device (see PROJECT_OVERVIEW_zh.md)
  - Replug the USB and re-run
EOF
    exit 2
  fi
else
  echo "Warning: ROBOTIQ_PORT not found"
fi

if command -v systemctl >/dev/null 2>&1; then
  if systemctl is-active ModemManager >/dev/null 2>&1; then
    echo "Warning: ModemManager is active and may interfere with ${ROBOTIQ_PORT}"
  fi
fi
if pgrep -x ModemManager >/dev/null 2>&1; then
  echo "Warning: ModemManager is running and may interfere with ${ROBOTIQ_PORT}"
fi

if ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "Robot reachable: ${ROBOT_IP}"
else
  echo "Warning: cannot ping robot (${ROBOT_IP})"
fi

/home/rsj/franka_cpp_control/build/franka_track_json_gripper_only_robotiq \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${GRIPPER_SPEED}" \
  "${GRIPPER_START_DELAY}" \
  "${GRIPPER_FORCE}" &
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
