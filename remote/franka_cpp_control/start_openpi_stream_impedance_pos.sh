#!/usr/bin/env bash
set -euo pipefail

BIN="/home/rsj/franka_cpp_control/build/franka_openpi_stream_impedance_pos_robotiq"
ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
ACTION_PORT="${ACTION_PORT:-15123}"
STATE_PORT="${STATE_PORT:-15124}"
RATE_HZ="${RATE_HZ:-30}"
K_GAIN="${K_GAIN:-30}"
D_GAIN="${D_GAIN:-5}"
K_ALPHA="${K_ALPHA:-0.3}"
Q_ALPHA="${Q_ALPHA:-0.2}"
TIMEOUT_S="${TIMEOUT_S:-2}"
ROBOTIQ_PORT="${ROBOTIQ_PORT:-/dev/robotiq}"
INVERT_GRIPPER="${INVERT_GRIPPER:-0}"
ACTION_LOG_DIR="${ACTION_LOG_DIR:-/home/rsj/franka_cpp_control/action_buffer}"
ACTION_ORDER="${ACTION_ORDER:-fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,fr3_joint5,fr3_joint6,fr3_joint7}"
USE_GRAVITY_COMP="${USE_GRAVITY_COMP:-0}"
LOAD_MASS="${LOAD_MASS:-}"
LOAD_COM="${LOAD_COM:-}"
LOAD_INERTIA="${LOAD_INERTIA:-}"
LOAD_SCALE="${LOAD_SCALE:-1}"
K_GAINS="${K_GAINS:-}"
D_GAINS="${D_GAINS:-}"
PID_FILE="/tmp/openpi_stream_impedance_pos.pid"
LOG_FILE="/tmp/openpi_stream_impedance_pos.log"
STOP_SCRIPT="${STOP_SCRIPT:-/home/rsj/gello_software/kill_ros2_stale.sh}"
LIBFRANKA_LIB_DIR="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"

mkdir -p "$ACTION_LOG_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "Missing binary: $BIN" >&2
  exit 1
fi

cmd() {
  export ROBOTIQ_PORT ACTION_LOG_DIR ACTION_ORDER
  export INVERT_GRIPPER
  export USE_GRAVITY_COMP LOAD_MASS LOAD_COM LOAD_INERTIA LOAD_SCALE
  export K_GAINS D_GAINS
  if [[ -d "$LIBFRANKA_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$LIBFRANKA_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  "$BIN" "$ROBOT_IP" "$ACTION_PORT" "$STATE_PORT" "$RATE_HZ" \
    "$K_GAIN" "$D_GAIN" "$K_ALPHA" "$Q_ALPHA" "$TIMEOUT_S"
}

case "${1:-start}" in
  start)
    if [[ -x "$STOP_SCRIPT" ]]; then
      echo "Stopping ROS processes via $STOP_SCRIPT"
      "$STOP_SCRIPT" || true
    fi
    if [[ -f "$PID_FILE" ]] && ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      rm -f "$PID_FILE"
    fi
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "openpi impedance stream already running (pid $(cat "$PID_FILE"))"
      exit 0
    fi
    cmd_str="ROBOTIQ_PORT=${ROBOTIQ_PORT} INVERT_GRIPPER=${INVERT_GRIPPER} ACTION_LOG_DIR=${ACTION_LOG_DIR} ACTION_ORDER=${ACTION_ORDER} USE_GRAVITY_COMP=${USE_GRAVITY_COMP} LOAD_MASS=${LOAD_MASS} LOAD_COM=${LOAD_COM} LOAD_INERTIA=${LOAD_INERTIA} LOAD_SCALE=${LOAD_SCALE} K_GAINS=${K_GAINS} D_GAINS=${D_GAINS} LD_LIBRARY_PATH=${LIBFRANKA_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ${BIN} ${ROBOT_IP} ${ACTION_PORT} ${STATE_PORT} ${RATE_HZ} ${K_GAIN} ${D_GAIN} ${K_ALPHA} ${Q_ALPHA} ${TIMEOUT_S}"
    nohup bash -lc "$cmd_str" >"$LOG_FILE" 2>&1 &
    echo $! >"$PID_FILE"
    echo "openpi impedance stream started (pid $!), log: $LOG_FILE"
    ;;
  run)
    cmd
    ;;
  stop)
    if [[ -f "$PID_FILE" ]]; then
      PID="$(cat "$PID_FILE")"
      if kill -0 "$PID" 2>/dev/null; then
        kill "$PID"
        echo "stopped $PID"
      fi
      rm -f "$PID_FILE"
    fi
    ;;
  status)
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "running (pid $(cat "$PID_FILE"))"
    else
      echo "not running"
      exit 1
    fi
    ;;
  *)
    echo "Usage: $0 {start|run|stop|status}" >&2
    exit 1
    ;;
esac
