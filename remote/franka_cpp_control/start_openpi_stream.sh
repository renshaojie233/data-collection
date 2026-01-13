#!/usr/bin/env bash
set -euo pipefail

BIN="/home/rsj/franka_cpp_control/build/franka_openpi_stream_vel_robotiq"
PID_FILE="/tmp/openpi_stream_vel.pid"
LOG_FILE="/tmp/openpi_stream_vel.log"
STOP_SCRIPT="${STOP_SCRIPT:-/home/rsj/gello_software/kill_ros2_stale.sh}"
LIBFRANKA_LIB_DIR="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
ACTION_PORT="${ACTION_PORT:-15123}"
STATE_PORT="${STATE_PORT:-15124}"
RATE_HZ="${RATE_HZ:-15}"
VEL_SCALE="${VEL_SCALE:-0.2}"
MAX_ACC="${MAX_ACC:-5}"
VEL_ALPHA="${VEL_ALPHA:-0.9}"
TIMEOUT_S="${TIMEOUT_S:-2}"
INVERT_GRIPPER="${INVERT_GRIPPER:-1}"
ROBOTIQ_PORT="${ROBOTIQ_PORT:-/dev/robotiq}"

if [[ ! -x "$BIN" ]]; then
  echo "Missing binary: $BIN" >&2
  exit 1
fi

cmd() {
  export ROBOTIQ_PORT
  if [[ -d "$LIBFRANKA_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$LIBFRANKA_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  "$BIN" "$ROBOT_IP" "$ACTION_PORT" "$STATE_PORT" \
    "$RATE_HZ" "$VEL_SCALE" "$MAX_ACC" "$VEL_ALPHA" "$TIMEOUT_S" "$INVERT_GRIPPER"
}

case "${1:-start}" in
  start)
    if [[ -x "$STOP_SCRIPT" ]]; then
      echo "Stopping ROS processes via $STOP_SCRIPT"
      "$STOP_SCRIPT" || true
    fi
    if pgrep -f "/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh" >/dev/null 2>&1; then
      echo "Force killing run_fr3_real_ros2_robotiq.sh"
      pkill -9 -f "/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh" || true
    fi
    if [[ -f "$PID_FILE" ]] && ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      rm -f "$PID_FILE"
    fi
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "openpi stream already running (pid $(cat "$PID_FILE"))"
      exit 0
    fi
    cmd_str="ROBOTIQ_PORT=${ROBOTIQ_PORT} LD_LIBRARY_PATH=${LIBFRANKA_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ${BIN} ${ROBOT_IP} ${ACTION_PORT} ${STATE_PORT} ${RATE_HZ} ${VEL_SCALE} ${MAX_ACC} ${VEL_ALPHA} ${TIMEOUT_S} ${INVERT_GRIPPER}"
    nohup bash -lc "$cmd_str" >"$LOG_FILE" 2>&1 &
    echo $! >"$PID_FILE"
    echo "openpi stream started (pid $!), log: $LOG_FILE"
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
