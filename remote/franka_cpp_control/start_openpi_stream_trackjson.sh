#!/usr/bin/env bash
set -euo pipefail

BIN="/home/rsj/franka_cpp_control/build
ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
START_JSON="${START_JSON:-/home/rsj/franka_cpp_control/replay_data/action/action_data_20251230_012732.json}"
START_SPEED="${START_SPEED:-0.1}"
ACTION_PORT="${ACTION_PORT:-15123}"
STATE_PORT="${STATE_PORT:-15124}"
RATE_HZ="${RATE_HZ:-30}"
MAX_ACC="${MAX_ACC:-5}"/franka_openpi_stream_vel_robotiq_trackjson_30hz"
MOVE_BIN="/home/rsj/franka_cpp_control/build/franka_move_to_json_start"
PID_FILE="/tmp/openpi_stream_vel_trackjson.pid"
LOG_FILE="/tmp/openpi_stream_vel_trackjson.log"
STOP_SCRIPT="${STOP_SCRIPT:-/home/rsj/gello_software/kill_ros2_stale.sh}"
LIBFRANKA_LIB_DIR="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"

VEL_ALPHA="${VEL_ALPHA:-0.9}"
TIMEOUT_S="${TIMEOUT_S:-2}"
INVERT_GRIPPER="${INVERT_GRIPPER:-1}"
KP="${KP:-2.0}"
TIME_SCALE="${TIME_SCALE:-1.0}"
ROBOTIQ_PORT="${ROBOTIQ_PORT:-/dev/robotiq}"
ACTION_LOG_DIR="${ACTION_LOG_DIR:-/home/rsj/franka_cpp_control/action_buffer}"
#ACTION_ORDER="${ACTION_ORDER:-fr3_joint1,fr3_joint3,fr3_joint6,fr3_joint7,fr3_joint2,fr3_joint4,fr3_joint5}"
ACTION_ORDER="${ACTION_ORDER:-fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,fr3_joint5,fr3_joint6,fr3_joint7}"


mkdir -p "$ACTION_LOG_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "Missing binary: $BIN" >&2
  exit 1
fi
if [[ ! -x "$MOVE_BIN" ]]; then
  echo "Missing binary: $MOVE_BIN" >&2
  exit 1
fi
if [[ ! -f "$START_JSON" ]]; then
  echo "Missing start JSON: $START_JSON" >&2
  exit 1
fi

move_to_start() {
  if [[ -d "$LIBFRANKA_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$LIBFRANKA_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  echo "Moving to start pose from ${START_JSON}"
  "$MOVE_BIN" "$ROBOT_IP" "$START_JSON" "$START_SPEED"
}

cmd() {
  export ROBOTIQ_PORT
  if [[ -d "$LIBFRANKA_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$LIBFRANKA_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  export ACTION_LOG_DIR ACTION_ORDER
  "$BIN" "$ROBOT_IP" "$ACTION_PORT" "$STATE_PORT" \
    "$RATE_HZ" "$MAX_ACC" "$VEL_ALPHA" "$TIMEOUT_S" "$INVERT_GRIPPER" "$KP" "$TIME_SCALE"
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
      echo "openpi stream trackjson already running (pid $(cat "$PID_FILE"))"
      exit 0
    fi
    move_to_start
    cmd_str="ROBOTIQ_PORT=${ROBOTIQ_PORT} ACTION_LOG_DIR=${ACTION_LOG_DIR} ACTION_ORDER=${ACTION_ORDER} LD_LIBRARY_PATH=${LIBFRANKA_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ${BIN} ${ROBOT_IP} ${ACTION_PORT} ${STATE_PORT} ${RATE_HZ} ${MAX_ACC} ${VEL_ALPHA} ${TIMEOUT_S} ${INVERT_GRIPPER} ${KP} ${TIME_SCALE}"
    nohup bash -lc "$cmd_str" >"$LOG_FILE" 2>&1 &
    echo $! >"$PID_FILE"
    echo "openpi stream trackjson started (pid $!), log: $LOG_FILE"
    ;;
  run)
    move_to_start
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
