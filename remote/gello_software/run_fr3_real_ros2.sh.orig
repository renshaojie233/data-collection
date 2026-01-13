#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

safe_source() {
    local file="$1"
    set +u
    # shellcheck disable=SC1090
    source "$file"
    set -u
}

sanitize_path() {
    local path="${1:-}"
    local filtered=""
    IFS=":" read -r -a parts <<< "$path"
    for p in "${parts[@]}"; do
        case "$p" in
            "$HOME/anaconda3/bin"|"$HOME/anaconda3/condabin"|"$HOME/miniconda3/bin"|"$HOME/miniconda3/condabin") ;;
            "") ;;
            *) filtered="${filtered:+$filtered:}$p" ;;
        esac
    done
    echo "$filtered"
}

sanitize_ld_library_path() {
    local path="${1:-}"
    local filtered=""
    IFS=":" read -r -a parts <<< "$path"
    for p in "${parts[@]}"; do
        case "$p" in
            "$HOME/anaconda3/lib"|"$HOME/miniconda3/lib") ;;
            "") ;;
            *) filtered="${filtered:+$filtered:}$p" ;;
        esac
    done
    echo "$filtered"
}

sanitize_pythonpath() {
    local path="${1:-}"
    local filtered=""
    IFS=":" read -r -a parts <<< "$path"
    for p in "${parts[@]}"; do
        case "$p" in
            *"/anaconda3/"*|*"/miniconda3/"*|*"/conda/"*|"$HOME/gello_control/third_party/franky"*) ;;
            "") ;;
            *) filtered="${filtered:+$filtered:}$p" ;;
        esac
    done
    echo "$filtered"
}

sanitize_env() {
    PATH="$(sanitize_path "$PATH")"
    export PATH

    if [ -n "${LD_LIBRARY_PATH:-}" ]; then
        LD_LIBRARY_PATH="$(sanitize_ld_library_path "$LD_LIBRARY_PATH")"
        if [ -n "$LD_LIBRARY_PATH" ]; then
            export LD_LIBRARY_PATH
        else
            unset LD_LIBRARY_PATH
        fi
    fi

    unset LD_PRELOAD || true
    if [ -n "${PYTHONPATH:-}" ]; then
        PYTHONPATH="$(sanitize_pythonpath "$PYTHONPATH")"
        if [ -n "$PYTHONPATH" ]; then
            export PYTHONPATH
        else
            unset PYTHONPATH
        fi
    fi
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_SHLVL _CE_CONDA _CE_M || true
}

stop_stale_gello() {
    local pids=""
    pids="$(pgrep -f 'franka_gello_state_publisher/gello_publisher' || true)"
    if [ -n "$pids" ]; then
        echo "Stopping existing gello_publisher processes: $pids"
        kill $pids 2>/dev/null || true
        sleep 1
    fi
}

wait_for_gello_topic() {
    local timeout_sec="${1:-15}"
    if timeout "${timeout_sec}s" ros2 topic echo --once /gello/joint_states >/dev/null 2>&1; then
        return 0
    fi
    local hz_log
    hz_log="$(mktemp)"
    timeout 6s ros2 topic hz /gello/joint_states >"$hz_log" 2>/dev/null || true
    if command -v rg >/dev/null 2>&1; then
        rg -q "average rate" "$hz_log" && { rm -f "$hz_log"; return 0; }
    else
        grep -q "average rate" "$hz_log" && { rm -f "$hz_log"; return 0; }
    fi
    rm -f "$hz_log"
    return 1
}

detect_gello_port() {
    local matches=()
    local pattern
    if [ -e /dev/gello ]; then
        echo "/dev/gello"
        return 0
    fi
    for pattern in /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter* /dev/serial/by-id/usb-ROBOTIS_OpenRB-150*; do
        [ -e "$pattern" ] && matches+=("$pattern")
    done
    if [ "${#matches[@]}" -eq 1 ]; then
        echo "${matches[0]}"
        return 0
    fi
    return 1
}

detect_fallback_port() {
    local matches=()
    local pattern
    for pattern in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$pattern" ] && matches+=("$pattern")
    done
    if [ "${#matches[@]}" -eq 1 ]; then
        echo "${matches[0]}"
        return 0
    fi
    return 1
}

wait_for_gello_port() {
    local attempts=0
    local port=""
    while true; do
        port="$(detect_gello_port || true)"
        if [ -n "$port" ]; then
            echo "$port"
            return 0
        fi
        port="$(detect_fallback_port || true)"
        if [ -n "$port" ]; then
            echo "$port"
            return 0
        fi
        attempts=$((attempts + 1))
        if [ "$attempts" -eq 1 ]; then
            echo "Waiting for GELLO USB device... (reconnect if needed)" >&2
        else
            echo "Still waiting for GELLO USB device..." >&2
        fi
        sleep 2
    done
}

update_gello_config_port() {
    local cfg="$1"
    local port_id="$2"
    local escaped="$port_id"
    escaped="${escaped//\\/\\\\}"
    escaped="${escaped//&/\\&}"
    escaped="${escaped//|/\\|}"
    [ -f "$cfg" ] || return 0
    if grep -q "^[[:space:]]*com_port:" "$cfg"; then
        sed -i "s|^[[:space:]]*com_port:.*|  com_port: \"${escaped}\"|" "$cfg"
    fi
}

ROS_DISTRO="${ROS_DISTRO:-humble}"
FRANKA_WS="${FRANKA_WS:-$HOME/franka_ros2_ws}"
GELLO_WS="${GELLO_WS:-$HOME/gello_software/ros2}"
LIBFRANKA_PREFIX="${LIBFRANKA_PREFIX:-}"

GELLO_CFG="${GELLO_CFG:-fr3_rsjt.yaml}"
FR3_CFG="${FR3_CFG:-fr3_rsjt.yaml}"
GRIPPER_CFG="${GRIPPER_CFG:-fr3_rsjt_franka_hand.yaml}"

GELLO_CFG_SRC="$GELLO_WS/src/franka_gello_state_publisher/config/$GELLO_CFG"
GELLO_CFG_INSTALL="$GELLO_WS/install/franka_gello_state_publisher/share/franka_gello_state_publisher/config/$GELLO_CFG"
FR3_CFG_SRC="$GELLO_WS/src/franka_fr3_arm_controllers/config/$FR3_CFG"
FR3_CFG_INSTALL="$GELLO_WS/install/franka_fr3_arm_controllers/share/franka_fr3_arm_controllers/config/$FR3_CFG"
GRIPPER_CFG_INSTALL="$GELLO_WS/install/franka_gripper_manager/share/franka_gripper_manager/config/$GRIPPER_CFG"

GELLO_CFG_PATH="$GELLO_CFG_INSTALL"
[ -f "$GELLO_CFG_PATH" ] || GELLO_CFG_PATH="$GELLO_CFG_SRC"
FR3_CFG_PATH="$FR3_CFG_INSTALL"
[ -f "$FR3_CFG_PATH" ] || FR3_CFG_PATH="$FR3_CFG_SRC"

sanitize_env

if [ -z "$LIBFRANKA_PREFIX" ]; then
    if [ -d "$HOME/.local/libfranka-0.18.0" ]; then
        LIBFRANKA_PREFIX="$HOME/.local/libfranka-0.18.0"
    else
        LIBFRANKA_PREFIX="/usr/local/libfranka-0.18.0"
    fi
fi
if [ -d "$LIBFRANKA_PREFIX/lib" ]; then
    export LD_LIBRARY_PATH="$LIBFRANKA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export CMAKE_PREFIX_PATH="$LIBFRANKA_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi

AUTO_PORT="$(wait_for_gello_port | tail -n 1)"
update_gello_config_port "$GELLO_CFG_SRC" "$AUTO_PORT"
update_gello_config_port "$GELLO_CFG_INSTALL" "$AUTO_PORT"

if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    safe_source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "Missing /opt/ros/$ROS_DISTRO/setup.bash"
    exit 1
fi

if [ -f "$FRANKA_WS/install/setup.bash" ]; then
    safe_source "$FRANKA_WS/install/setup.bash"
else
    echo "Missing Franka workspace: $FRANKA_WS/install/setup.bash"
    exit 1
fi

if [ -f "$GELLO_WS/install/setup.bash" ]; then
    safe_source "$GELLO_WS/install/setup.bash"
else
    echo "Missing GELLO ROS2 workspace: $GELLO_WS/install/setup.bash"
    exit 1
fi

if [ ! -f "$GELLO_CFG_INSTALL" ]; then
    echo "Missing installed GELLO config: $GELLO_CFG_INSTALL"
    echo "Rebuild the ROS2 workspace with colcon."
    exit 1
fi
if [ ! -f "$FR3_CFG_INSTALL" ]; then
    echo "Missing installed FR3 config: $FR3_CFG_INSTALL"
    echo "Rebuild the ROS2 workspace with colcon."
    exit 1
fi
if [ ! -f "$GRIPPER_CFG_INSTALL" ]; then
    echo "Missing installed gripper config: $GRIPPER_CFG_INSTALL"
    echo "Rebuild the ROS2 workspace with colcon."
    exit 1
fi

sanitize_env

if [ "${ROS2_DEBUG:-0}" = "1" ]; then
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
    export RCL_LOG_LEVEL=debug
fi

ROBOT_IP=""
if [ -f "$FR3_CFG_PATH" ]; then
    ROBOT_IP="$(awk -F: '/robot_ip:/ {gsub(/"/, "", $2); gsub(/[[:space:]]/, "", $2); print $2; exit}' "$FR3_CFG_PATH")"
fi

echo ""
echo "============================================================"
echo "GELLO -> Franka FR3 (ROS2)"
echo "============================================================"
if [ -n "$ROBOT_IP" ]; then
    echo "Robot IP: $ROBOT_IP"
    if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
        echo "✓ Robot reachable"
    else
        echo "⚠ Warning: cannot ping robot ($ROBOT_IP)"
    fi
else
    echo "⚠ Warning: robot_ip not found in $FR3_CFG_PATH"
fi
echo ""
echo "Serial devices:"
if [ -d /dev/serial/by-id ]; then
    ls -la /dev/serial/by-id/ 2>/dev/null || true
else
    echo "  (no /dev/serial/by-id found)"
fi
echo ""
echo "Using configs:"
echo "  GELLO: $GELLO_CFG"
echo "  FR3: $FR3_CFG"
echo "  Gripper: $GRIPPER_CFG"
echo ""

stop_stale_gello

echo "Launching GELLO publisher..."
ros2 launch franka_gello_state_publisher main.launch.py config_file:="$GELLO_CFG" &
PID_GELLO=$!

echo "Waiting for /gello/joint_states..."
if ! wait_for_gello_topic 15; then
    echo "No /gello/joint_states received. Check GELLO USB connection and power."
    kill "$PID_GELLO" 2>/dev/null || true
    stop_stale_gello
    exit 1
fi

echo "Launching FR3 controller..."
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:="$FR3_CFG" &
PID_FR3=$!

sleep 2
echo "Launching gripper manager..."
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:="$GRIPPER_CFG" &
PID_GRIPPER=$!

cleanup() {
    echo "Stopping ROS2 nodes..."
    kill "$PID_GELLO" "$PID_FR3" "$PID_GRIPPER" 2>/dev/null || true
    stop_stale_gello
}
trap cleanup INT TERM

wait
