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

stop_stale_relative() {
    local pids=""
    pids="$(pgrep -f 'gello_relative_publisher.py' || true)"
    if [ -n "$pids" ]; then
        echo "Stopping existing gello_relative_publisher processes: $pids"
        kill $pids 2>/dev/null || true
        sleep 1
    fi
}

set_rt_priority_for_ros2_control() {
    local pgid="$1"
    local prio="${2:-80}"
    local pid=""
    if ! command -v chrt >/dev/null 2>&1; then
        echo "Warning: chrt not found; skipping RT priority setup"
        return 1
    fi
    for _ in $(seq 1 30); do
        pid="$(pgrep -g "$pgid" -f 'ros2_control_node' | head -n 1)"
        if [ -n "$pid" ]; then
            if chrt -f -p "$prio" "$pid" >/dev/null 2>&1; then
                echo "Set ros2_control_node RT priority to ${prio} (pid=${pid})"
                return 0
            fi
            echo "Warning: failed to set RT priority for ros2_control_node (pid=${pid})"
            return 1
        fi
        sleep 0.2
    done
    echo "Warning: ros2_control_node not found for RT priority setup"
    return 1
}

pin_cpu_for_ros2_control() {
    local pgid="$1"
    local cpu="$2"
    local pid=""
    if [ -z "$cpu" ]; then
        return 0
    fi
    if ! command -v taskset >/dev/null 2>&1; then
        echo "Warning: taskset not found; skipping CPU pin"
        return 1
    fi
    for _ in $(seq 1 30); do
        pid="$(pgrep -g "$pgid" -f 'ros2_control_node' | head -n 1)"
        if [ -n "$pid" ]; then
            if taskset -pc "$cpu" "$pid" >/dev/null 2>&1; then
                echo "Pinned ros2_control_node to CPU ${cpu} (pid=${pid})"
                return 0
            fi
            echo "Warning: failed to pin ros2_control_node to CPU ${cpu} (pid=${pid})"
            return 1
        fi
        sleep 0.2
    done
    echo "Warning: ros2_control_node not found for CPU pin"
    return 1
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
        if ! is_same_device "/dev/gello" "${ROBOTIQ_PORT:-}"; then
            echo "/dev/gello"
            return 0
        fi
    fi
    for pattern in /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter* /dev/serial/by-id/usb-ROBOTIS_OpenRB-150*; do
        if [ -e "$pattern" ] && ! is_same_device "$pattern" "${ROBOTIQ_PORT:-}"; then
            matches+=("$pattern")
        fi
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
        if [ -e "$pattern" ] && ! is_same_device "$pattern" "${ROBOTIQ_PORT:-}"; then
            matches+=("$pattern")
        fi
    done
    if [ "${#matches[@]}" -eq 1 ]; then
        echo "${matches[0]}"
        return 0
    fi
    return 1
}

is_same_device() {
    local a="${1:-}"
    local b="${2:-}"
    local ra rb
    [ -n "$a" ] || return 1
    [ -n "$b" ] || return 1
    if [ "$a" = "$b" ]; then
        return 0
    fi
    ra="$(readlink -f "$a" 2>/dev/null || echo "$a")"
    rb="$(readlink -f "$b" 2>/dev/null || echo "$b")"
    [ "$ra" = "$rb" ]
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
            if is_same_device "$port" "${ROBOTIQ_PORT:-}"; then
                echo "Skipping Robotiq port $port; waiting for GELLO device..." >&2
            else
            echo "$port"
            return 0
            fi
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

update_gripper_config_port() {
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

ROS_DISTRO="${ROS_DISTRO:-humble}"
FRANKA_WS="${FRANKA_WS:-$HOME/franka_ros2_ws}"
GELLO_WS="${GELLO_WS:-$HOME/gello_software/ros2}"
LIBFRANKA_PREFIX="${LIBFRANKA_PREFIX:-}"

GELLO_CFG="${GELLO_CFG:-fr3_rsjt.yaml}"
RELATIVE_CFG="${RELATIVE_CFG:-$HOME/gello_software/configs/relative_gello_control.yaml}"
RELATIVE_NODE="${RELATIVE_NODE:-$HOME/gello_software/scripts/gello_relative_publisher.py}"
FR3_CFG="${FR3_CFG:-fr3_rsjt_robotiq.yaml}"
GRIPPER_CFG="${GRIPPER_CFG:-example_fr3_config_robotiq.yaml}"
ROBOTIQ_PORT="${ROBOTIQ_PORT:-}"
ROBOTIQ_PREFIX="${ROBOTIQ_PREFIX:-$GELLO_WS/install}"
CPU_COUNT="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"
DEFAULT_RT_CPU=""
if [ "$CPU_COUNT" -ge 4 ]; then
    DEFAULT_RT_CPU="2"
elif [ "$CPU_COUNT" -ge 2 ]; then
    DEFAULT_RT_CPU="1"
fi
ROS2_CONTROL_CPU="${ROS2_CONTROL_CPU:-$DEFAULT_RT_CPU}"

GELLO_CFG_SRC="$GELLO_WS/src/franka_gello_state_publisher/config/$GELLO_CFG"
GELLO_CFG_INSTALL="$GELLO_WS/install/franka_gello_state_publisher/share/franka_gello_state_publisher/config/$GELLO_CFG"
FR3_CFG_SRC="$GELLO_WS/src/franka_fr3_arm_controllers/config/$FR3_CFG"
FR3_CFG_INSTALL="$GELLO_WS/install/franka_fr3_arm_controllers/share/franka_fr3_arm_controllers/config/$FR3_CFG"
GRIPPER_CFG_SRC="$GELLO_WS/src/franka_gripper_manager/config/$GRIPPER_CFG"
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

if [ -z "$ROBOTIQ_PORT" ]; then
    ROBOTIQ_PORT="$(detect_robotiq_port || true)"
fi

AUTO_PORT="$(wait_for_gello_port | tail -n 1)"
update_gello_config_port "$GELLO_CFG_SRC" "$AUTO_PORT"
update_gello_config_port "$GELLO_CFG_INSTALL" "$AUTO_PORT"

if [ -n "$ROBOTIQ_PORT" ]; then
    update_gripper_config_port "$GRIPPER_CFG_SRC" "$ROBOTIQ_PORT"
    update_gripper_config_port "$GRIPPER_CFG_INSTALL" "$ROBOTIQ_PORT"
fi

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

if [ -d "$ROBOTIQ_PREFIX/share/robotiq_description" ] || [ -d "$ROBOTIQ_PREFIX/share/robotiq_controllers" ]; then
    export AMENT_PREFIX_PATH="$ROBOTIQ_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$ROBOTIQ_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$ROBOTIQ_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PATH="$ROBOTIQ_PREFIX/bin${PATH:+:$PATH}"
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
if [ -n "$ROBOTIQ_PORT" ]; then
    echo "  Robotiq port: $ROBOTIQ_PORT"
fi
echo ""

stop_stale_gello
stop_stale_relative

PID_GELLO=""
PID_FR3=""
PID_GRIPPER=""
PGID_GELLO=""
PGID_FR3=""
PGID_GRIPPER=""
WATCHER_PID=""

echo "Launching relative GELLO publisher..."
setsid python3 "$RELATIVE_NODE" --gello-config "$GELLO_CFG_PATH" --relative-config "$RELATIVE_CFG" &
PID_GELLO=$!
PGID_GELLO="$PID_GELLO"

echo "Waiting for /gello/joint_states..."
if ! wait_for_gello_topic 15; then
    echo "No /gello/joint_states received. Check GELLO USB connection and power."
    kill "$PID_GELLO" 2>/dev/null || true
    stop_stale_gello
    exit 1
fi

echo "Launching FR3 controller..."
setsid ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:="$FR3_CFG" &
PID_FR3=$!
PGID_FR3="$PID_FR3"

if [ "${ENABLE_RT:-1}" = "1" ]; then
    set_rt_priority_for_ros2_control "$PGID_FR3" "${RT_PRIORITY:-80}" || true
fi
if [ "${PIN_CPU:-1}" = "1" ] && [ -n "${ROS2_CONTROL_CPU:-}" ]; then
    pin_cpu_for_ros2_control "$PGID_FR3" "$ROS2_CONTROL_CPU" || true
fi

ENABLE_SERVICE="/gello_relative/enable"

enable_relative_control() {
    local attempts=0
    local response=""
    while [ $attempts -lt 40 ]; do
        response="$(ros2 service call "$ENABLE_SERVICE" std_srvs/srv/SetBool "{data: true}" 2>/dev/null || true)"
        if [ -n "$response" ]; then
            if command -v rg >/dev/null 2>&1; then
                echo "$response" | rg -q "success[=:][[:space:]]*(True|true)" && {
                    echo "Relative control enabled."
                    return 0
                }
            else
                echo "$response" | grep -q "success[=:][[:space:]]*\(True\|true\)" && {
                    echo "Relative control enabled."
                    return 0
                }
            fi
        fi
        attempts=$((attempts + 1))
        sleep 0.5
    done
    echo "Warning: failed to enable relative control."
    if [ -n "$response" ]; then
        echo "Last enable response: $response"
    fi
    echo "Run manually: ros2 service call $ENABLE_SERVICE std_srvs/srv/SetBool '{data: true}'"
    return 1
}

if [ "${RELATIVE_AUTO_ENABLE:-0}" = "1" ]; then
    echo "Auto-enabling relative control..."
    enable_relative_control
else
    if [ -t 0 ]; then
        echo ""
        echo "Robot is aligning to fixed pose."
        echo "Type y and press ENTER to enable relative control..."
        while true; do
            read -r confirm
            case "$confirm" in
                y|Y)
                    enable_relative_control
                    break
                    ;;
                *)
                    echo "Waiting for 'y'..."
                    ;;
            esac
        done
    else
        echo ""
        echo "Robot is aligning to fixed pose."
        echo "No TTY available, skipping auto-enable."
        echo "Run manually: ros2 service call $ENABLE_SERVICE std_srvs/srv/SetBool '{data: true}'"
    fi
fi

sleep 2
echo "Launching Robotiq gripper manager..."
setsid ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py config_file:="$GRIPPER_CFG" &
PID_GRIPPER=$!
PGID_GRIPPER="$PID_GRIPPER"

start_cleanup_watcher() {
    # Detached watcher ensures child groups are terminated if this script dies abruptly.
    local parent_pid="$$"
    local pgids=()
    [ -n "${PGID_GELLO:-}" ] && pgids+=("$PGID_GELLO")
    [ -n "${PGID_FR3:-}" ] && pgids+=("$PGID_FR3")
    [ -n "${PGID_GRIPPER:-}" ] && pgids+=("$PGID_GRIPPER")
    setsid bash -c '
        parent="$1"
        shift
        while kill -0 "$parent" 2>/dev/null; do
            sleep 1
        done
        for pg in "$@"; do
            [ -n "$pg" ] && kill -TERM -- "-$pg" 2>/dev/null || true
        done
    ' _ "$parent_pid" "${pgids[@]}" >/dev/null 2>&1 &
    WATCHER_PID=$!
}

start_cleanup_watcher

cleanup() {
    echo "Stopping ROS2 nodes..."
    if [ -n "${PGID_GELLO:-}" ]; then
        kill -TERM -- "-$PGID_GELLO" 2>/dev/null || true
    elif [ -n "${PID_GELLO:-}" ]; then
        kill -TERM "$PID_GELLO" 2>/dev/null || true
    fi
    if [ -n "${PGID_FR3:-}" ]; then
        kill -TERM -- "-$PGID_FR3" 2>/dev/null || true
    elif [ -n "${PID_FR3:-}" ]; then
        kill -TERM "$PID_FR3" 2>/dev/null || true
    fi
    if [ -n "${PGID_GRIPPER:-}" ]; then
        kill -TERM -- "-$PGID_GRIPPER" 2>/dev/null || true
    elif [ -n "${PID_GRIPPER:-}" ]; then
        kill -TERM "$PID_GRIPPER" 2>/dev/null || true
    fi
    stop_stale_gello
}
trap cleanup EXIT HUP INT TERM

wait
