#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.18.0}"
LIBFRANKA_PREFIX="${LIBFRANKA_PREFIX:-}"
FRANKA_WS="${FRANKA_WS:-$HOME/franka_ros2_ws}"
GELLO_WS="${GELLO_WS:-$HOME/gello_software/ros2}"

sanitize_ld_library_path() {
    local path="${1:-}"
    local filtered=""
    IFS=":" read -r -a parts <<< "$path"
    for p in "${parts[@]}"; do
        case "$p" in
            *"/anaconda3/"*|*"/miniconda3/"*|*"/conda/"*) ;;
            "") ;;
            *) filtered="${filtered:+$filtered:}$p" ;;
        esac
    done
    echo "$filtered"
}

sanitize_path() {
    local path="${1:-}"
    local filtered=""
    IFS=":" read -r -a parts <<< "$path"
    for p in "${parts[@]}"; do
        case "$p" in
            *"/anaconda3/"*|*"/miniconda3/"*|*"/conda/"*) ;;
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
            *"/anaconda3/"*|*"/miniconda3/"*|*"/conda/"*) ;;
            "") ;;
            *) filtered="${filtered:+$filtered:}$p" ;;
        esac
    done
    echo "$filtered"
}

prepare_env() {
    unset LD_PRELOAD
    if [ -n "${LD_LIBRARY_PATH:-}" ]; then
        export LD_LIBRARY_PATH
        LD_LIBRARY_PATH="$(sanitize_ld_library_path "$LD_LIBRARY_PATH")"
    fi
    if [ -n "${PATH:-}" ]; then
        export PATH
        PATH="$(sanitize_path "$PATH")"
    fi
    if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
        export CMAKE_PREFIX_PATH
        CMAKE_PREFIX_PATH="$(sanitize_path "$CMAKE_PREFIX_PATH")"
    fi
    if [ -n "${PKG_CONFIG_PATH:-}" ]; then
        export PKG_CONFIG_PATH
        PKG_CONFIG_PATH="$(sanitize_path "$PKG_CONFIG_PATH")"
    fi
    if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
        export AMENT_PREFIX_PATH
        AMENT_PREFIX_PATH="$(sanitize_path "$AMENT_PREFIX_PATH")"
    fi
    if [ -n "${PYTHONPATH:-}" ]; then
        export PYTHONPATH
        PYTHONPATH="$(sanitize_pythonpath "$PYTHONPATH")"
    fi
    export PYTHON_EXECUTABLE=/usr/bin/python3
    export Python3_EXECUTABLE=/usr/bin/python3
}

safe_source() {
    local file="$1"
    set +u
    # shellcheck disable=SC1090
    source "$file"
    set -u
}

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Missing /opt/ros/$ROS_DISTRO/setup.bash"
    exit 1
fi
if [ ! -d "$FRANKA_WS/src" ]; then
    echo "Missing Franka workspace: $FRANKA_WS/src"
    exit 1
fi
if [ ! -d "$GELLO_WS/src" ]; then
    echo "Missing GELLO ROS2 workspace: $GELLO_WS/src"
    exit 1
fi

echo "Rebuilding Franka ROS2 workspace..."
cd "$FRANKA_WS"
prepare_env
safe_source /opt/ros/$ROS_DISTRO/setup.bash
prepare_env
if [ -z "$LIBFRANKA_PREFIX" ]; then
    if [ -d "$HOME/.local/libfranka-$LIBFRANKA_VERSION" ]; then
        LIBFRANKA_PREFIX="$HOME/.local/libfranka-$LIBFRANKA_VERSION"
    else
        LIBFRANKA_PREFIX="/usr/local/libfranka-$LIBFRANKA_VERSION"
    fi
fi
export CMAKE_PREFIX_PATH="$LIBFRANKA_PREFIX:${CMAKE_PREFIX_PATH:-}"
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

echo "Rebuilding GELLO ROS2 workspace..."
cd "$GELLO_WS"
prepare_env
safe_source /opt/ros/$ROS_DISTRO/setup.bash
safe_source "$FRANKA_WS/install/setup.bash"
prepare_env
rm -rf build install log
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

echo ""
echo "Rebuild complete."
echo "Next, source the workspaces:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $FRANKA_WS/install/setup.bash"
echo "  source $GELLO_WS/install/setup.bash"
