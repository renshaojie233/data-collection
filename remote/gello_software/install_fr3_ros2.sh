#!/bin/bash
set -euo pipefail

ROS_DISTRO="humble"
ROS_VARIANT="${ROS_VARIANT:-ros-base}"  # ros-base or desktop
FRANKA_ROS2_VERSION="v2.0.4"
FRANKA_DESCRIPTION_VERSION="1.0.2"
LIBFRANKA_VERSION="0.18.0"
ROS2_APT_MIRROR="${ROS2_APT_MIRROR:-http://packages.ros.org/ros2/ubuntu}"

FRANKA_WS="$HOME/franka_ros2_ws"
GELLO_WS="$HOME/gello_software/ros2"

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
    # ROS 2 setup files may reference unset vars; disable nounset temporarily.
    set +u
    # shellcheck disable=SC1090
    source "$file"
    set -u
}

echo "Installing ROS 2 $ROS_DISTRO and Franka ROS 2 stack..."

sudo apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=20 -o Acquire::https::Timeout=20 update
sudo apt-get install -y curl gnupg lsb-release

if [ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]; then
    sudo mkdir -p /etc/apt/keyrings
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg >/dev/null
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] ${ROS2_APT_MIRROR} $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=20 -o Acquire::https::Timeout=20 update
sudo apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-$ROS_VARIANT \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    python3-catkin-pkg \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libfmt-dev \
    libpoco-dev \
    ros-$ROS_DISTRO-pinocchio

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

if [ ! -d /usr/local/libfranka-$LIBFRANKA_VERSION ]; then
    echo "Building libfranka $LIBFRANKA_VERSION..."
    tmp_dir=$(mktemp -d)
    git clone --recursive https://github.com/frankarobotics/libfranka.git "$tmp_dir/libfranka"
    cd "$tmp_dir/libfranka"
    git checkout "$LIBFRANKA_VERSION"
    git submodule update --init --recursive
    mkdir -p build && cd build
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local/libfranka-$LIBFRANKA_VERSION
    cmake --build . -j"$(nproc)"
    sudo cmake --install .
    sudo bash -c "echo \"/usr/local/libfranka-$LIBFRANKA_VERSION/lib\" > /etc/ld.so.conf.d/libfranka.conf"
    sudo ldconfig
    cd ~
    rm -rf "$tmp_dir"
fi

if [ ! -d "$FRANKA_WS/src" ]; then
    mkdir -p "$FRANKA_WS/src"
    cd "$FRANKA_WS/src"
    git clone --recursive https://github.com/frankarobotics/franka_ros2.git --branch "$FRANKA_ROS2_VERSION"
    git clone --recursive https://github.com/frankarobotics/franka_description.git --branch "$FRANKA_DESCRIPTION_VERSION"
fi

cd "$FRANKA_WS"
prepare_env
safe_source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
export CMAKE_PREFIX_PATH="/usr/local/libfranka-$LIBFRANKA_VERSION:${CMAKE_PREFIX_PATH:-}"
prepare_env
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

cd "$GELLO_WS"
prepare_env
safe_source /opt/ros/$ROS_DISTRO/setup.bash
safe_source "$FRANKA_WS/install/setup.bash"
rosdep install --from-paths src --ignore-src -r -y
prepare_env
rm -rf build install log
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

echo ""
echo "Install complete."
echo "Next, source the workspaces:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $FRANKA_WS/install/setup.bash"
echo "  source $GELLO_WS/install/setup.bash"
