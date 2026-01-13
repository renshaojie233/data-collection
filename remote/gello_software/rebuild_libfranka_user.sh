#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.18.0}"
PREFIX="${LIBFRANKA_PREFIX:-$HOME/.local/libfranka-$LIBFRANKA_VERSION}"

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

prepare_env() {
    if [ -n "${PATH:-}" ]; then
        export PATH
        PATH="$(sanitize_path "$PATH")"
    fi
    if [ -n "${LD_LIBRARY_PATH:-}" ]; then
        export LD_LIBRARY_PATH
        LD_LIBRARY_PATH="$(sanitize_path "$LD_LIBRARY_PATH")"
    fi
    if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
        export CMAKE_PREFIX_PATH
        CMAKE_PREFIX_PATH="$(sanitize_path "$CMAKE_PREFIX_PATH")"
    fi
    if [ -n "${PKG_CONFIG_PATH:-}" ]; then
        export PKG_CONFIG_PATH
        PKG_CONFIG_PATH="$(sanitize_path "$PKG_CONFIG_PATH")"
    fi
    unset PYTHONPATH
}

prepare_env
export CMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO:/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="/opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

tmp_dir=$(mktemp -d)
cleanup() {
    rm -rf "$tmp_dir"
}
trap cleanup EXIT

echo "Building libfranka $LIBFRANKA_VERSION into $PREFIX"
git clone --recursive https://github.com/frankarobotics/libfranka.git "$tmp_dir/libfranka"
cd "$tmp_dir/libfranka"
git checkout "$LIBFRANKA_VERSION"
git submodule update --init --recursive
mkdir -p build
cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="$PREFIX"

cmake --build . -j"$(nproc)"
cmake --install .

echo ""
echo "libfranka installed to $PREFIX"
