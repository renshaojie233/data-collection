#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ENV_NAME="${GELLO_ENV_NAME:-gello_software}"
CONDA_SH="${GELLO_CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
CFG="configs/fr3_sim_gello_panda.yaml"
CONDA_NO_PLUGINS="${CONDA_NO_PLUGINS:-true}"
CONDA_SOLVER="${CONDA_SOLVER:-classic}"
# Avoid conda CUDA probe using multiprocessing which can fail on some systems.
export CONDA_OVERRIDE_CUDA="${CONDA_OVERRIDE_CUDA:-0}"

run_conda() {
    local proxy_env=()
    if [ -n "${http_proxy:-}${HTTP_PROXY:-}" ]; then
        if [[ "${http_proxy:-}${HTTP_PROXY:-}" == *"127.0.0.1:7890"* ]]; then
            if command -v nc >/dev/null 2>&1 && nc -z 127.0.0.1 7890 >/dev/null 2>&1; then
                proxy_env=()
            else
                proxy_env=("HTTP_PROXY=" "HTTPS_PROXY=" "http_proxy=" "https_proxy=" "FTP_PROXY=" "ftp_proxy=" "ALL_PROXY=" "all_proxy=")
            fi
        fi
    fi
    "${proxy_env[@]}" conda --no-plugins "$@"
}


# Prefer system Mesa drivers even if conda base is active.
export LIBGL_DRIVERS_PATH="${LIBGL_DRIVERS_PATH:-/usr/lib/x86_64-linux-gnu/dri}"
if [ -n "${DISPLAY:-}" ]; then
    if command -v xhost >/dev/null 2>&1 && ! xhost >/dev/null 2>&1; then
        echo "Warning: cannot access X11 display ${DISPLAY}. No window will appear."
        echo "Try: export XAUTHORITY=~/.Xauthority and run 'xhost +SI:localuser:$USER'"
    fi
    export MUJOCO_GL=glfw
else
    export MUJOCO_GL=egl
fi

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

if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH
    LD_LIBRARY_PATH="$(sanitize_ld_library_path "$LD_LIBRARY_PATH")"
fi

if [ -f "$CONDA_SH" ]; then
    # shellcheck disable=SC1090
    source "$CONDA_SH"
else
    echo "Error: conda not found at $CONDA_SH"
    echo "Set GELLO_CONDA_SH if your conda path differs."
    exit 1
fi

CONDA_ROOT="$(cd "$(dirname "$CONDA_SH")/../.." && pwd)"
ENV_DIR="$CONDA_ROOT/envs/$ENV_NAME"
FALLBACK_ENV="${GELLO_ENV_FALLBACK:-gello_control}"
FALLBACK_DIR="$CONDA_ROOT/envs/$FALLBACK_ENV"

if [ ! -d "$ENV_DIR" ] && [ -d "$FALLBACK_DIR" ]; then
    echo "Conda env '$ENV_NAME' not found. Falling back to '$FALLBACK_ENV'."
    ENV_NAME="$FALLBACK_ENV"
    ENV_DIR="$FALLBACK_DIR"
fi

if [ ! -d "$ENV_DIR" ]; then
    echo "Conda env '$ENV_NAME' not found. Creating and installing dependencies..."
    run_conda create --solver "$CONDA_SOLVER" -n "$ENV_NAME" -c conda-forge python=3.11 -y
    conda activate "$ENV_NAME"
    python -m pip install -U pip
    python -m pip install numpy pyzmq omegaconf tyro termcolor mujoco dm_control pyserial
    python -m pip install -e .
    python -m pip install -e third_party/DynamixelSDK/python
else
    conda activate "$ENV_NAME"
fi

# Re-sanitize after conda activation in case base env re-added paths.
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH
    LD_LIBRARY_PATH="$(sanitize_ld_library_path "$LD_LIBRARY_PATH")"
fi

if [ ! -d "third_party/mujoco_menagerie" ] || [ -z "$(ls -A third_party/mujoco_menagerie 2>/dev/null)" ]; then
    git submodule update --init --recursive
fi
if [ ! -d "third_party/DynamixelSDK" ] || [ -z "$(ls -A third_party/DynamixelSDK 2>/dev/null)" ]; then
    git submodule update --init --recursive
fi

if [ ! -f "$CFG" ]; then
    echo "Error: missing config $CFG"
    exit 1
fi

if command -v rg >/dev/null 2>&1; then
    HAS_PLACEHOLDER=$(rg -q "FTXXXXXXXX" "$CFG" && echo "yes" || echo "no")
else
    HAS_PLACEHOLDER=$(grep -q "FTXXXXXXXX" "$CFG" && echo "yes" || echo "no")
fi
if [ "$HAS_PLACEHOLDER" = "yes" ]; then
    echo "Error: update agent.port in $CFG before running."
    echo "Available serial ports:"
    ls -la /dev/serial/by-id/ 2>/dev/null || echo "  (no /dev/serial/by-id found)"
    exit 1
fi

echo "Launching FR3 sim with config: $CFG"
echo "Tips:"
echo "  - Ctrl+C to stop"
echo "  - MUJOCO_GL=egl for headless systems"
echo ""

export PYTHONPATH="$SCRIPT_DIR:$SCRIPT_DIR/third_party/DynamixelSDK/python/src${PYTHONPATH:+:$PYTHONPATH}"

PYTHONUNBUFFERED=1 env -u LD_LIBRARY_PATH -u LD_PRELOAD \
    python -u experiments/launch_yaml.py --left-config-path "$CFG"
