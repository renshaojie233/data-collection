#!/bin/bash
set -euo pipefail

DATASET_PATH="${1:-/home/ubuntu/take_data/lerobot_data/lerobot_take_droid_pi05_15fps}"
RERUN_BIN="/home/ubuntu/take_data/lerobot_viewer/.venv/bin/rerun"

# Clear persisted viewer state to avoid stale blueprints or view configs.
RERUN_ANALYTICS=off "$RERUN_BIN" reset || true
RERUN_ANALYTICS=off "$RERUN_BIN" "$DATASET_PATH"
