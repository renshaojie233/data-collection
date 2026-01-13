#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_ROOT="${1:-/home/ubuntu/take_data}"
DEST_ROOT="${REPO_ROOT}/local/take_data"

if [ ! -d "$SRC_ROOT" ]; then
  echo "Source not found: $SRC_ROOT" >&2
  exit 1
fi

mkdir -p "$DEST_ROOT"

rsync -av \
  --exclude 'data' \
  --exclude 'lerobot_data' \
  --exclude 'log' \
  --exclude 'logs' \
  --exclude '__pycache__' \
  --exclude '.venv' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude '*.log' \
  --exclude '*.mp4' \
  --exclude '*.avi' \
  --exclude '*.mkv' \
  --exclude 'tmp_*' \
  "$SRC_ROOT/" "$DEST_ROOT/"

echo "Local sync complete: $DEST_ROOT"
