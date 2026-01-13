#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${REMOTE_HOST:-172.16.1.2}"
REMOTE_USER="${REMOTE_USER:-rsj}"
REMOTE_BASE="${REMOTE_BASE:-/home/rsj}"

SSH_OPTS="-o StrictHostKeyChecking=no"
if [ -n "${SSH_PASS:-}" ]; then
  RSYNC_RSH="sshpass -p ${SSH_PASS} ssh ${SSH_OPTS}"
else
  RSYNC_RSH="ssh ${SSH_OPTS}"
fi

REMOTE_ROOT="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_BASE}"
DEST_ROOT="${REPO_ROOT}/remote"
mkdir -p "$DEST_ROOT"

rsync -av -e "$RSYNC_RSH" \
  --exclude '.git' \
  --exclude '.ros' \
  --exclude '__pycache__' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '*.log' \
  --exclude 'replay_data' \
  --exclude 'data' \
  "${REMOTE_ROOT}/gello_software" "$DEST_ROOT/"

rsync -av -e "$RSYNC_RSH" \
  --exclude '.git' \
  --exclude '.ros' \
  --exclude '__pycache__' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '*.log' \
  --exclude 'replay_data' \
  --exclude 'data' \
  "${REMOTE_ROOT}/franka_cpp_control" "$DEST_ROOT/"

echo "Remote sync complete: $DEST_ROOT"
