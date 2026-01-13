#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="${OPENPI_REMOTE_CONFIG:-/home/ubuntu/take_data/take_video_action/video_action_recorder.py}"
REMOTE_HOST="${OPENPI_REMOTE_HOST:-}"
REMOTE_USER="${OPENPI_REMOTE_USER:-}"
REMOTE_PASSWORD="${OPENPI_REMOTE_PASSWORD:-}"

if [[ -z "$REMOTE_HOST" || -z "$REMOTE_USER" || -z "$REMOTE_PASSWORD" ]]; then
  if [[ -f "$CONFIG_FILE" ]]; then
    readarray -t vals < <(python3 - <<'PY'
import re
from pathlib import Path
path = Path("/home/ubuntu/take_data/take_video_action/video_action_recorder.py")
text = path.read_text(encoding="utf-8")
for key in ("REMOTE_HOST", "REMOTE_USER", "REMOTE_PASSWORD"):
    m = re.search(rf"^{key}\\s*=\\s*\"([^\"]+)\"", text, re.M)
    print(m.group(1) if m else "")
PY
)
    if [[ -z "$REMOTE_HOST" ]]; then REMOTE_HOST="${vals[0]}"; fi
    if [[ -z "$REMOTE_USER" ]]; then REMOTE_USER="${vals[1]}"; fi
    if [[ -z "$REMOTE_PASSWORD" ]]; then REMOTE_PASSWORD="${vals[2]}"; fi
  fi
fi

if [[ -z "$REMOTE_HOST" || -z "$REMOTE_USER" ]]; then
  echo "Missing remote host/user. Set OPENPI_REMOTE_HOST/OPENPI_REMOTE_USER." >&2
  exit 1
fi

ACTION_PORT="${OPENPI_ACTION_PORT:-15123}"
STATE_PORT="${OPENPI_STATE_PORT:-15124}"
ACTION_HOST="${OPENPI_ACTION_HOST:-$REMOTE_HOST}"
STATE_HOST="${OPENPI_STATE_HOST:-$REMOTE_HOST}"
POLICY_HOST="${OPENPI_POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${OPENPI_POLICY_PORT:-8000}"
PROMPT="${OPENPI_PROMPT:-do something}"
VEL_SCALE="${OPENPI_VEL_SCALE:-0.2}"
ROBOTIQ_PORT="${OPENPI_ROBOTIQ_PORT:-/dev/robotiq}"

if ! ss -ltn 2>/dev/null | grep -q ":${POLICY_PORT} "; then
  echo "Warning: policy server not detected on port ${POLICY_PORT}." >&2
fi

SSH_BASE=(ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}")
if [[ -n "$REMOTE_PASSWORD" ]] && ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass not found but REMOTE_PASSWORD is set." >&2
  exit 1
fi

if [[ -n "$REMOTE_PASSWORD" ]]; then
  SSH_BASE=(sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}")
fi

REMOTE_CMD=(
  "ROBOTIQ_PORT=${ROBOTIQ_PORT}"
  "ACTION_PORT=${ACTION_PORT}"
  "STATE_PORT=${STATE_PORT}"
  "VEL_SCALE=${VEL_SCALE}"
  "/home/rsj/franka_cpp_control/start_openpi_stream.sh"
  "start"
)

"${SSH_BASE[@]}" "${REMOTE_CMD[*]}"

exec conda run -n take_data python /home/ubuntu/take_data/take_video_action/openpi_infer_client.py \
  --policy-host "${POLICY_HOST}" \
  --policy-port "${POLICY_PORT}" \
  --action-host "${ACTION_HOST}" \
  --action-port "${ACTION_PORT}" \
  --state-host "${STATE_HOST}" \
  --state-port "${STATE_PORT}" \
  --prompt "${PROMPT}" \
  "$@"
