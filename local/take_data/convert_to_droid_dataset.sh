#!/bin/bash
set -euo pipefail

ROOT="/home/ubuntu/take_data"
LOG_PATH="${ROOT}/convert_to_droid_dataset.log"
PYTHON_BIN="/home/ubuntu/anaconda3/bin/python"
export PATH="/home/ubuntu/anaconda3/bin:${PATH}"

notify() {
  if command -v notify-send >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    notify-send "LeRobot DROID Convert" "$1"
  fi
}

total_records=$(ls -d "${ROOT}/data"/record_* 2>/dev/null | wc -l | tr -d ' ')
camera_count=3
convert_total=$((total_records * camera_count))
stats_total=$((total_records * camera_count))
if [ "${total_records}" -eq 0 ]; then
  echo "No records found under ${ROOT}/data" | tee "${LOG_PATH}"
  exit 1
fi
convert_done=0
stats_done=0

progress_fd=""
if command -v zenity >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  progress_pipe=$(mktemp -u)
  mkfifo "${progress_pipe}"
  zenity --progress \
    --title="LeRobot DROID Convert" \
    --text="Starting..." \
    --width=620 \
    --height=140 \
    --percentage=0 \
    --auto-close \
    --no-cancel < "${progress_pipe}" &
  progress_pid=$!
  exec 3> "${progress_pipe}"
  progress_fd="3"
fi

progress_update() {
  local percent="$1"
  local message="$2"
  if [ -n "${progress_fd}" ]; then
    printf "%s\n" "${percent}" >&3
    printf "# %s\n" "${message}" >&3
  fi
}

handle_line() {
  local line="$1"
  echo "${line}" >> "${LOG_PATH}"
  if [[ "${line}" =~ ^PROGRESS[[:space:]]+([a-z_]+)[[:space:]]+([0-9]+)[[:space:]]+([0-9]+)$ ]]; then
    local stage="${BASH_REMATCH[1]}"
    local current="${BASH_REMATCH[2]}"
    local total="${BASH_REMATCH[3]}"
    if [ "${stage}" = "convert_video" ]; then
      convert_done="${current}"
      convert_total="${total}"
      local done=$((convert_done + stats_done))
      local total_steps=$((convert_total + stats_total))
      local percent=$((done * 100 / total_steps))
      progress_update "${percent}" "Converting videos: ${convert_done}/${convert_total} | Overall: ${done}/${total_steps} (${percent}%)"
    elif [ "${stage}" = "stats_video" ]; then
      stats_done="${current}"
      stats_total="${total}"
      local done=$((convert_done + stats_done))
      local total_steps=$((convert_total + stats_total))
      local percent=$((done * 100 / total_steps))
      progress_update "${percent}" "Computing stats: ${stats_done}/${stats_total} | Overall: ${done}/${total_steps} (${percent}%)"
    fi
  fi
}

echo "Starting conversion at $(date)" | tee "${LOG_PATH}"
notify "Conversion started. See ${LOG_PATH} for progress."

progress_update 0 "Converting videos: 0/${convert_total} | Overall: 0/$((convert_total + stats_total)) (0%)"
while IFS= read -r line; do
  handle_line "${line}"
done < <("${PYTHON_BIN}" "${ROOT}/convert_to_droid_lerobot_v30.py" 2>&1)

echo "Done at $(date)" | tee -a "${LOG_PATH}"
notify "Conversion finished."

if [ -n "${progress_fd}" ]; then
  exec 3>&-
  rm -f "${progress_pipe}"
  wait "${progress_pid}" || true
fi
