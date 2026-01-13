#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-172.16.0.2}"
JSON_PATH="${2:-}"
JSON_DIR="/home/rsj/franka_cpp_control/replay_data/action"

if [[ -z "${JSON_PATH}" ]]; then
  JSON_PATH="$(find "${JSON_DIR}" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi

if [[ -z "${JSON_PATH}" ]]; then
  echo "No JSON files found in ${JSON_DIR}"
  exit 1
fi

TIME_SCALE="${TIME_SCALE:-1.5}"
START_SPEED="${START_SPEED:-0.1}"
K_GAIN="${K_GAIN:-30}"
D_GAIN="${D_GAIN:-5}"
K_ALPHA="${K_ALPHA:-0.3}"
Q_ALPHA="${Q_ALPHA:-0.2}"
GRIPPER_SPEED="${GRIPPER_SPEED:-0.1}"
GRIPPER_FORCE="${GRIPPER_FORCE:-20.0}"

# Create output directory with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="/home/rsj/franka_cpp_control/comparison_results/${TIMESTAMP}"
mkdir -p "${OUTPUT_DIR}"

RECORDED_JSON="${OUTPUT_DIR}/recorded_trajectory.json"

LIB_PATH="/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib"
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH:-}"

echo "========================================="
echo "Running trajectory tracking with recording"
echo "========================================="
echo "Robot IP: ${ROBOT_IP}"
echo "Reference JSON: ${JSON_PATH}"
echo "Time scale: ${TIME_SCALE}"
echo "Output directory: ${OUTPUT_DIR}"
echo ""

# Run the recording program
/home/rsj/franka_cpp_control/build/franka_track_json_impedance_gripper_record \
  "${ROBOT_IP}" \
  "${JSON_PATH}" \
  "${TIME_SCALE}" \
  "${START_SPEED}" \
  "${K_GAIN}" \
  "${D_GAIN}" \
  "${K_ALPHA}" \
  "${Q_ALPHA}" \
  "${GRIPPER_SPEED}" \
  "${GRIPPER_FORCE}" \
  "${RECORDED_JSON}"

RECORD_STATUS=$?

if [ ${RECORD_STATUS} -ne 0 ]; then
  echo "Error: Recording failed with status ${RECORD_STATUS}"
  exit ${RECORD_STATUS}
fi

echo ""
echo "========================================="
echo "Generating comparison plots"
echo "========================================="
echo ""

# Run comparison script
/home/rsj/franka_cpp_control/compare_trajectories.py \
  --reference "${JSON_PATH}" \
  --recorded "${RECORDED_JSON}" \
  --output-dir "${OUTPUT_DIR}"

COMPARE_STATUS=$?

if [ ${COMPARE_STATUS} -ne 0 ]; then
  echo "Error: Comparison failed with status ${COMPARE_STATUS}"
  exit ${COMPARE_STATUS}
fi

echo ""
echo "========================================="
echo "Results saved to: ${OUTPUT_DIR}"
echo "  - recorded_trajectory.json"
echo "  - trajectory_comparison.png"
echo "  - tracking_errors.png"
echo "========================================="
