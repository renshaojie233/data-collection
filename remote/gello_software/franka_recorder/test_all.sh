#!/bin/bash
# Complete integration test for the recorder/replay system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================================"
echo "Franka Recorder/Replay Complete Integration Test"
echo "============================================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

echo "Step 1: Testing installation..."
python3 test_installation.py || exit 1

echo ""
echo "Step 2: Starting recorder node in background..."
python3 recorder_node.py > /tmp/test_recorder.log 2>&1 &
PID_RECORDER=$!
sleep 2

echo "Step 3: Starting replay node in background..."
python3 replay_node.py > /tmp/test_replay.log 2>&1 &
PID_REPLAY=$!
sleep 2

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $PID_RECORDER $PID_REPLAY 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Step 4: Testing recording and file verification..."
echo "" | python3 test_system.py || exit 1

echo ""
echo "Step 5: Testing replay..."
python3 test_replay.py || exit 1

echo ""
echo "============================================================"
echo "✓ ALL INTEGRATION TESTS PASSED!"
echo "============================================================"
echo ""
echo "The system is fully functional!"
echo ""
echo "To use the system:"
echo "  1. Run: cd ~/gello_software"
echo "  2. Run: ./run_fr3_with_recorder.sh"
echo "  3. Use the GUI to record and replay"
echo ""
