#!/bin/bash
# Launch script for Franka recorder and replay nodes with GUI
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2 environment
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "Error: ROS2 $ROS_DISTRO not found"
    exit 1
fi

# Make scripts executable
chmod +x recorder_node.py
chmod +x replay_node.py
chmod +x control_gui.py

echo "============================================================"
echo "Franka Recorder & Replay System"
echo "============================================================"
echo ""
echo "Starting recorder node..."
python3 recorder_node.py &
PID_RECORDER=$!

echo "Starting replay node..."
python3 replay_node.py &
PID_REPLAY=$!

# Wait a moment for nodes to initialize
sleep 2

echo "Starting GUI control panel..."
python3 control_gui.py &
PID_GUI=$!

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down recorder and replay system..."
    kill $PID_RECORDER $PID_REPLAY $PID_GUI 2>/dev/null || true
    wait 2>/dev/null || true
    echo "Shutdown complete"
}

trap cleanup INT TERM EXIT

# Wait for GUI to close
wait $PID_GUI
