#!/bin/bash
#
# Start Robot Data Collection
# This script:
# 1. Starts run_fr3_real_ros2_robotiq.sh on remote machine
# 2. Uploads and runs data bridge on remote machine
# 3. Runs local data receiver to collect and save data
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Configuration
REMOTE_HOST="172.16.1.2"
REMOTE_USER="rsj"
REMOTE_PASSWORD="2064027038"
REMOTE_PORT=22
TCP_PORT=9999

# Paths
REMOTE_SCRIPT="/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh"
REMOTE_SCRIPT_LOG="/tmp/$(basename "$REMOTE_SCRIPT" .sh).log"
REMOTE_BRIDGE_DIR="/tmp/robot_data_bridge"
LOCAL_RECEIVER="$SCRIPT_DIR/local_data_receiver.py"
REMOTE_BRIDGE="$SCRIPT_DIR/remote_data_bridge.py"
DATA_DIR="$PROJECT_DIR/data"
LOG_DIR="$PROJECT_DIR/logs"

# Create directories
mkdir -p "$DATA_DIR" "$LOG_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Global flag to track if cleanup has been done
CLEANUP_DONE=0

echo ""
echo "============================================"
echo "  Robot Data Collection System"
echo "============================================"
echo ""

# Function to run SSH commands
run_ssh() {
    sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" "$@" 2>/dev/null || true
}

# Function to upload file
upload_file() {
    local src="$1"
    local dst="$2"
    sshpass -p "$REMOTE_PASSWORD" scp -o StrictHostKeyChecking=no "$src" "$REMOTE_USER@$REMOTE_HOST:$dst" 2>/dev/null
}

# Cleanup function - will be called on exit
cleanup() {
    # Prevent multiple cleanup calls
    if [ $CLEANUP_DONE -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1

    local exit_code=$?

    echo ""
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}  正在清理远程进程...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    # Kill remote processes - be thorough
    echo "  停止数据桥接器..."
    run_ssh "pkill -9 -f 'remote_data_bridge.py'"

    echo "  停止ROS2控制系统..."
    run_ssh "pkill -9 -f 'run_fr3_real_ros2_robotiq.sh'"
    run_ssh "pkill -9 -f 'gello_publisher'"
    run_ssh "pkill -9 -f 'franka_fr3_arm_controllers'"
    run_ssh "pkill -9 -f 'franka_gripper'"
    run_ssh "pkill -9 -f 'ros2'"

    # Clean up remote directory
    run_ssh "rm -rf $REMOTE_BRIDGE_DIR"

    echo ""
    echo -e "${GREEN}✓ 远程进程已清理${NC}"
    echo -e "${GREEN}✓ 数据已保存到: $DATA_DIR${NC}"
    echo ""

    # Restore original exit code
    exit $exit_code
}

# Set trap for cleanup - will be called on ANY exit
trap cleanup EXIT INT TERM QUIT

# Step 1: Check remote connection
echo -e "${YELLOW}[1/5] Checking remote connection...${NC}"
if ping -c 1 -W 2 "$REMOTE_HOST" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Remote host reachable${NC}"
else
    echo -e "${RED}✗ Cannot reach remote host $REMOTE_HOST${NC}"
    exit 1
fi

# Step 2: Upload bridge script to remote
echo -e "${YELLOW}[2/5] Uploading data bridge to remote...${NC}"
run_ssh "mkdir -p $REMOTE_BRIDGE_DIR"
upload_file "$REMOTE_BRIDGE" "$REMOTE_BRIDGE_DIR/remote_data_bridge.py"
run_ssh "chmod +x $REMOTE_BRIDGE_DIR/remote_data_bridge.py"
echo -e "${GREEN}✓ Bridge script uploaded${NC}"

# Step 3: Start remote ROS2 system
echo -e "${YELLOW}[3/5] Starting remote ROS2 system...${NC}"
LOG_FILE="$LOG_DIR/ros2_output_$(date +%Y%m%d_%H%M%S).log"
echo "  Output will be logged to: $LOG_FILE"

# Start ROS2 script in background
sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" \
    "nohup $REMOTE_SCRIPT > $REMOTE_SCRIPT_LOG 2>&1 &" &

echo "  Waiting for ROS2 system to initialize (15 seconds)..."
sleep 15

# Check if system is running
if run_ssh "pgrep -f 'ros2' > /dev/null"; then
    echo -e "${GREEN}✓ ROS2 system started${NC}"
else
    echo -e "${RED}✗ Failed to start ROS2 system${NC}"
    echo "Check log: ssh $REMOTE_USER@$REMOTE_HOST 'cat $REMOTE_SCRIPT_LOG'"
    exit 1
fi

# Step 4: Start data bridge on remote
echo -e "${YELLOW}[4/5] Starting remote data bridge...${NC}"

sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" \
    "cd $REMOTE_BRIDGE_DIR && source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash && source ~/gello_software/ros2/install/setup.bash && nohup python3 remote_data_bridge.py > /tmp/bridge.log 2>&1 &" &

echo "  Waiting for bridge to start (5 seconds)..."
sleep 5

if run_ssh "pgrep -f 'remote_data_bridge.py' > /dev/null"; then
    echo -e "${GREEN}✓ Data bridge started on port $TCP_PORT${NC}"
else
    echo -e "${RED}✗ Failed to start data bridge${NC}"
    echo "Check log: ssh $REMOTE_USER@$REMOTE_HOST 'cat /tmp/bridge.log'"
    exit 1
fi

# Step 5: Start local data receiver
echo -e "${YELLOW}[5/5] Starting local data receiver...${NC}"
echo ""
echo "============================================"
echo "  Collection in progress..."
echo "  Press Ctrl+C to stop and save data"
echo "============================================"
echo ""

cd "$SCRIPT_DIR"
python3 local_data_receiver.py --host "$REMOTE_HOST" --port "$TCP_PORT" --save-dir "$DATA_DIR"

# Cleanup is handled by trap
