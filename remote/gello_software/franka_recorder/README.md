# Franka Robot Recorder & Replay System

This system allows you to record and replay Franka FR3 robot arm trajectories while using the original gello control interface.

## Features

- **Record Real Robot Data**: Records actual Franka robot arm joint states from `/franka/joint_states` topic
- **Replay Trajectories**: Replays recorded trajectories by publishing to `/gello/joint_states` topic
- **GUI Control Panel**: Easy-to-use interface with one-click recording and replay buttons
- **Non-Intrusive**: Original `run_fr3_real_ros2.sh` functionality remains completely unchanged

## Directory Structure

```
franka_recorder/
├── recorder_node.py       # ROS2 node for recording robot data
├── replay_node.py         # ROS2 node for replaying trajectories
├── control_gui.py         # GUI control panel
├── launch_recorder.sh     # Launch script for recorder/replay only
├── environment.yaml       # Conda environment configuration
└── README.md             # This file
```

## Installation

### Option 1: Using Conda (Recommended)

```bash
cd ~/gello_software/franka_recorder

# Create conda environment
conda env create -f environment.yaml

# Activate environment
conda activate franka_recorder
```

### Option 2: Using System Python

Make sure you have ROS2 Humble installed and Python 3.10+:

```bash
# Install required Python packages
pip3 install rclpy
```

## Usage

### Method 1: Integrated Launch (Recommended)

Use the enhanced launch script that includes all original functionality plus recorder/replay:

```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

This will:
1. Start all original GELLO and FR3 nodes (unchanged functionality)
2. Start recorder and replay nodes
3. Open the GUI control panel

### Method 2: Separate Launch

If you want to run the recorder/replay separately while `run_fr3_real_ros2.sh` is already running:

Terminal 1 (Original system):
```bash
cd ~/gello_software
./run_fr3_real_ros2.sh
```

Terminal 2 (Recorder/Replay):
```bash
cd ~/gello_software/franka_recorder
./launch_recorder.sh
```

## GUI Controls

### Recording
1. Click **"🔴 Start Recording"** button to begin recording real robot data
2. Operate the robot using GELLO as normal
3. Click **"⏹️ Stop Recording"** when done
4. Recording is automatically saved to `~/gello_software/franka_recordings/`

### Replay
1. Click **"🔄"** button to refresh the list of recordings
2. Select a recording from the dropdown menu
3. Click **"▶️ Start Replay"** to replay the trajectory
4. The robot will execute the recorded trajectory
5. Click **"⏹️ Stop Replay"** to stop early if needed

## Data Format

Recordings are saved as pickle files with the following structure:

```python
{
    'metadata': {
        'timestamp': str,           # Recording timestamp
        'sample_count': int,        # Number of samples
        'duration': float          # Duration in seconds
    },
    'data': [
        {
            'type': 'joint_state',  # or 'gripper_state'
            'timestamp': rclpy.time.Time,
            'elapsed': float,       # Elapsed time in seconds
            'data': {
                'header': Header,
                'name': list[str],  # Joint names
                'position': list[float],
                'velocity': list[float],
                'effort': list[float]
            }
        },
        ...
    ]
}
```

## ROS2 Services

The system provides the following ROS2 services:

### Recorder Services
- `/franka_recorder/start_stop` (SetBool): Start/stop recording

### Replay Services
- `/franka_replay/start_stop` (SetBool): Start/stop replay
- `/franka_replay/list_recordings` (Trigger): List available recordings
- `/franka_replay/load_recording` (String): Load a specific recording file

## Command Line Usage (Alternative to GUI)

You can also control recording and replay via command line:

```bash
# Start recording
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: true}"

# Stop recording
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: false}"

# List recordings
ros2 service call /franka_replay/list_recordings std_srvs/srv/Trigger

# Load a recording
ros2 service call /franka_replay/load_recording std_msgs/msg/String "{data: 'franka_recording_20231215_143022.pkl'}"

# Start replay
ros2 service call /franka_replay/start_stop std_srvs/srv/SetBool "{data: true}"

# Stop replay
ros2 service call /franka_replay/start_stop std_srvs/srv/SetBool "{data: false}"
```

## Troubleshooting

### GUI doesn't start
- Make sure you have tkinter installed: `sudo apt-get install python3-tk`
- Check that ROS2 nodes are running: `ros2 node list`

### Recording is empty
- Verify that `/franka/joint_states` topic is publishing: `ros2 topic echo /franka/joint_states`
- Make sure the FR3 controller is running

### Replay doesn't work
- Ensure a recording is loaded (check GUI selection)
- Verify that `/gello/joint_states` topic is being subscribed by the controller
- Check that the robot is in the correct mode

### Permission denied errors
- Make scripts executable: `chmod +x *.sh *.py`

## Safety Notes

⚠️ **IMPORTANT SAFETY WARNINGS**:

1. **Always monitor the robot** during replay to ensure safe operation
2. **Keep the emergency stop accessible** at all times
3. **Start with short recordings** to verify trajectory safety
4. **Check robot workspace** before replay to avoid collisions
5. **Recordings capture the exact motion** - make sure the environment is similar during replay

## Files and Directories

- **Recordings**: Saved to `~/gello_software/franka_recordings/`
- **Logs**: Check ROS2 logs with `ros2 node list` and `ros2 topic list`

## Integration with Original System

This system is designed to be completely non-intrusive:

- **Original script unchanged**: `run_fr3_real_ros2.sh` works exactly as before
- **Can run independently**: Recorder/replay can be launched separately
- **No modifications to robot control**: Uses standard ROS2 topics
- **Optional usage**: System works with or without recorder/replay active

## Advanced Usage

### Custom Recording Directory

Set the environment variable before launching:

```bash
export FRANKA_RECORDINGS_DIR="/path/to/custom/directory"
```

### Integration with Other Tools

Since the system uses standard ROS2 topics and services, you can integrate it with:
- ROS2 bag recordings
- Custom analysis scripts
- Motion planning libraries
- Visualization tools (RViz)

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS2 logs: `ros2 topic list`, `ros2 node list`
3. Verify all nodes are running: `ps aux | grep python3`

## License

Same as the parent gello_software project.
