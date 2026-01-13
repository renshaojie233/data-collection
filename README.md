# GELLO Data Collection (Local + Remote)

This repository consolidates the local data collection GUI, remote GELLO control stack, and replay tools used in the workflow.

## Repo Layout

```
local/
  take_data/                 # Local GUI + video/action recorder + utilities
remote/
  gello_software/            # Remote ROS2/GELLO control stack
  franka_cpp_control/        # Remote replay (track JSON, impedance/velocity)
```

## Quick Start (Local GUI)

1) Open the GUI:
```
python3 local/take_data/take_video_action/video_action_recorder.py
```

2) Ensure the remote host details are correct in:
`local/take_data/take_video_action/video_action_recorder.py`
- `REMOTE_HOST`
- `REMOTE_USER`
- `REMOTE_PORT`
Credentials are **not stored** in the repo. Set the password via env if needed:
```
export REMOTE_PASSWORD=your_password_here
```
If `REMOTE_PASSWORD` is empty, the GUI uses normal SSH (keys/agent).

3) The GUI defaults to **relative control** (can switch in the right panel).

4) Click **开始录制** to start recording. For relative mode, the system waits for the robot to return to `robot_zero` before recording.

5) Click **停止录制** to stop. In relative mode, the robot will return to `robot_zero` smoothly.

## Deployment (Local Machine)

1) Install system dependencies (Ubuntu 20.04/22.04):
```
sudo apt update
sudo apt install -y python3 python3-pip python3-tk python3-venv
```

2) Install RealSense and OpenCV:
```
sudo apt install -y librealsense2-utils librealsense2-dev
pip3 install opencv-python pyrealsense2 numpy h5py pillow
```

3) Run the GUI:
```
python3 local/take_data/take_video_action/video_action_recorder.py
```

4) If you need to update local code from `/home/ubuntu/take_data`:
```
scripts/sync_local_from_ubuntu.sh /home/ubuntu/take_data
```

## Deployment (Remote Robot)

Remote code is stored under `remote/gello_software` and `remote/franka_cpp_control`.

1) Ensure ROS2 Humble is installed on the remote machine.

2) Copy the remote folders to the robot:
```
rsync -av remote/gello_software rsj@172.16.1.2:/home/rsj/
rsync -av remote/franka_cpp_control rsj@172.16.1.2:/home/rsj/
```

3) Relative control entrypoint:
```
/home/rsj/gello_software/start_relative_gello_control.sh
```

4) Absolute control entrypoint:
```
/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh
```

## Sync Scripts

Two helper scripts keep the repo in sync:

- Local (Ubuntu machine):
```
scripts/sync_local_from_ubuntu.sh /home/ubuntu/take_data
```

- Remote robot:
```
export REMOTE_HOST=172.16.1.2
export REMOTE_USER=rsj
export REMOTE_BASE=/home/rsj
export SSH_PASS=your_password_here   # optional (uses sshpass)
scripts/sync_remote_from_robot.sh
```

If you do not set `SSH_PASS`, the script uses normal SSH and expects keys/agent.

## Credentials Policy

- No passwords or tokens are committed to this repo.
- GUI uses `REMOTE_PASSWORD` environment variable for SSH (optional).
- Sync script uses `SSH_PASS` environment variable (optional).

## System Configuration (Current Machines)

Local data collection machine:
- OS: Ubuntu 22.04.3 LTS (kernel 6.2.0-26-generic)
- CPU: Intel(R) Core(TM) i9-14900KF (32 threads)
- RAM: 188 GiB
- Disk: 1.8T (root filesystem)

Remote robot machine:
- OS: Ubuntu 22.04.5 LTS (kernel 6.12.58-rt14)
- CPU: Intel(R) Core(TM) Ultra 9 285H
- RAM: 30 GiB
- Disk: 492G (root filesystem)

## Remote Setup (GELLO Control)

Remote control code is under `remote/gello_software/`.

Key entry points:
- `remote/gello_software/start_relative_gello_control.sh`
- `remote/gello_software/run_fr3_real_ros2_robotiq.sh`

Relative control config:
`remote/gello_software/configs/relative_gello_control.yaml`

Important fields:
- `robot_zero`: fixed initial robot joint pose
- `publish_rate_hz`: control publish rate
- `capture_robot_zero`: `false` uses fixed `robot_zero` (recommended)
- `return_speed_rad_s`: speed limit when returning to zero

Relative publisher:
`remote/gello_software/scripts/gello_relative_publisher.py`

## Replay (Remote)

Replay tools live in `remote/franka_cpp_control/`.

Typical entry:
- `remote/franka_cpp_control/run_track_json_impedance_gripper_robotiq.sh`

The local GUI copies action/video data to the remote replay directory and triggers the replay script.

## Configurable Sampling Rates

- Video/action record FPS (local):
  `local/take_data/take_video_action/video_action_recorder.py` → `self.record_fps`

- GELLO relative publish rate (remote):
  `remote/gello_software/configs/relative_gello_control.yaml` → `publish_rate_hz`

## Data Outputs

Recorded sessions are stored under:
`/home/ubuntu/take_data/data/record_XXX/`

Each session includes:
- `video/` (MP4s + raw frames)
- `action/` (JSON + HDF5)

## Troubleshooting

- **No robot data in GUI**
  - Ensure only one control stack is running on remote.
  - Use the built-in cleanup in the GUI or run:
    `remote/gello_software/kill_ros2_stale.sh`

- **Relative start fails**
  - Check `relative_gello_control.yaml` for correct `robot_zero`.
  - Increase `return_speed_rad_s` if return feels too slow.

- **Robot reflex / red light after stop**
  - Lower `return_speed_rad_s` to smooth the return.
  - Verify `robot_zero` is reachable and safe.

## Notes

- This repo intentionally excludes large data outputs and ROS build artifacts.
- Any secrets (SSH passwords, tokens) should be edited locally and **not** committed.
