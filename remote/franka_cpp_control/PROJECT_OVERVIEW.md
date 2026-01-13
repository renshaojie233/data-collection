# franka_cpp_control Project Overview

This project provides minimal, self-contained C++ programs based on
libfranka 0.18.0 to control a Franka FR3 arm. It includes:

- Ready-pose motion
- Circular Cartesian motion
- Trajectory replay (position/velocity/impedance variants)
- A smoothed impedance replay tuned for stable playback of recorded data

All programs are built with CMake and keep dependencies local to the workspace.

## Directory Layout

- `src/`:
  - `main.cpp`: move to common joint poses (ready/home/etc.)
  - `circle_xy.cpp`: draw a horizontal circle around a fixed center
  - `track_json_raw.cpp`: naive joint-position replay (may trigger reflex)
  - `track_json_vel.cpp`: velocity replay using original timestamps
  - `track_json_vel_smooth.cpp`: velocity replay with extra smoothing
  - `track_json_impedance.cpp`: joint impedance replay (stable params)
  - `track_json_impedance_smooth.cpp`: impedance replay with goal filtering
- `third_party/libfranka-0.18.0/`: local libfranka install
- `build/`: CMake build output
- `run_track_json_impedance.sh`: one-click launcher for the smooth impedance replay
- `run_track_json_impedance.desktop`: launcher icon for GNOME app grid
- `replay_data/`: recorded data for replay
  - `action/`: JSON/H5 trajectories
  - `video/`: recorded videos (if any)

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

The CMake config auto-detects the local libfranka install in
`third_party/libfranka-0.18.0`.

## Programs

### Ready Pose

Moves the arm to a preset joint configuration.

```bash
./build/franka_ready_pose <robot_ip> [speed_factor] [pose|demo]
```

Example:

```bash
./build/franka_ready_pose 172.16.0.2 0.2 demo
```

Supported poses: `ready`, `home`, `straight`, `folded`, `demo`.

### Circle in XY Plane

Moves the end-effector in a horizontal circle around a fixed center with
constant orientation.

```bash
./build/franka_circle_xy <robot_ip> [radius] [angular_speed]
```

Example:

```bash
./build/franka_circle_xy 172.16.0.2 0.1 0.5
```

### Trajectory Replay (Raw)

Direct joint-position replay with simple interpolation. This can trigger
reflex stops if the recorded data is noisy.

```bash
./build/franka_track_json_raw <robot_ip> <json_path> [resample_hz] [start_speed]
```

### Trajectory Replay (Velocity)

Velocity replay using original timestamps. Smoother than raw position replay.

```bash
./build/franka_track_json_vel <robot_ip> <json_path> [time_scale] [start_speed] [max_vel] [max_acc] [vel_alpha]
```

### Trajectory Replay (Velocity + Smoothing)

Velocity replay with additional filtering and position feedback.

```bash
./build/franka_track_json_vel_smooth <robot_ip> <json_path> [time_scale] [start_speed] [max_vel] [max_acc] [vel_alpha] [kp]
```

### Trajectory Replay (Impedance)

Joint impedance replay using torque control. This is the most stable method
for recorded trajectories.

```bash
./build/franka_track_json_impedance <robot_ip> <json_path> [time_scale] [start_speed] [k] [d] [k_alpha]
```

Recommended stable parameters:

```bash
./build/franka_track_json_impedance 172.16.0.2 \
  /home/rsj/franka_cpp_control/replay_data/action/action_data_20251224_045112.json \
  1.5 0.2 30 5 0.3
```

### Trajectory Replay (Impedance + Goal Filtering)

Adds a low-pass filter on the goal trajectory to reduce micro-oscillations.

```bash
./build/franka_track_json_impedance_smooth <robot_ip> <json_path> [time_scale] [start_speed] [k] [d] [k_alpha] [q_alpha]
```

Recommended stable parameters:

```bash
./build/franka_track_json_impedance_smooth 172.16.0.2 \
  /home/rsj/franka_cpp_control/replay_data/action/action_data_20251224_045112.json \
  1.5 0.2 30 5 0.3 0.2
```

## One-Click Launcher

The script `run_track_json_impedance.sh` launches the smooth impedance replay
(`franka_track_json_impedance_smooth`) with stable defaults and the updated
replay path.

```bash
/home/rsj/franka_cpp_control/run_track_json_impedance.sh
```

Environment variables can override defaults:

- `TIME_SCALE`
- `START_SPEED`
- `K_GAIN`
- `D_GAIN`
- `K_ALPHA`
- `Q_ALPHA`

Example:

```bash
TIME_SCALE=1.2 K_GAIN=35 D_GAIN=6 /home/rsj/franka_cpp_control/run_track_json_impedance.sh
```

The GNOME app grid entry is installed at:

- `~/.local/share/applications/franka_track_json_impedance.desktop`

## Safety Notes

- Ensure the robot is in FCI mode and free of errors before running any program.
- Always keep a safe distance and a clear workspace during motion.
- If a reflex stop occurs, reduce gains, increase `time_scale`, or increase filtering.
