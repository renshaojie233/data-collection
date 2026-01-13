# OpenPI Impedance Position Stream Test Report

This report summarizes the interactive tests run for
`start_openpi_stream_impedance_pos.sh` and the 15 Hz replay sender on the
Franka robot in `/home/rsj/franka_cpp_control`.

## Scope
- Receiver: `/home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh`
- Sender: `/home/rsj/franka_cpp_control/stream_json_positions.py`
  (via `/home/rsj/franka_cpp_control/run_stream_impedance_pos_from_replay.sh`)
- Test trajectory: `/home/rsj/franka_cpp_control/replay_data/action/action_data_20260105_104718.json`

## Notable Files and Logs
- Receiver log: `/tmp/openpi_stream_impedance_pos.log`
- Action logs: `/home/rsj/franka_cpp_control/action_buffer/*.log`
- Sender script: `/home/rsj/franka_cpp_control/stream_json_positions.py`
- Receiver binary: `/home/rsj/franka_cpp_control/build/franka_openpi_stream_impedance_pos_robotiq`

## Key Config Knobs
- Receiver controls:
  - `RATE_HZ`, `K_GAIN`, `D_GAIN`, `K_ALPHA`, `Q_ALPHA`, `TIMEOUT_S`
  - `ACTION_INTERP` (added during this test for optional interpolation)
- Sender controls:
  - `RATE_HZ`, `BLEND_S`, `TIME_SCALE`, `ACTION_ORDER`

## Test Summary

### 1) Baseline Receiver Health Check
- Command: `start_openpi_stream_impedance_pos.sh run` (short timeout)
- Result: Receiver listened on `15123/15124` and reported waiting for stream.

### 2) Single-Sample Loopback Test
- Read from state port `15124`, re-send same joint positions to `15123`.
- Result: Receiver logged `Received action chunk size=1`, confirming round-trip.

### 3) Replay Test (30 Hz, BLEND_S=2)
- Sender: `stream_json_positions.py ... --blend 2`
- Result: Triggered reflex stop:
  `libfranka exception: ... motion aborted by reflex! ["controller_torque_discontinuity"]`
- Sender exited with `Broken pipe` when receiver stopped.

### 4) Replay Test (30 Hz, BLEND_S=5)
- Sender: `--blend 5`
- Result: Completed cleanly; no reflex stop.

### 5) Replay Test (15 Hz, BLEND_S=5)
- Sender: `RATE_HZ=15 BLEND_S=5`
- Result: Completed cleanly; receiver logged continuous action chunks.

### 6) Reduced Impedance (K/D) + Less Blend
- Receiver: `K_GAIN=20 D_GAIN=4`
- Sender: `RATE_HZ=15 BLEND_S=2`
- Result: Completed cleanly.

### 7) Receiver-Side Interpolation (Added Feature)
- Code change: optional linear interpolation between adjacent action samples.
- Enable with `ACTION_INTERP=1` on receiver.
- Test: `ACTION_INTERP=1 RATE_HZ=15 K_GAIN=20 D_GAIN=4`
- Result: Completed cleanly with smoother motion while sender stayed at 15 Hz.

### 8) "Closest to Original" Replay (No extra smoothing)
- Goal: minimize extra smoothing and follow raw 15 Hz samples.
- Receiver: `ACTION_INTERP=0 K_ALPHA=1 Q_ALPHA=1 RATE_HZ=15 K_GAIN=30 D_GAIN=5`
- Sender: `BLEND_S=0 RATE_HZ=15`
- Important ordering fix:
  1) Run `franka_move_to_json_start` first.
  2) Start receiver second.
  3) Start sender last.
- Result: Completed cleanly after ordering fix.

## Issues Observed
- Reflex stop on BLEND_S=2 at 30 Hz due to torque discontinuity.
- `Connection refused` when `franka_move_to_json_start` and receiver ran
  simultaneously. The move-to-start program takes control of the robot and
  causes the receiver to drop. Resolved by moving to start first.
- `libpinocchio.so.2.9.0` missing for `franka_move_to_json_start` when not
  using `LD_LIBRARY_PATH` from `third_party/libfranka-0.18.0/lib`.

## Code Change (Receiver Interpolation)
- File modified: `/home/rsj/franka_cpp_control/src/openpi_stream_impedance_pos_robotiq.cpp`
- Feature: `ACTION_INTERP=1` enables linear interpolation between the current
  and next action in the control loop.
- Build command:
  `cmake --build /home/rsj/franka_cpp_control/build --target franka_openpi_stream_impedance_pos_robotiq`

## Current Recommended "Closest to Original" Run (15 Hz)
1) Move to start:
   - `LD_LIBRARY_PATH=/home/rsj/franka_cpp_control/third_party/libfranka-0.18.0/lib \
      /home/rsj/franka_cpp_control/build/franka_move_to_json_start \
      172.16.0.2 /home/rsj/franka_cpp_control/replay_data/action/action_data_20260105_104718.json 0.1`
2) Start receiver (no interpolation, no low-pass):
   - `ACTION_INTERP=0 RATE_HZ=15 K_GAIN=30 D_GAIN=5 K_ALPHA=1 Q_ALPHA=1 \
      /home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh start`
3) Send trajectory:
   - `BLEND_S=0 RATE_HZ=15 \
      /home/rsj/franka_cpp_control/run_stream_impedance_pos_from_replay.sh \
      172.16.0.2 /home/rsj/franka_cpp_control/replay_data/action/action_data_20260105_104718.json`

