# franka_cpp_control

Minimal libfranka 0.18.0 example to move a Franka arm into a common ready pose.

## Prerequisites

- libfranka 0.18.0 installed locally.
- Robot in FCI mode and no active errors.

If you built libfranka with `/home/rsj/gello_software/rebuild_libfranka_user.sh`,
the default install prefix is:

- `$HOME/.local/libfranka-0.18.0`

## Build

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/.local/libfranka-0.18.0
cmake --build build -j
```

If you installed libfranka somewhere else, point `CMAKE_PREFIX_PATH` to that
prefix or set `Franka_DIR` to `<prefix>/lib/cmake/Franka`.

## Run

```bash
./build/franka_ready_pose <robot_ip> [speed_factor]
```

Example:

```bash
./build/franka_ready_pose 172.16.0.2 0.2
```

The `speed_factor` range is (0, 1], where 0.2 is a slow, safe default.
