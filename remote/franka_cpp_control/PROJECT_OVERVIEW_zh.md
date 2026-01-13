# franka_cpp_control 项目说明

本项目提供基于 libfranka 0.18.0 的最小化 C++ 控制程序，用于控制 Franka FR3 机械臂。
包含如下功能：

- 常见关节姿态（ready/home 等）
- 水平面圆轨迹
- 录制轨迹回放（位置/速度/阻抗多版本）
- 针对录制数据的“平滑阻抗回放”稳定版本

所有程序通过 CMake 构建，libfranka 依赖使用本地副本。

## 目录结构

- `src/`
  - `main.cpp`：移动到常见关节姿态（ready/home/straight/folded）
  - `circle_xy.cpp`：固定中心的水平圆轨迹
  - `track_json_raw.cpp`：直接关节位置回放（容易触发 reflex）
  - `track_json_vel.cpp`：速度回放（原始时间戳插值）
  - `track_json_vel_smooth.cpp`：速度回放 + 反馈与平滑
  - `track_json_impedance.cpp`：关节阻抗回放（稳定）
  - `track_json_impedance_smooth.cpp`：关节阻抗回放 + 目标平滑
- `third_party/libfranka-0.18.0/`：本地 libfranka
- `build/`：编译输出
- `run_track_json_impedance.sh`：一键启动脚本（稳定回放）
- `run_track_json_impedance.desktop`：GNOME 启动图标
- `replay_data/`：录制数据
  - `action/`：JSON/H5 轨迹
  - `video/`：视频

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

## 程序说明

### 常见姿态

```bash
./build/franka_ready_pose <robot_ip> [speed_factor] [pose|demo]
```

例：

```bash
./build/franka_ready_pose 172.16.0.2 0.2 demo
```

支持：`ready` `home` `straight` `folded` `demo`

### 水平圆轨迹

```bash
./build/franka_circle_xy <robot_ip> [radius] [angular_speed]
```

### 轨迹回放（原始位置版）

```bash
./build/franka_track_json_raw <robot_ip> <json_path> [resample_hz] [start_speed]
```

### 轨迹回放（速度版）

```bash
./build/franka_track_json_vel <robot_ip> <json_path> [time_scale] [start_speed] [max_vel] [max_acc] [vel_alpha]
```

### 轨迹回放（速度+平滑）

```bash
./build/franka_track_json_vel_smooth <robot_ip> <json_path> [time_scale] [start_speed] [max_vel] [max_acc] [vel_alpha] [kp]
```

### 轨迹回放（阻抗版）

```bash
./build/franka_track_json_impedance <robot_ip> <json_path> [time_scale] [start_speed] [k] [d] [k_alpha]
```

推荐稳定参数：

```bash
./build/franka_track_json_impedance 172.16.0.2 \
  /home/rsj/franka_cpp_control/replay_data/action/action_data_20251224_045112.json \
  1.5 0.2 30 5 0.3
```

### 轨迹回放（阻抗+目标平滑）

```bash
./build/franka_track_json_impedance_smooth <robot_ip> <json_path> [time_scale] [start_speed] [k] [d] [k_alpha] [q_alpha]
```

推荐稳定参数：

```bash
./build/franka_track_json_impedance_smooth 172.16.0.2 \
  /home/rsj/franka_cpp_control/replay_data/action/action_data_20251224_045112.json \
  1.5 0.2 30 5 0.3 0.2
```

## 一键启动

### Franka 原装夹爪

```bash
/home/rsj/franka_cpp_control/run_track_json_impedance_gripper_oneclick.sh
```

默认使用 `franka_track_json_impedance_smooth`（阻抗+目标平滑）并控制 Franka 原装夹爪。

可用环境变量覆盖默认值：

- `TIME_SCALE`
- `START_SPEED`
- `K_GAIN`
- `D_GAIN`
- `K_ALPHA`
- `Q_ALPHA`

### Robotiq 2F-85

```bash
/home/rsj/franka_cpp_control/run_track_json_impedance_gripper_robotiq_oneclick.sh
```

特点：

- 自动选择最新且包含 `gripper_joints` 的 JSON 轨迹。
- 位置优先（减少目标平滑），默认使用关节刚度：`240,240,240,240,100,60,20`。
- 默认**关闭重力补偿**（`USE_GRAVITY_COMP=0`）。

常用环境变量（可按需覆盖）：

- `ROBOTIQ_PORT`（推荐使用 `/dev/serial/by-id/...`）
- `ROBOTIQ_LAYOUT`（默认 `codex`，对应寄存器字节顺序）
- `ROBOTIQ_START_MODE`（`hold`/`follow`，默认 `hold`，避免启动瞬间开合）
- `ROBOTIQ_ACTIVATE`（`auto`/`skip`/`force`，默认 `auto`）
- `ROBOTIQ_SPEED` / `ROBOTIQ_FORCE`
- `K_GAINS` / `D_GAINS`（7 个数，用逗号分隔）
- `USE_GRAVITY_COMP`（0/1）
- `LOAD_MASS` / `LOAD_COM` / `LOAD_INERTIA` / `LOAD_SCALE`

示例（开启重力补偿）：

```bash
USE_GRAVITY_COMP=1 /home/rsj/franka_cpp_control/run_track_json_impedance_gripper_robotiq.sh
```

## Robotiq 故障排查（夹爪不动/重启后）

- 确认端口：`/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA61OLIF-if00-port0` -> `/dev/ttyUSB1`
- 若已安装 udev 规则，可直接使用固定端口 `/dev/robotiq`
- 若提示 `Permission denied` 或无法打开端口，优先检查 ModemManager：
  - `sudo systemctl stop ModemManager`（可选再 `disable`）
  - 或添加 udev 规则忽略该设备并设置权限，重载规则后重新插拔

## Robotiq 端口与寄存器约定（重要）

- 端口优先级：`/dev/robotiq` > `/dev/serial/by-id/...` > `/dev/ttyUSB*`
- 推荐使用固定名称 `/dev/robotiq`，避免重启后端口号变化导致找不到设备。
- Robotiq 2F-85 常见“能连接但不动”的原因是寄存器起始地址或寄存器字节顺序不匹配。

本项目默认采用与你的 `robotiq 2f-85/codex` 一致的配置：

- 输出寄存器起始：`0x03E8`
- 输入寄存器起始：`0x07D0`
- 寄存器字节顺序（写入 6 字节）：`[b0, 0, 0, rPR, rSP, rFR]`

如需兼容旧实现（例如 gello 默认布局），可通过环境变量切换：

- `ROBOTIQ_LAYOUT=legacy`（使用 `[b0, rPR, rSP, rFR, 0, 0]`）
- 或显式指定 `ROBOTIQ_OUT_START=0x03E9`

推荐 udev 规则（保存为 `/etc/udev/rules.d/99-robotiq.rules`）：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DA61OLIF", SYMLINK+="robotiq", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
```

重载规则并重新插拔：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

确认用户在 `dialout` 组（`groups`），必要时注销再登录。

## 图标位置

应用菜单入口在：

- `~/.local/share/applications/franka-trajectory-franka-gripper.desktop`
- `~/.local/share/applications/franka-trajectory-robotiq-2f85.desktop`

## 安全提示

- 确保机械臂处于 FCI 模式且无错误。
- 运行前清空工作空间，保持安全距离。
- 若触发 reflex，降低增益或增加滤波。
