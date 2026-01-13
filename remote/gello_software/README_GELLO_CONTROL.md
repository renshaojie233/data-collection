# GELLO FR3 控制脚本说明

本文件详细介绍两个一键启动脚本的用途、流程、配置和常见问题：

- `start_relative_gello_control.sh`：相对控制（以确认时刻为零点）
- `run_fr3_real_ros2_robotiq.sh`：绝对控制（GELLO 关节角直接映射）

## 1. 两种控制方式的区别

### 1.1 绝对控制（`run_fr3_real_ros2_robotiq.sh`）
- **核心逻辑**：GELLO 当前关节角直接作为机器人目标关节角（绝对值）。
- **特点**：
  - 启动后不需要确认步骤。
  - 只要 GELLO 姿态改变，机器人就会跟随变化。
  - 适合一致标定好的绝对映射场景。

### 1.2 相对控制（`start_relative_gello_control.sh`）
- **核心逻辑**：
  - 先让机器人对齐到一个固定姿态（配置里的 `robot_zero`）。
  - 用户按下确认（或自动使能）时，记录：
    - `robot_zero` = **确认瞬间机器人当前关节角**
    - `gello_zero` = **确认瞬间 GELLO 当前关节角**
  - 之后控制为：
    - `robot_target = robot_zero + (gello_now - gello_zero)`
- **特点**：
  - 只看 **GELLO 相对变化**，而不是绝对姿态。
  - 适合每次操作时重新“设零点”的场景。

## 2. 快速启动

### 2.1 绝对控制（直接启动）
```bash
./run_fr3_real_ros2_robotiq.sh
```

### 2.2 相对控制（手动确认）
```bash
./start_relative_gello_control.sh
# 机械臂对齐到固定姿态后，输入 y + 回车确认
```

### 2.3 相对控制（自动确认）
```bash
RELATIVE_AUTO_ENABLE=1 ./start_relative_gello_control.sh
```

## 3. 相对控制脚本详细流程

`start_relative_gello_control.sh` 执行流程如下：

1. **清理环境变量**
   - 过滤 conda PATH / LD_LIBRARY_PATH / PYTHONPATH，避免干扰 ROS2。

2. **自动检测串口并更新配置**
   - 自动查找 GELLO USB 设备 `/dev/gello` 或 `/dev/serial/by-id/*`。
   - 自动查找 Robotiq 夹爪串口 `/dev/robotiq` 等。
   - 自动更新以下配置文件中的 `com_port` 字段：
     - `ros2/src/franka_gello_state_publisher/config/fr3_rsjt.yaml`
     - `ros2/install/franka_gello_state_publisher/.../fr3_rsjt.yaml`
     - `ros2/src/franka_gripper_manager/config/example_fr3_config_robotiq.yaml`
     - `ros2/install/franka_gripper_manager/.../example_fr3_config_robotiq.yaml`

3. **加载 ROS2 环境**
   - `/opt/ros/$ROS_DISTRO/setup.bash`
   - `$FRANKA_WS/install/setup.bash`
   - `$GELLO_WS/install/setup.bash`

4. **启动相对控制节点**
   - Python 脚本：`scripts/gello_relative_publisher.py`
   - 发布话题：`gello/joint_states`
   - 如果还未确认，会先持续发布 `robot_zero`（固定姿态）。

5. **等待 /gello/joint_states 出现**
   - 未检测到数据会退出，提示检查 GELLO USB。

6. **启动 FR3 控制器**
   - `ros2 launch franka_fr3_arm_controllers ...`
   - 可选设置实时优先级、CPU 绑定。

7. **确认相对零点（关键步骤）**
   - 手动输入 `y` 或自动使能时会：
     - 读取 `franka/joint_states` → 设为 `robot_zero`
     - 读取 GELLO 关节 → 设为 `gello_zero`
   - 后续控制为相对增量：
     - `robot_target = robot_zero + (gello_now - gello_zero)`

8. **启动 Robotiq 夹爪**
   - `ros2 launch franka_gripper_manager ...`

9. **退出清理**
   - 捕捉信号后会停止所有子进程。

## 4. 绝对控制脚本详细流程

`run_fr3_real_ros2_robotiq.sh` 流程与相对控制类似，但差异在核心控制逻辑：

1. 环境清理、串口检测、配置更新、ROS2 环境加载。
2. 启动 GELLO 绝对发布节点：
   - `ros2 launch franka_gello_state_publisher main.launch.py`
   - `gello/joint_states` 发布 **绝对关节角**。
3. 启动 FR3 控制器，直接使用这些绝对关节角作为目标。
4. 启动 Robotiq 夹爪。

**注意**：该脚本不会等待或捕捉“相对零点”，因此只要 GELLO 姿态变化，机器人就会跟随。

## 5. 配置文件说明

### 5.1 相对控制配置
文件：`configs/relative_gello_control.yaml`
```yaml
gello_zero: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot_zero: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 1.57]
publish_rate_hz: 25.0
# robot_joint_topic: franka/joint_states   # 可选，默认此值
```
- `robot_zero`：**对齐用的固定姿态**（未确认前会持续发布）。
- `gello_zero`：仅作为默认值，确认时会被自动覆盖。
- `robot_joint_topic`：用于捕捉机器人当前姿态的关节话题。

### 5.2 GELLO 硬件配置
文件：`ros2/src/franka_gello_state_publisher/config/fr3_rsjt.yaml`
- `com_port` 会被脚本自动更新。
- `best_offsets / joint_signs` 决定关节映射方向和零位校准。

## 6. 常用环境变量

两脚本通用：
- `ROS_DISTRO`：默认 `humble`
- `FRANKA_WS`：默认 `$HOME/franka_ros2_ws`
- `GELLO_WS`：默认 `$HOME/gello_software/ros2`
- `LIBFRANKA_PREFIX`：默认 `~/.local/libfranka-0.18.0` 或 `/usr/local/libfranka-0.18.0`
- `GELLO_CFG`：默认 `fr3_rsjt.yaml`
- `FR3_CFG`：默认 `fr3_rsjt_robotiq.yaml`
- `GRIPPER_CFG`：默认 `example_fr3_config_robotiq.yaml`
- `ROBOTIQ_PORT`：可手动指定串口（否则自动检测）
- `ROBOTIQ_PREFIX`：默认 `$GELLO_WS/install`
- `ROS2_CONTROL_CPU`：CPU 绑定核心（自动选择）
- `ENABLE_RT`：是否设置实时优先级，默认 `1`
- `RT_PRIORITY`：实时优先级，默认 `80`
- `PIN_CPU`：是否绑定 CPU，默认 `1`
- `ROS2_DEBUG`：是否启用 debug 日志

相对控制专用：
- `RELATIVE_CFG`：相对配置文件路径
- `RELATIVE_NODE`：相对控制 Python 节点路径
- `RELATIVE_AUTO_ENABLE=1`：自动确认零点

## 7. 常见问题排查

### 7.1 找不到 /gello/joint_states
- 检查 GELLO USB 是否连接、供电是否正常。
- 查看 `/dev/serial/by-id/` 是否有 GELLO 设备。
- 确认 `com_port` 已被正确写入配置。

### 7.2 相对控制启用失败
- 确认 `/franka/joint_states` 已发布。
- 等待 FR3 控制器启动完成后再确认。
- 手动调用服务：
  ```bash
  ros2 service call /gello_relative/enable std_srvs/srv/SetBool '{data: true}'
  ```

### 7.3 机器人突然跳变或报 reflex
- 相对控制模式下，确认前会向 `robot_zero` 对齐。
- 建议将 `robot_zero` 设为安全姿态。
- 确认时请确保机器人已到达稳定姿态。

### 7.4 提示无法锁定内存
- 日志：`Unable to lock the memory` 通常是权限问题。
- 不影响功能，但可通过设置 `ulimit -l` 或 `CAP_IPC_LOCK` 优化实时性。

### 7.5 Robotiq 初始化异常
- 如出现 `Reset failed`，检查电源与串口连接。
- `Robotiq connected` 后一般可正常使用。

## 8. 推荐操作流程（相对控制）

1. 确认机械臂在安全区域。
2. 启动脚本：
   ```bash
   ./start_relative_gello_control.sh
   ```
3. 等机械臂对齐到固定姿态。
4. 确认 GELLO 操作手柄在舒适初始位置。
5. 输入 `y` 确认启用相对控制。
6. 开始操作。

如需自动确认：
```bash
RELATIVE_AUTO_ENABLE=1 ./start_relative_gello_control.sh
```

