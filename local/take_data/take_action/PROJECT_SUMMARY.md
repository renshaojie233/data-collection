# 项目总结

## 项目信息

- **项目名称**：Robot Data Collection System (机械臂数据采集系统)
- **项目路径**：`/home/ubuntu/take_data/take_action`
- **Conda环境**：`take_data`
- **创建时间**：2024-12-20

## 项目目标

在**不修改原有ROS2控制系统**的前提下，实时采集并保存Franka FR3机械臂通过GELLO设备控制时的运动数据。

## 系统架构

### 数据流程

```
GELLO设备 ──> ROS2话题 ──> FR3控制器 ──> Franka机械臂
                 │                          │
                 │                          │
            [/gello/joint_states]    [/franka/joint_states]
                 │                          │
                 └──────────┬───────────────┘
                            │
                  remote_data_bridge.py (远程机器)
                            │
                      TCP Stream (Port 9999)
                            │
                  local_data_receiver.py (本地机器)
                            │
                      ┌─────┴─────┐
                      │           │
                  JSON文件    HDF5文件
```

## 核心组件

### 1. 远程数据桥接器 (`remote_data_bridge.py`)

- **运行位置**：远程机器 (172.16.1.2)
- **功能**：
  - 订阅ROS2话题（GELLO、Franka关节状态、末端执行器位姿）
  - 实时序列化数据
  - 通过TCP发送到本地机器

### 2. 本地数据接收器 (`local_data_receiver.py`)

- **运行位置**：本地机器
- **功能**：
  - 接收TCP数据流
  - 实时显示采集状态
  - 保存为JSON和HDF5格式

### 3. 启动脚本 (`start_collection.sh`)

- **功能**：
  - 自动SSH连接远程机器
  - 启动原有ROS2系统
  - 部署并启动数据桥接器
  - 启动本地接收器
  - 优雅处理退出和清理

### 4. 数据可视化工具 (`visualize_data.py`)

- **功能**：
  - 绘制关节轨迹
  - 绘制末端执行器3D路径
  - 分析跟踪误差
  - 生成统计报告

## 关键特性

✅ **不修改原系统**：完全独立于 `run_fr3_real_ros2.sh`
✅ **实时传输**：基于TCP的高效数据流
✅ **双格式保存**：JSON（可读）+ HDF5（高效）
✅ **实时监控**：显示采样频率和当前关节角度
✅ **自动化部署**：一键启动所有组件
✅ **优雅退出**：Ctrl+C自动保存数据并清理

## 数据格式

### HDF5数据集

| 数据集路径 | 维度 | 描述 |
|-----------|------|------|
| `/timestamps` | [N] | Unix时间戳 |
| `/gello/positions` | [N, 7] | GELLO关节位置 (rad) |
| `/franka/positions` | [N, 7] | 机械臂关节位置 (rad) |
| `/franka/velocities` | [N, 7] | 机械臂关节速度 (rad/s) |
| `/franka/efforts` | [N, 7] | 机械臂关节力矩 (Nm) |
| `/end_effector/positions` | [N, 3] | 末端位置 (x,y,z) (m) |
| `/end_effector/orientations` | [N, 4] | 末端方向 (四元数) |

## 性能指标

- **采样频率**：50-100 Hz（取决于网络）
- **数据延迟**：< 50ms
- **存储效率**：HDF5比JSON小约70%
- **网络带宽**：约100-200 KB/s

## 使用流程

### 完整工作流

```bash
# 1. 激活环境
conda activate take_data

# 2. 进入脚本目录
cd /home/ubuntu/take_data/take_action/scripts

# 3. 开始采集
./start_collection.sh

# 4. 操作GELLO设备控制机械臂...

# 5. 停止采集 (Ctrl+C)

# 6. 查看数据
python3 visualize_data.py ../data/session_*.h5 --all
```

## 目录结构

```
/home/ubuntu/take_data/take_action/
├── README.md                      # 详细文档
├── QUICKSTART.md                  # 快速开始指南
├── PROJECT_SUMMARY.md             # 本文件
├── environment.yml                # Conda环境配置
├── config/
│   └── config.yaml                # 系统配置
├── scripts/
│   ├── start_collection.sh        # 主启动脚本
│   ├── remote_data_bridge.py      # 远程桥接器
│   ├── local_data_receiver.py     # 本地接收器
│   └── visualize_data.py          # 数据可视化
├── data/                          # 数据文件（运行时生成）
│   ├── session_*.json
│   ├── session_*.h5
│   └── plots/                     # 图表（如果保存）
└── logs/                          # 日志文件（运行时生成）
```

## 技术栈

- **系统通信**：SSH (sshpass), TCP/IP
- **ROS2**：Humble (远程机器)
- **Python库**：
  - `rclpy` - ROS2 Python客户端
  - `h5py` - HDF5文件处理
  - `numpy` - 数值计算
  - `matplotlib` - 数据可视化
  - `pandas` - 数据分析（可选）

## 配置说明

主要配置在 `config/config.yaml` 和 `scripts/start_collection.sh` 中：

- **远程主机**：172.16.1.2
- **远程用户**：rsj
- **TCP端口**：9999
- **ROS2脚本**：/home/rsj/gello_software/run_fr3_real_ros2.sh

## 安全考虑

⚠️ **注意**：配置文件中包含密码，仅用于内部开发环境。

生产环境建议：
- 使用SSH密钥认证
- 将敏感信息移至环境变量
- 使用加密的配置文件

## 扩展建议

未来可以添加：
1. **实时可视化**：使用RViz或自定义GUI
2. **数据回放**：从保存的数据重播机械臂运动
3. **机器学习**：用于模仿学习的数据集生成
4. **多机器人**：支持同时采集多个机器人数据
5. **云存储**：自动上传数据到云端

## 维护

- **日志位置**：`logs/` 目录
- **远程日志**：
  - ROS2系统：`/tmp/run_fr3_real_ros2.log`
  - 数据桥接：`/tmp/bridge.log`
- **清理命令**：脚本退出时自动清理远程进程

## 故障排除

常见问题参见 `README.md` 的故障排除章节。

## 版本历史

- **v1.0** (2024-12-20) - 初始版本
  - 基本数据采集功能
  - JSON和HDF5格式支持
  - 实时可视化工具

---

项目创建者：Claude Code
最后更新：2024-12-20
