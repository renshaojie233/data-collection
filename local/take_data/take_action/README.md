# Robot Data Collection System

这个项目用于收集和保存Franka FR3机械臂通过GELLO设备控制时的运动数据。

## 系统架构

```
本地机器 (ubuntu)                    远程机器 (rsj@172.16.1.2)
┌─────────────────────┐             ┌──────────────────────────────┐
│                     │             │                              │
│  start_collection.sh│────SSH─────>│  run_fr3_real_ros2.sh        │
│                     │             │  (原有ROS2控制系统)           │
│                     │             │                              │
│                     │             │  remote_data_bridge.py       │
│  local_data_receiver│<───TCP──────│  (ROS2话题订阅 + TCP发送)     │
│                     │   Port 9999 │                              │
│  ↓                  │             └──────────────────────────────┘
│  data/              │
│  ├─ session_*.json  │
│  └─ session_*.h5    │
└─────────────────────┘
```

## 功能特性

- ✅ 远程启动ROS2控制系统（不修改原有脚本）
- ✅ 实时传输机械臂数据到本地
- ✅ 同时保存JSON和HDF5格式
- ✅ 实时显示数据流状态
- ✅ 自动清理和优雅退出

## 安装依赖

### 1. 创建Conda环境

```bash
cd /home/ubuntu/take_data/take_action
conda env create -f environment.yml
conda activate take_data
```

### 2. 验证安装

```bash
python3 -c "import numpy, pandas, h5py; print('✓ All dependencies installed')"
```

## 使用方法

### 快速开始

```bash
cd /home/ubuntu/take_data/take_action/scripts
conda activate take_data
./start_collection.sh
```

### 脚本会自动完成以下步骤：

1. 连接到远程机器 (172.16.1.2)
2. 上传数据桥接脚本
3. 启动 `run_fr3_real_ros2.sh`
4. 启动数据桥接器
5. 开始收集数据

### 停止收集

按 `Ctrl+C` 停止收集，数据会自动保存到 `data/` 目录。

## 数据格式

### JSON格式 (`session_YYYYMMDD_HHMMSS.json`)

```json
{
  "session_name": "20241220_123456",
  "start_time": 1703088896.123,
  "packet_count": 1000,
  "data": [
    {
      "timestamp": 1703088896.123,
      "datetime": "2024-12-20T12:34:56",
      "gello_joints": {
        "position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        "velocity": [...],
        "effort": [...]
      },
      "franka_joints": {
        "position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        "velocity": [...],
        "effort": [...]
      },
      "end_effector_pose": {
        "position": {"x": 0.5, "y": 0.0, "z": 0.3},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
      }
    }
  ]
}
```

### HDF5格式 (`session_YYYYMMDD_HHMMSS.h5`)

高效的数值数组存储格式，适合大规模数据分析：

```
/timestamps                  [N]      时间戳
/gello/positions            [N, 7]   GELLO关节位置
/franka/positions           [N, 7]   机械臂关节位置
/franka/velocities          [N, 7]   机械臂关节速度
/franka/efforts             [N, 7]   机械臂关节力矩
/end_effector/positions     [N, 3]   末端执行器位置 (x,y,z)
/end_effector/orientations  [N, 4]   末端执行器方向 (四元数)
```

## 读取数据示例

### Python读取HDF5

```python
import h5py
import numpy as np

# 打开文件
with h5py.File('data/session_20241220_123456.h5', 'r') as f:
    timestamps = f['timestamps'][:]
    franka_pos = f['franka/positions'][:]
    ee_pos = f['end_effector/positions'][:]

    print(f"Collected {len(timestamps)} samples")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
    print(f"Average rate: {len(timestamps)/(timestamps[-1]-timestamps[0]):.1f} Hz")
```

### Python读取JSON

```python
import json

with open('data/session_20241220_123456.json', 'r') as f:
    data = json.load(f)

print(f"Total packets: {data['packet_count']}")
for item in data['data'][:5]:  # 打印前5个数据点
    print(f"Time: {item['datetime']}")
    print(f"Franka joints: {item['franka_joints']['position']}")
```

## 目录结构

```
/home/ubuntu/take_data/take_action/
├── README.md                   # 本文件
├── environment.yml             # Conda环境配置
├── scripts/
│   ├── start_collection.sh     # 主启动脚本
│   ├── remote_data_bridge.py   # 远程数据桥接（运行在远程机器）
│   └── local_data_receiver.py  # 本地数据接收器
├── data/                       # 收集的数据文件
│   ├── session_*.json
│   └── session_*.h5
├── config/                     # 配置文件
└── logs/                       # 日志文件
```

## 配置说明

默认配置在 `scripts/start_collection.sh` 中：

- **远程主机**: 172.16.1.2
- **远程用户**: rsj
- **TCP端口**: 9999
- **ROS2脚本**: /home/rsj/gello_software/run_fr3_real_ros2.sh

如需修改，编辑脚本顶部的配置变量。

## 故障排除

### 无法连接到远程主机

```bash
ping 172.16.1.2
ssh rsj@172.16.1.2
```

### 检查远程ROS2系统状态

```bash
ssh rsj@172.16.1.2 'cat /tmp/run_fr3_real_ros2.log'
```

### 检查数据桥接器状态

```bash
ssh rsj@172.16.1.2 'cat /tmp/bridge.log'
```

### 手动清理远程进程

```bash
ssh rsj@172.16.1.2 'pkill -f remote_data_bridge'
ssh rsj@172.16.1.2 'pkill -f run_fr3_real_ros2'
```

## 技术细节

### 数据流

1. **GELLO设备** → `/gello/joint_states` (ROS2)
2. **FR3控制器** 读取GELLO状态并控制机械臂
3. **机械臂** → `/franka/joint_states` (ROS2)
4. **remote_data_bridge.py** 订阅ROS2话题 → TCP发送
5. **local_data_receiver.py** 接收TCP数据 → 保存文件

### 采样频率

- GELLO发布频率：~100 Hz
- Franka状态频率：~100 Hz
- TCP传输频率：取决于网络延迟
- 实际保存频率：通常在50-100 Hz

## 许可证

内部使用项目
