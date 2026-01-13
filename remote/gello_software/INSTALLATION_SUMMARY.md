# Franka录制重放系统 - 安装总结

## ✅ 已完成的工作

我已经为您创建了一个完整的Franka机械臂录制与重放系统。以下是详细说明：

## 📦 创建的文件列表

### 主目录文件
```
~/gello_software/
├── run_fr3_real_ros2.sh              # 原始脚本（未修改，保持原样）
├── run_fr3_with_recorder.sh          # 新增：集成录制功能的启动脚本
├── FRANKA_RECORDER_QUICKSTART.md     # 快速开始指南
└── INSTALLATION_SUMMARY.md           # 本文件
```

### 录制系统文件
```
~/gello_software/franka_recorder/
├── recorder_node.py                  # ROS2录制节点（订阅真实机械臂数据）
├── replay_node.py                    # ROS2重放节点（发布控制命令）
├── control_gui.py                    # GUI控制面板
├── launch_recorder.sh                # 独立启动脚本
├── test_installation.py              # 安装测试脚本
├── environment.yaml                  # Conda环境配置
├── README.md                         # 完整英文文档
└── USAGE_CN.md                       # 详细中文使用说明
```

### 自动创建的目录
```
~/gello_software/franka_recordings/   # 录制文件存储目录（首次运行时创建）
```

## 🎯 核心功能

### 1. 数据录制 (`recorder_node.py`)
- ✅ 订阅 `/franka/joint_states` - **真实机械臂数据**
- ✅ 订阅 `/franka_gripper/joint_states` - **真实夹爪数据**
- ✅ 只在点击录制按钮后才保存数据
- ✅ 自动保存为pickle格式，包含时间戳和元数据
- ✅ 提供ROS2服务接口控制录制开始/停止

### 2. 数据重放 (`replay_node.py`)
- ✅ 读取录制的pickle文件
- ✅ 发布到 `/gello/joint_states` topic控制机械臂
- ✅ 保持原始时间序列精确重放
- ✅ 自动加载最新录制
- ✅ 提供ROS2服务接口控制重放

### 3. GUI控制面板 (`control_gui.py`)
- ✅ 一键录制按钮（🔴 Start Recording / ⏹️ Stop Recording）
- ✅ 一键重放按钮（▶️ Start Replay / ⏹️ Stop Replay）
- ✅ 录制文件选择下拉菜单
- ✅ 刷新文件列表功能
- ✅ 打开录制文件夹功能
- ✅ 实时状态显示
- ✅ 友好的错误提示

## 🚀 使用方法

### 方法1：集成启动（推荐）
```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

这个脚本包含：
- ✅ 所有原有功能（GELLO + FR3控制）
- ✅ 录制节点
- ✅ 重放节点
- ✅ GUI控制面板

### 方法2：单独启动
如果 `run_fr3_real_ros2.sh` 已经在运行：
```bash
cd ~/gello_software/franka_recorder
./launch_recorder.sh
```

## 🔍 系统架构

```
录制流程：
GELLO控制器 --> 机械臂运动 --> /franka/joint_states --> 录制节点 --> .pkl文件

重放流程：
.pkl文件 --> 重放节点 --> /gello/joint_states --> 机械臂控制器 --> 机械臂运动

GUI：
control_gui.py <-- ROS2 Services --> recorder_node.py & replay_node.py
```

## 📋 安装检查

运行以下命令验证安装：

```bash
cd ~/gello_software/franka_recorder
./test_installation.py
```

应该看到所有测试通过：
```
Testing Python imports...
  ✓ rclpy imported successfully
  ✓ tkinter imported successfully
  ✓ pickle imported successfully
  ✓ sensor_msgs imported successfully
  ✓ std_srvs imported successfully

Testing file structure...
  ✓ recorder_node.py exists
  ✓ replay_node.py exists
  ✓ control_gui.py exists
  ...

🎉 All tests passed! Installation is complete.
```

## 🎓 快速教程

### 第一次使用

1. **启动系统**
   ```bash
   cd ~/gello_software
   ./run_fr3_with_recorder.sh
   ```
   等待GUI窗口弹出（大约5-10秒）

2. **测试录制**
   - 点击 `🔴 Start Recording`
   - 使用GELLO控制机械臂做一个简单的5秒动作
   - 点击 `⏹️ Stop Recording`
   - 会弹出成功提示，显示保存的文件名

3. **测试重放**
   - 点击 `🔄` 刷新列表
   - 从下拉菜单选择刚才的录制
   - **确保机械臂周围安全**
   - 点击 `▶️ Start Replay`
   - 机械臂会重复刚才的动作

## 🛡️ 安全特性

- ✅ 随时可以停止录制或重放（点击停止按钮）
- ✅ 重放前必须先选择录制文件
- ✅ 状态实时显示，知道系统在做什么
- ✅ 错误提示清晰，便于排查问题
- ✅ **不影响原有系统**：原始脚本完全未修改

## 📊 数据格式说明

录制文件 `.pkl` 包含：
```python
{
    'metadata': {
        'timestamp': '20231215_143022',
        'sample_count': 3000,           # 数据点数量
        'duration': 30.5               # 持续时间（秒）
    },
    'data': [
        {
            'type': 'joint_state',      # 或 'gripper_state'
            'timestamp': <ROS时间戳>,
            'elapsed': 0.033,           # 经过时间（秒）
            'data': {
                'name': ['joint1', 'joint2', ...],
                'position': [0.1, 0.2, ...],
                'velocity': [0.0, 0.0, ...],
                'effort': [0.0, 0.0, ...]
            }
        },
        ...
    ]
}
```

## 🔧 依赖项

### 系统依赖
- ROS2 Humble
- Python 3.10+
- libfranka 0.18.0
- tkinter（GUI）

### Python依赖
- rclpy（ROS2 Python客户端）
- sensor_msgs（ROS2消息类型）
- std_srvs（ROS2服务类型）
- pickle（数据序列化，Python内置）
- tkinter（GUI，需单独安装）

### 安装缺失依赖

如果GUI无法启动：
```bash
sudo apt-get install python3-tk
```

如果缺少ROS2消息：
```bash
sudo apt-get install ros-humble-sensor-msgs ros-humble-std-srvs
```

## 🐛 常见问题

### Q1: GUI不弹出？
```bash
sudo apt-get install python3-tk
# 然后重新运行
```

### Q2: 录制为空？
检查FR3控制器是否运行：
```bash
ros2 topic echo /franka/joint_states
# 应该看到持续的数据流
```

### Q3: 重放不工作？
检查节点状态：
```bash
ros2 node list | grep franka
# 应该看到 franka_recorder_node 和 franka_replay_node
```

### Q4: 想用回原来的系统？
```bash
# 原始脚本完全未修改，直接使用
./run_fr3_real_ros2.sh
```

## 📖 文档指南

- **快速开始**：`FRANKA_RECORDER_QUICKSTART.md`
- **详细中文说明**：`franka_recorder/USAGE_CN.md`
- **完整技术文档**：`franka_recorder/README.md`
- **安装测试**：运行 `franka_recorder/test_installation.py`

## ✅ 验证清单

在第一次使用前，请确认：

- [ ] 原始系统 `run_fr3_real_ros2.sh` 工作正常
- [ ] ROS2 Humble已正确安装和配置
- [ ] 运行 `test_installation.py` 所有测试通过
- [ ] 可以看到 `/franka/joint_states` topic有数据
- [ ] tkinter已安装（GUI需要）

## 🎉 完成！

您现在拥有一个完整的Franka机械臂录制与重放系统：

1. ✅ **不影响原有功能** - 原始脚本保持不变
2. ✅ **记录真实数据** - 从 `/franka/joint_states` 获取
3. ✅ **一键操作** - GUI简单易用
4. ✅ **安全可靠** - 随时可以停止
5. ✅ **完整文档** - 中英文使用说明

现在可以开始使用了！

```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

祝使用愉快！🚀
