# Franka录制与重放功能 - 快速开始

## 🚀 快速使用

### 方法1：集成启动（推荐）

直接运行增强版启动脚本，包含原有功能 + 录制重放功能：

```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

这个脚本会：
1. ✅ 启动所有原有的GELLO和FR3节点（功能完全不变）
2. ✅ 启动录制和重放节点
3. ✅ 打开GUI控制面板

### 方法2：单独启动

如果你已经运行了 `run_fr3_real_ros2.sh`，可以单独启动录制重放系统：

```bash
cd ~/gello_software/franka_recorder
./launch_recorder.sh
```

## 📹 如何使用

### 录制轨迹

1. 点击 **"🔴 Start Recording"** 按钮开始录制
2. 正常使用GELLO控制机械臂
3. 点击 **"⏹️ Stop Recording"** 停止录制
4. 录制文件自动保存到 `~/gello_software/franka_recordings/`

### 重放轨迹

1. 点击 **"🔄"** 刷新录制列表
2. 从下拉菜单选择一个录制文件
3. 点击 **"▶️ Start Replay"** 开始重放
4. 机械臂将执行录制的轨迹
5. 可随时点击 **"⏹️ Stop Replay"** 停止

## 🎯 核心特点

- ✅ **不影响原有功能**：`run_fr3_real_ros2.sh` 完全不变
- ✅ **记录真实机械臂数据**：记录的是 `/franka/joint_states`（真实机械臂），不是GELLO数据
- ✅ **一键操作**：GUI提供简单的录制和重放按钮
- ✅ **使用conda环境**：独立的Python环境，不影响系统
- ✅ **安全可靠**：可随时停止录制或重放

## 📁 文件说明

```
gello_software/
├── run_fr3_real_ros2.sh           # 原有脚本（未修改）
├── run_fr3_with_recorder.sh       # 新增：集成录制功能的启动脚本
├── franka_recorder/
│   ├── recorder_node.py           # 录制节点
│   ├── replay_node.py             # 重放节点
│   ├── control_gui.py             # GUI控制面板
│   ├── launch_recorder.sh         # 单独启动录制系统
│   ├── environment.yaml           # Conda环境配置
│   └── README.md                  # 详细说明文档
└── franka_recordings/             # 录制文件存储目录（自动创建）
    └── franka_recording_YYYYMMDD_HHMMSS.pkl
```

## 🔧 安装

### 安装conda环境（可选但推荐）

```bash
cd ~/gello_software/franka_recorder
conda env create -f environment.yaml
conda activate franka_recorder
```

### 安装tkinter（如果GUI无法启动）

```bash
sudo apt-get install python3-tk
```

## ⚠️ 安全提示

1. **重放前检查环境**：确保机械臂周围无障碍物
2. **保持监控**：重放时始终监控机械臂运动
3. **急停准备**：确保急停按钮随时可触
4. **从短轨迹开始**：先测试短时间录制
5. **环境一致性**：重放时环境应与录制时相似

## 📊 数据格式

录制文件包含：
- 关节位置 (position)
- 关节速度 (velocity)
- 关节力矩 (effort)
- 时间戳信息
- 夹爪状态

文件保存为pickle格式，可用Python直接读取分析。

## 🐛 故障排除

### GUI无法启动
```bash
sudo apt-get install python3-tk
```

### 录制为空
```bash
# 检查topic是否发布
ros2 topic echo /franka/joint_states
```

### 重放不工作
```bash
# 检查节点是否运行
ros2 node list
# 检查服务是否可用
ros2 service list | grep franka
```

## 💡 高级用法

### 命令行控制（不使用GUI）

```bash
# 开始录制
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: true}"

# 停止录制
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: false}"

# 列出录制文件
ros2 service call /franka_replay/list_recordings std_srvs/srv/Trigger

# 开始重放
ros2 service call /franka_replay/start_stop std_srvs/srv/SetBool "{data: true}"
```

## 📞 技术支持

详细文档请查看：`~/gello_software/franka_recorder/README.md`

## ✅ 验证安装

运行以下命令验证系统是否正常：

```bash
cd ~/gello_software/franka_recorder
python3 -c "import rclpy; import tkinter; print('✅ All dependencies OK')"
```

如果没有错误，说明安装成功！
