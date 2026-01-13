# Franka机械臂录制与重放系统 - 中文使用说明

## 📖 系统说明

这个系统可以让您在使用GELLO控制Franka FR3机械臂时，记录**真实机械臂**的运动数据，并在之后重放这些轨迹。

### 核心功能
- ✅ 录制真实机械臂数据（从 `/franka/joint_states` topic）
- ✅ 重放录制的轨迹（发布到 `/gello/joint_states` topic）
- ✅ 图形界面一键操作
- ✅ **完全不影响原有功能**

## 🚀 使用步骤

### 步骤1：启动系统

打开终端，运行：

```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

等待几秒钟，会自动弹出GUI控制面板。

### 步骤2：录制轨迹

```
┌─────────────────────────────────────┐
│   Franka Robot Control              │
├─────────────────────────────────────┤
│                                     │
│  Recording                          │
│  ┌───────────────────────────────┐  │
│  │  🔴 Start Recording           │  │  <-- 点击这里开始录制
│  └───────────────────────────────┘  │
│  Status: Ready                      │
│                                     │
└─────────────────────────────────────┘
```

**操作流程：**
1. 点击 `🔴 Start Recording` 按钮
2. 按钮变成红色 `⏹️ Stop Recording`
3. **正常使用GELLO控制机械臂**完成您想要的动作
4. 动作完成后，点击 `⏹️ Stop Recording` 停止录制
5. 系统自动保存录制文件，弹出成功提示

录制文件保存在：`~/gello_software/franka_recordings/franka_recording_日期时间.pkl`

### 步骤3：重放轨迹

```
┌─────────────────────────────────────┐
│  Replay                             │
│  ┌───────────────────────────────┐  │
│  │ Select Recording:             │  │
│  │ ┌─────────────────────┬───┐   │  │
│  │ │ franka_recording... │ 🔄│   │  │  <-- 1. 选择录制文件
│  │ └─────────────────────┴───┘   │  │      2. 点击🔄刷新列表
│  └───────────────────────────────┘  │
│                                     │
│  ┌───────────────────────────────┐  │
│  │  ▶️  Start Replay             │  │  <-- 3. 点击开始重放
│  └───────────────────────────────┘  │
│  Status: Ready                      │
│                                     │
└─────────────────────────────────────┘
```

**操作流程：**
1. 点击 `🔄` 按钮刷新录制文件列表
2. 从下拉菜单选择要重放的录制文件（默认选择最新的）
3. 点击 `▶️ Start Replay` 按钮开始重放
4. 机械臂会自动执行录制的轨迹
5. 如需提前停止，点击 `⏹️ Stop Replay`

## 📊 工作原理

```
录制模式：
┌──────────┐    控制信号    ┌──────────┐    真实数据    ┌──────────┐
│  GELLO   │  ---------->  │  机械臂  │  ---------->  │ 录制节点  │
└──────────┘               └──────────┘               └──────────┘
                                                            │
                                                            ↓
                                                    保存到 .pkl 文件

重放模式：
┌──────────┐    读取文件    ┌──────────┐    控制信号    ┌──────────┐
│录制文件  │  ---------->  │ 重放节点  │  ---------->  │  机械臂  │
└──────────┘               └──────────┘               └──────────┘
                    (发布到 /gello/joint_states)
```

## 🎯 使用场景

1. **演示任务**：录制一次操作，可以重复演示多次
2. **数据收集**：收集机械臂轨迹数据用于分析或训练
3. **自动化流程**：将手动示教的动作自动化
4. **质量控制**：确保每次执行相同的精确动作

## 📁 文件管理

### 查看录制文件

点击GUI底部的 `📁 Open Recordings Folder` 按钮，会自动打开文件管理器。

或者手动打开：
```bash
cd ~/gello_software/franka_recordings
ls -lh
```

### 录制文件命名规则

```
franka_recording_YYYYMMDD_HHMMSS.pkl
                 ^^^^^^^^_^^^^^^
                 日期      时间

例如：franka_recording_20231215_143022.pkl
     表示 2023年12月15日 14:30:22 录制
```

### 删除录制文件

```bash
cd ~/gello_software/franka_recordings
rm franka_recording_20231215_143022.pkl  # 删除特定文件
```

## ⚠️ 重要安全提示

### 录制时
- ✅ 确保机械臂周围无障碍物
- ✅ 使用GELLO正常控制，动作流畅
- ✅ 避免突然的快速运动

### 重放时
- ⚠️ **务必检查环境**：确保机械臂周围无人无物
- ⚠️ **随时准备急停**：保持急停按钮触手可及
- ⚠️ **先测试短轨迹**：第一次重放时，先用短时间的录制测试
- ⚠️ **环境一致性**：重放环境应与录制时相似
- ⚠️ **全程监控**：整个重放过程不要离开

### 如何安全测试
1. 第一次录制只做 **5秒钟** 的简单动作
2. 重放前清空机械臂周围区域
3. 手放在急停按钮上，开始重放
4. 确认安全后，再录制更长的轨迹

## 🔧 故障排除

### 问题1：GUI无法启动

**现象：** 运行脚本后没有窗口弹出

**解决：**
```bash
sudo apt-get update
sudo apt-get install python3-tk
```

### 问题2：录制为空

**现象：** 停止录制时提示"no data recorded"

**检查：**
```bash
# 查看机械臂数据是否在发布
ros2 topic echo /franka/joint_states
```

如果没有数据，说明FR3控制器没有正常运行。

### 问题3：重放不工作

**现象：** 点击重放后机械臂不动

**检查：**
```bash
# 1. 查看节点是否运行
ros2 node list

# 应该看到：
# /franka_recorder_node
# /franka_replay_node

# 2. 查看topic
ros2 topic list | grep gello

# 应该看到：
# /gello/joint_states
```

### 问题4：原始控制不工作了

**解决：** 使用原始脚本
```bash
cd ~/gello_software
./run_fr3_real_ros2.sh  # 这个脚本完全没有被修改
```

## 🎓 高级用法

### 命令行操作（不用GUI）

如果您更喜欢命令行：

```bash
# 开始录制
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: true}"

# 停止录制
ros2 service call /franka_recorder/start_stop std_srvs/srv/SetBool "{data: false}"

# 列出所有录制
ros2 service call /franka_replay/list_recordings std_srvs/srv/Trigger

# 加载指定录制
ros2 service call /franka_replay/load_recording std_msgs/msg/String "{data: 'franka_recording_20231215_143022.pkl'}"

# 开始重放
ros2 service call /franka_replay/start_stop std_srvs/srv/SetBool "{data: true}"

# 停止重放
ros2 service call /franka_replay/start_stop std_srvs/srv/SetBool "{data: false}"
```

### Python脚本读取录制数据

```python
import pickle

# 读取录制文件
with open('franka_recording_20231215_143022.pkl', 'rb') as f:
    data = pickle.load(f)

# 查看元数据
print(f"样本数量: {data['metadata']['sample_count']}")
print(f"持续时间: {data['metadata']['duration']}秒")

# 遍历数据
for sample in data['data']:
    if sample['type'] == 'joint_state':
        positions = sample['data']['position']
        print(f"关节位置: {positions}")
```

## 📞 需要帮助？

1. **查看详细文档**：
   ```bash
   cat ~/gello_software/franka_recorder/README.md
   ```

2. **运行测试脚本**：
   ```bash
   cd ~/gello_software/franka_recorder
   ./test_installation.py
   ```

3. **检查ROS2节点**：
   ```bash
   ros2 node list
   ros2 topic list
   ```

## ✅ 快速检查清单

使用前请确认：
- [ ] ROS2 Humble已安装
- [ ] 原始 `run_fr3_real_ros2.sh` 可以正常运行
- [ ] Python 3.10+ 已安装
- [ ] tkinter 已安装（GUI需要）
- [ ] 所有脚本都有执行权限

全部确认后，就可以开始使用了！

---

**祝使用愉快！如有问题，请参考README.md或联系技术支持。**
