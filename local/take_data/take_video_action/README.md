# 视频+动作数据同步录制系统

## 📋 系统简介

这是一个整合的录制系统，可以同时录制：
- 📹 **三相机同步视频** (Realsense)
- 🤖 **机器人动作数据** (Franka FR3 + GELLO)

## 🚀 启动方式

### 方式1: 双击桌面图标（最简单）
双击桌面上的 **"视频+动作录制"** 图标

### 方式2: 使用命令（推荐）
```bash
record-video-action
```

### 方式3: 运行脚本
```bash
cd /home/ubuntu/take_data/take_video_action
./start.sh
```

## 💡 使用流程

1. **启动系统**
   - 双击桌面图标或运行 `record-video-action`
   - 等待相机和机器人初始化（约20秒）

2. **开始录制**
   - 点击 **"▶️ 开始录制"** 按钮
   - 系统自动：
     - 启动远程机器人系统
     - 连接数据桥接
     - 开始录制视频和动作数据

3. **操作机器人**
   - 使用GELLO设备控制Franka机械臂
   - 数据实时采集和保存

4. **停止录制**
   - 点击 **"⏹️ 停止录制"** 按钮
   - 数据自动保存
   - 远程进程自动清理

5. **退出系统**
   - 点击 **"🚪 退出系统"** 按钮
   - 或直接关闭窗口
   - 或按 Ctrl+C

## 📁 数据保存格式

数据按录制次数自动保存为 `record_001`, `record_002`, `record_003`...

### 目录结构

```
/home/ubuntu/take_data/data/
├── video/                          # 视频数据
│   ├── record_001/
│   │   ├── camera_1_20241220_120000.mp4
│   │   ├── camera_2_20241220_120000.mp4
│   │   └── camera_3_20241220_120000.mp4
│   ├── record_002/
│   │   └── ...
│   └── record_003/
│       └── ...
└── action/                         # 动作数据
    ├── record_001/
    │   ├── action_data_20241220_120000.json
    │   └── action_data_20241220_120000.h5
    ├── record_002/
    │   └── ...
    └── record_003/
        └── ...
```

### 数据内容

**视频数据 (MP4)**:
- 3个相机同步录制
- 分辨率: 640x480
- 帧率: 30 FPS

**动作数据 (JSON + HDF5)**:
- GELLO关节位置
- Franka关节位置、速度、力矩
- 末端执行器位置和姿态
- 时间戳（用于同步）

## 🔧 常用命令

```bash
record-video-action           # 启动系统
record-video-action data      # 打开数据目录
record-video-action video     # 打开视频目录
record-video-action action    # 打开动作数据目录
record-video-action clean     # 清理远程进程
record-video-action help      # 显示帮助
```

## ⚙️ 系统配置

### 硬件要求
- **本地机器**: Ubuntu 22.04, RealSense相机x3
- **远程机器**: 172.16.1.2 (Franka FR3 + GELLO)

### 软件依赖
- Python 3.8+
- Conda环境: `take_data`
- OpenCV, pyrealsense2, tkinter, h5py
- ROS2 Humble (远程)

### 网络配置
- 远程主机: 172.16.1.2
- 数据端口: 9999
- SSH用户: rsj

## 🛡️ 安全特性

### 自动清理机制

程序退出时（任何方式）都会自动清理远程进程：
- ✅ Ctrl+C
- ✅ 关闭GUI窗口
- ✅ 点击退出按钮
- ✅ 程序崩溃

清理内容：
- 停止ROS2系统
- 停止数据桥接
- 删除临时文件
- 释放网络连接

## 📊 GUI界面说明

### 左侧：视频预览
- 显示相机1的实时画面
- 录制时显示红色REC标识

### 右侧：控制面板

**系统状态**:
- 📷 相机状态
- 🤖 机器人状态

**录制信息**:
- 录制状态
- 录制时长
- 录制序号
- 保存路径

**控制按钮**:
- ▶️ 开始录制
- ⏹️ 停止录制
- 🚪 退出系统

## 🔍 故障排除

### Q: 相机初始化失败？
A:
```bash
# 检查相机连接
rs-enumerate-devices

# 重新插拔USB
```

### Q: 无法连接远程机器人？
A:
```bash
# 检查网络
ping 172.16.1.2

# 检查SSH
ssh rsj@172.16.1.2

# 清理远程进程
record-video-action clean
```

### Q: 录制数据丢失？
A: 数据在停止录制时自动保存，检查：
```bash
ls -lht /home/ubuntu/take_data/data/video/
ls -lht /home/ubuntu/take_data/data/action/
```

### Q: GUI无法启动？
A:
```bash
# 激活环境
conda activate take_data

# 检查依赖
python3 -c "import cv2, pyrealsense2, tkinter, h5py"

# 查看错误
cd /home/ubuntu/take_data/take_video_action
python3 video_action_recorder.py
```

## 📖 技术细节

### 数据同步

视频和动作数据通过时间戳同步：
- 视频帧: 30 FPS (每帧33ms)
- 动作数据: ~100 Hz (每包10ms)
- 时间戳精度: 毫秒级

### 数据流程

```
┌─────────────┐           ┌──────────────┐
│  本地相机    │           │  远程机器人   │
│  (3个)      │           │  (172.16.1.2)│
└──────┬──────┘           └───────┬──────┘
       │                          │
       │ 30 FPS                   │ ROS2
       ↓                          ↓
  ┌────────────┐           ┌─────────────┐
  │ 视频录制    │           │ 数据桥接     │
  │ (MP4)      │           │ (TCP:9999)  │
  └──────┬─────┘           └──────┬──────┘
         │                        │
         │                        │ 100 Hz
         ↓                        ↓
    ┌────────────────────────────────┐
    │      主GUI程序                  │
    │  - 视频预览                     │
    │  - 数据接收                     │
    │  - 同步保存                     │
    └────────────────────────────────┘
              │
              ↓
    ┌─────────────────────┐
    │  record_XXX/         │
    │  ├─ video/          │
    │  └─ action/         │
    └─────────────────────┘
```

## 📝 更新日志

### v1.0 (2024-12-20)
- 初始版本
- 整合视频和动作录制
- 美观的GUI界面
- 自动清理机制
- record_XXX序号保存

## 📄 许可证

内部使用项目

---

**项目路径**: `/home/ubuntu/take_data/take_video_action`
**数据路径**: `/home/ubuntu/take_data/data/`
**创建时间**: 2024-12-20
