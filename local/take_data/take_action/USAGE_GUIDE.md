# 使用指南 - 一键启动

## 🚀 三种启动方式

### 方式1：双击桌面图标（最简单）

1. 在桌面找到 **"机器人数据采集"** 图标
2. 双击图标
3. 系统自动启动！

### 方式2：使用全局命令（推荐）

打开任意终端，输入：

```bash
robot-data
```

就这么简单！

### 方式3：直接运行脚本

```bash
cd /home/ubuntu/take_data/take_action
./start_robot_data.sh
```

## 📋 全局命令详解

`robot-data` 命令提供多个子命令：

### 启动数据采集

```bash
robot-data              # 默认启动
robot-data start        # 显式启动
```

**自动完成：**
- ✅ 检查并创建conda环境
- ✅ 激活take_data环境
- ✅ 检查Python依赖
- ✅ 测试远程连接
- ✅ SSH连接远程机器
- ✅ 启动 `run_fr3_real_ros2.sh`
- ✅ 部署数据桥接器
- ✅ 开始收集数据

### 停止远程进程

```bash
robot-data stop
```

清理远程机器上的所有进程（ROS2、数据桥接等）

### 查看系统状态

```bash
robot-data status
```

显示：
- 远程主机连接状态
- 正在运行的进程
- 最近的数据文件

### 打开数据目录

```bash
robot-data data
```

在文件管理器中打开数据目录

### 可视化最新数据

```bash
robot-data viz          # 显示图表
robot-data viz --save   # 保存为PNG
```

自动找到最新的HDF5文件并可视化

### 查看帮助

```bash
robot-data help
```

## 🎯 完整工作流程

### 典型使用场景

```bash
# 1. 启动系统
robot-data

# 2. 操作GELLO设备，控制机械臂...
#    数据实时显示在屏幕上

# 3. 按 Ctrl+C 停止并保存数据

# 4. 查看刚才收集的数据
robot-data viz

# 5. 停止远程进程（可选）
robot-data stop
```

## 📊 启动界面说明

当你启动系统时，会看到：

```
╔════════════════════════════════════════════════════════════╗
║                                                            ║
║         机器人数据采集系统 - 一键启动                      ║
║       Robot Data Collection System - Quick Start          ║
║                                                            ║
╚════════════════════════════════════════════════════════════╝

[1/4] 检查Conda环境...
  ✓ 环境已存在

[2/4] 激活Conda环境...
  ✓ 环境已激活: take_data

[3/4] 检查Python依赖...
  ✓ 依赖已安装

[4/4] 检查远程连接...
  ✓ 远程主机 172.16.1.2 可访问

╔════════════════════════════════════════════════════════════╗
║  所有检查通过！正在启动数据采集系统...                     ║
╚════════════════════════════════════════════════════════════╝
```

然后自动进入数据采集界面：

```
============================================
  Robot Data Collection System
============================================

[1/5] 检查远程连接...
✓ Remote host reachable

[2/5] 上传数据桥接器...
✓ Bridge script uploaded

[3/5] 启动远程ROS2系统...
  等待ROS2系统初始化 (15秒)...
✓ ROS2 system started

[4/5] 启动远程数据桥接器...
  等待桥接器启动 (5秒)...
✓ Data bridge started on port 9999

[5/5] 启动本地数据接收器...

============================================
  Collection in progress...
  Press Ctrl+C to stop and save data
============================================

Connecting to 172.16.1.2:9999...
✓ Connected to remote bridge at 172.16.1.2:9999

--- Streaming Data (Press Ctrl+C to stop) ---

[   123 packets |  98.5 Hz] Franka: [ 0.105 -0.081  0.234 -1.687 -0.063  1.620  0.943]
```

## 🛑 停止数据采集

### 正常停止（保存数据）

按 **Ctrl+C**，系统会：
1. 停止接收数据
2. 保存JSON文件
3. 保存HDF5文件
4. 清理远程进程
5. 显示保存位置

### 紧急停止

如果程序卡住，可以：

```bash
# 新开一个终端
robot-data stop
```

## 📁 数据保存位置

数据自动保存到：

```
/home/ubuntu/take_data/take_action/data/
├── session_20241220_123456.json    # JSON格式
├── session_20241220_123456.h5      # HDF5格式
└── plots/                          # 图表（如果保存）
    ├── session_20241220_123456_joints.png
    ├── session_20241220_123456_ee_path.png
    └── session_20241220_123456_error.png
```

## 🔧 常见问题

### Q: 双击桌面图标没反应？

A: 右键图标 → 属性 → 权限 → 勾选"允许作为程序执行"

或者在终端运行：
```bash
chmod +x /home/ubuntu/Desktop/robot-data-collection.desktop
```

### Q: 提示"无法连接到远程主机"？

A: 检查：
1. 远程电脑是否开机
2. 网络是否连接（`ping 172.16.1.2`）
3. SSH服务是否运行（应该已经安装了）

### Q: 想修改远程主机IP？

A: 编辑配置文件：
```bash
nano /home/ubuntu/take_data/take_action/config/config.yaml
```

### Q: 如何只启动远程ROS2，不收集数据？

A: 在远程桌面中直接运行：
```bash
/home/rsj/gello_software/run_fr3_real_ros2.sh
```

### Q: `robot-data` 命令找不到？

A: 重新打开终端，或运行：
```bash
source ~/.bashrc
```

## 💡 高级技巧

### 后台运行

如果想让采集在后台运行（不推荐，因为看不到实时状态）：

```bash
nohup robot-data > /tmp/robot_data.log 2>&1 &
```

### 定时自动停止

采集30分钟后自动停止：

```bash
timeout 1800 robot-data
```

### 批量处理数据

可视化所有数据文件：

```bash
cd /home/ubuntu/take_data/take_action/scripts
for f in ../data/*.h5; do
    python3 visualize_data.py "$f" --all --save
done
```

## 📞 获取帮助

- **快速开始**: `cat /home/ubuntu/take_data/take_action/QUICKSTART.md`
- **详细文档**: `cat /home/ubuntu/take_data/take_action/README.md`
- **性能分析**: `cat /home/ubuntu/take_data/take_action/PERFORMANCE.md`
- **在线帮助**: `robot-data help`

## 🎓 下一步

现在你可以：

1. ✅ 双击桌面图标启动系统
2. ✅ 使用 `robot-data` 命令管理系统
3. ✅ 操作GELLO控制机械臂并自动采集数据
4. ✅ 使用 `robot-data viz` 可视化数据

祝你使用愉快！🎉
