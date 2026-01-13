# 🚀 开始使用 - START HERE

欢迎使用机器人数据采集系统！这是最简单的入门指南。

## ⚡ 快速开始（三选一）

### 🖱️ 方式1：双击桌面图标（最简单！）

1. 在桌面找到 **"机器人数据采集"** 图标 📊
2. **双击** 图标
3. 完成！✅

---

### ⌨️ 方式2：使用命令（推荐！）

打开终端，输入：

```bash
robot-data
```

就这么简单！🎉

---

### 📁 方式3：运行脚本

```bash
cd /home/ubuntu/take_data/take_action
./start_robot_data.sh
```

---

## 🎬 启动后会发生什么？

系统会自动：

1. ✅ 检查环境配置
2. ✅ 连接远程机器 (172.16.1.2)
3. ✅ 启动机器人ROS2系统
4. ✅ 开始采集数据
5. ✅ 实时显示状态

你会看到类似这样的界面：

```
[   123 packets |  98.5 Hz] Franka: [ 0.105 -0.081  0.234 ...]
```

这表示数据正在以 **~100 Hz** 的频率采集！

---

## 🛑 如何停止？

按 **Ctrl+C**

数据会自动保存到：`/home/ubuntu/take_data/take_action/data/`

---

## 📊 查看数据

### 可视化最新数据

```bash
robot-data viz
```

### 打开数据文件夹

```bash
robot-data data
```

或直接进入：`/home/ubuntu/take_data/take_action/data/`

---

## 🔧 常用命令

```bash
robot-data          # 启动采集
robot-data stop     # 停止远程进程
robot-data status   # 查看状态
robot-data viz      # 可视化数据
robot-data help     # 查看帮助
```

---

## 📚 需要更多帮助？

- **使用指南**: `USAGE_GUIDE.md` - 详细使用说明
- **快速入门**: `QUICKSTART.md` - 快速上手
- **完整文档**: `README.md` - 完整技术文档
- **性能分析**: `PERFORMANCE.md` - 性能指标
- **项目总结**: `PROJECT_SUMMARY.md` - 项目概览

查看文档：

```bash
cd /home/ubuntu/take_data/take_action
cat USAGE_GUIDE.md    # 推荐先看这个！
```

---

## ❓ 遇到问题？

### 桌面图标点不了？

```bash
chmod +x /home/ubuntu/Desktop/robot-data-collection.desktop
```

### robot-data命令找不到？

```bash
source ~/.bashrc
```

然后重新打开终端

### 连不上远程机器？

```bash
ping 172.16.1.2
```

检查网络和远程电脑是否开机

---

## 🎯 典型工作流程

```bash
# 1️⃣ 启动系统
robot-data

# 2️⃣ 操作GELLO设备控制机械臂
#    （数据自动采集中...）

# 3️⃣ 完成后按 Ctrl+C 停止并保存

# 4️⃣ 查看数据
robot-data viz

# 5️⃣ 清理（可选）
robot-data stop
```

---

## 🎉 准备好了吗？

现在就试试吧：

```bash
robot-data
```

或者双击桌面图标 **"机器人数据采集"** ！

祝你使用愉快！🚀✨

---

**项目路径**: `/home/ubuntu/take_data/take_action`
**数据保存**: `/home/ubuntu/take_data/take_action/data`
