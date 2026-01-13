# 🚀 自动对齐快速开始 | Auto-Alignment Quick Start

版本: 3.0 | 2025-12-22

---

## ⚡ 快速使用

### 1️⃣ 启动系统
```bash
cd ~/gello_software
./run_fr3_with_recorder.sh
```

### 2️⃣ 录制轨迹
1. 点击 **🔴 Start Recording**
2. 用GELLO控制机械臂
3. 点击 **⏹️ Stop Recording**

### 3️⃣ 重放轨迹（自动对齐）
1. 选择录制文件
2. 点击 **▶️ Start Replay**
3. 等待自动完成 (~5秒对齐 + 轨迹时长)

**✨ 就这么简单！无需手动移动到起始位置！**

---

## 🎯 核心特性

### ✅ 自动对齐
- 系统自动移动到轨迹起始位置
- 5秒平滑过渡
- 无需手动操作

### ✅ 安全可靠
- 不会触发红灯保护
- 平滑加减速
- 随时可中断

### ✅ 简单易用
- 一键录制
- 一键重放
- 无需专业知识

---

## 📊 工作流程

```
录制: 点击 → GELLO操作 → 保存
       ↓
重放: 选择文件 → 点击重放
       ↓
自动执行:
  Phase 1: 自动对齐 (5秒)
  Phase 2: 重放轨迹
       ↓
完成！
```

---

## 🔍 观察进度

启动后观察终端输出：

```
[INFO] Phase 1: Auto-aligning to start position (diff: 0.456 rad, duration: 5.0s)...
[INFO] Alignment progress: 20.0%
[INFO] Alignment progress: 40.0%
[INFO] Alignment progress: 60.0%
[INFO] Alignment progress: 80.0%
[INFO] ✓ Alignment complete, ready to replay
[INFO] Phase 2: Replaying recorded trajectory...
[INFO] Replay progress: 100/198 samples
[INFO] ✓ Replay completed successfully
```

---

## 💡 重要提示

### ✅ 要做的
- ✅ 直接点击重放（无需手动对齐）
- ✅ 观察终端进度
- ✅ 确保周围无障碍物
- ✅ 保持急停按钮可触及

### ❌ 不要做的
- ❌ 不要在对齐时移动GELLO
- ❌ 不要录制过快的动作
- ❌ 不要忽略错误信息

---

## 🧪 测试系统

```bash
cd ~/gello_software/franka_recorder

# 启动节点（如果未运行）
python3 recorder_node.py &
python3 replay_node.py &

# 运行测试
python3 test_auto_alignment.py
```

---

## 🔧 常见问题

**Q: 需要手动移动到起始位置吗？**
A: ❌ 不需要！系统会自动对齐。

**Q: 对齐需要多长时间？**
A: ⏱️ 默认5秒，可在代码中调整。

**Q: 会不会触发红灯？**
A: ❌ 不会！我们使用平滑插值，完全安全。

**Q: 可以中途停止吗？**
A: ✅ 可以！随时点击停止按钮。

**Q: 机械臂可以在任意位置吗？**
A: ✅ 是的！系统会自动对齐。

---

## 📚 详细文档

- **AUTO_ALIGNMENT_FEATURE.md** - 完整技术文档
- **README.md** - 系统总览
- **USAGE_CN.md** - 详细使用说明

---

## 🆘 需要帮助？

1. 查看终端错误信息
2. 运行测试脚本诊断
3. 检查录制文件是否存在
4. 确认所有节点正常运行

---

**版本**: 3.0
**状态**: ✅ 生产就绪
**推荐指数**: ⭐⭐⭐⭐⭐

**一句话总结**: 点击重放，系统自动对齐并执行轨迹，无需任何手动操作！
