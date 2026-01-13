# 快速开始指南

## 1. 安装环境（仅首次）

```bash
cd /home/ubuntu/take_data/take_action
conda env create -f environment.yml
```

## 2. 激活环境

```bash
conda activate take_data
```

## 3. 开始收集数据

```bash
cd /home/ubuntu/take_data/take_action/scripts
./start_collection.sh
```

这个命令会自动：
- ✅ 连接到远程机器 (172.16.1.2)
- ✅ 启动 GELLO → Franka FR3 控制系统
- ✅ 开始实时传输和保存数据

## 4. 停止收集

按 **Ctrl+C** 停止，数据会自动保存到 `data/` 目录。

## 5. 查看数据

### 方法1：查看统计信息和可视化

```bash
python3 visualize_data.py ../data/session_20241220_123456.h5 --all
```

### 方法2：快速查看统计

```bash
python3 visualize_data.py ../data/session_20241220_123456.h5
```

### 方法3：保存图表为PNG

```bash
python3 visualize_data.py ../data/session_20241220_123456.h5 --all --save
```

## 常见问题

### Q: 如何查看最新的数据文件？

```bash
ls -lht ../data/  # 按时间排序显示
```

### Q: 如何检查数据收集状态？

在收集过程中，屏幕会实时显示：
```
[   123 packets |  98.5 Hz] Franka: [ 0.105 -0.081  0.234 -1.687 -0.063  1.620  0.943]
```

### Q: 数据保存在哪里？

- JSON格式：`/home/ubuntu/take_data/take_action/data/session_*.json`
- HDF5格式：`/home/ubuntu/take_data/take_action/data/session_*.h5`
- 图表（如果保存）：`/home/ubuntu/take_data/take_action/data/plots/`

### Q: 如何手动清理远程进程？

```bash
ssh rsj@172.16.1.2 'pkill -f remote_data_bridge; pkill -f run_fr3_real_ros2'
```

## 数据使用示例

### Python读取HDF5数据

```python
import h5py
import numpy as np

with h5py.File('data/session_20241220_123456.h5', 'r') as f:
    timestamps = f['timestamps'][:]
    franka_positions = f['franka/positions'][:]

    # 打印基本信息
    print(f"样本数: {len(timestamps)}")
    print(f"时长: {timestamps[-1] - timestamps[0]:.2f} 秒")

    # 分析关节1的运动
    joint1_pos = franka_positions[:, 0]
    print(f"关节1范围: [{joint1_pos.min():.3f}, {joint1_pos.max():.3f}] rad")
```

### Python读取JSON数据

```python
import json

with open('data/session_20241220_123456.json', 'r') as f:
    data = json.load(f)

print(f"采集包数: {data['packet_count']}")

# 打印前5个时间点的数据
for i, item in enumerate(data['data'][:5]):
    print(f"\n时间点 {i+1}: {item['datetime']}")
    print(f"机械臂关节位置: {item['franka_joints']['position']}")
```

## 提示

1. **收集前确认**：确保GELLO设备已连接，机械臂已开机
2. **数据备份**：定期备份 `data/` 目录
3. **命名规范**：数据文件自动按时间命名，无需手动重命名
4. **网络要求**：确保本机和远程机器 (172.16.1.2) 网络畅通

## 技术支持

查看详细文档：`README.md`
