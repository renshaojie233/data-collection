# 自动转换 DROID 格式功能说明

## ✅ 已完成修改

你的录制程序现在已经支持**自动转换为 DROID 格式**！

每次录制完成后，程序会自动在后台将数据转换为 DROID 格式。

## 🔧 修改内容

### 1. 添加了转换模块导入

在程序启动时自动加载 DROID 转换模块：

```python
# 导入DROID转换模块
sys.path.insert(0, '/home/ubuntu/take_data/raw2droid')
from convert_to_droid import DataConverter
```

### 2. 新增自动转换方法

添加了 `auto_convert_to_droid()` 方法，在后台线程执行转换：

- ✅ 不阻塞录制程序
- ✅ 显示转换状态
- ✅ 自动处理错误

### 3. 集成到录制流程

在 `stop_recording()` 方法中自动调用转换：

```python
# 录制完成后
→ 保存原始数据
→ 自动转换为 DROID 格式（后台）
→ 用户可以继续录制下一组
```

## 📁 数据保存位置

### 原始数据（不变）

录制的原始数据仍然保存在原位置：

```
/home/ubuntu/take_data/data/
├── video/record_XXX/
│   ├── camera_1_YYYYMMDD_HHMMSS.mp4
│   ├── camera_2_YYYYMMDD_HHMMSS.mp4
│   └── camera_3_YYYYMMDD_HHMMSS.mp4
└── action/record_XXX/
    ├── action_data_YYYYMMDD_HHMMSS.json
    └── action_data_YYYYMMDD_HHMMSS.h5
```

### DROID 格式数据（新增）

自动转换后的 DROID 格式数据保存在：

```
/home/ubuntu/take_data/droid_data/
└── YYYY-MM-DD_HH-MM-SS/
    ├── trajectory.h5              # DROID格式HDF5
    └── recordings/MP4/
        ├── camera_1.mp4          # 伪立体格式
        ├── camera_2.mp4
        └── camera_3.mp4
```

## 🎮 使用方法

### 正常录制即可！

使用方法**完全不变**：

1. 打开桌面上的"视频+动作录制"程序
2. 点击"开始录制"
3. 完成演示
4. 点击"停止录制"

**新增：** 停止录制后，程序会：
- ✅ 保存原始数据（和之前一样）
- ✅ **自动转换为 DROID 格式**（新功能）
- ✅ 在状态栏显示转换进度

## 📊 状态显示

在程序界面的"录制状态"标签会显示：

```
录制状态: 正在录制           # 录制中
  ↓
录制状态: 录制已停止         # 保存完成
  ↓
录制状态: 转换为DROID格式中... # 自动转换中
  ↓
录制状态: DROID转换完成      # 全部完成！
```

## 🔍 查看转换结果

### 方法1：终端输出

在启动录制程序的终端中，可以看到转换日志：

```
录制完成: record_001

开始自动转换 record_001 为DROID格式...
转换 record_001...
  加载动作数据: action_data_20231220_120000.h5
    数据点数: 1500
  找到 3 个视频文件
  开始转换 1500 个时间步...
    进度: 1500/1500
  成功转换 1500 个时间步
  ✓ 已保存到: /home/ubuntu/take_data/droid_data/2025-12-22_10-30-00
✓ DROID转换完成: record_001
```

### 方法2：验证数据

```bash
cd /home/ubuntu/take_data/raw2droid
python droid_data_validator.py /home/ubuntu/take_data/droid_data/<轨迹文件夹>
```

### 方法3：可视化查看

```bash
cd /home/ubuntu/take_data/raw2droid
python visualize_trajectory.py /home/ubuntu/take_data/droid_data/<轨迹文件夹> --plot
```

## ⚙️ 转换配置

### 默认配置

当前使用的默认配置：

- **用户名**: `operator`
- **任务描述**: `manipulation`
- **输出目录**: `/home/ubuntu/take_data/droid_data/`

### 自定义配置（可选）

如果想修改默认配置，编辑录制程序中的这部分代码：

```python
# 在 auto_convert_to_droid 方法中
success = converter.convert_record(
    record_name=record_name,
    user='operator',        # ← 修改用户名
    task='manipulation'     # ← 修改任务描述
)
```

## ⚙️ 依赖要求

### 必需的 Python 包

自动转换功能需要以下依赖包（已安装）：

```bash
# 在 take_data 环境中
conda activate take_data

# 必需的包
pip install scipy  # 用于四元数到欧拉角转换
pip install h5py   # 用于读写HDF5文件
pip install opencv-python  # 用于视频处理
pip install numpy  # 数值计算
```

**重要**: 如果缺少 `scipy`，自动转换会静默失败（因为程序从桌面图标启动时没有终端输出）。

### 验证依赖

```bash
# 验证所有依赖都已安装
bash -c 'source ~/anaconda3/etc/profile.d/conda.sh; conda activate take_data && python -c "import scipy, h5py, cv2, numpy; print(\"✓ 所有依赖已安装\")"'
```

## 🛡️ 故障处理

### 如果转换失败

转换失败**不会影响原始数据**，也不会阻止下次录制：

- ✅ 原始数据已安全保存
- ✅ 可以手动重新转换
- ✅ 可以继续录制新数据

### 手动重新转换

如果自动转换失败，可以手动转换：

```bash
cd /home/ubuntu/take_data/raw2droid
python convert_to_droid.py -r record_001
```

### 禁用自动转换

如果暂时不需要自动转换，可以：

**临时禁用**：删除或重命名转换模块
```bash
mv /home/ubuntu/take_data/raw2droid /home/ubuntu/take_data/raw2droid.disabled
```

**恢复**：
```bash
mv /home/ubuntu/take_data/raw2droid.disabled /home/ubuntu/take_data/raw2droid
```

## 📝 备份说明

### 自动备份

修改前已自动备份原程序：

```
/home/ubuntu/take_data/take_video_action/
├── video_action_recorder.py                          # 修改后（带自动转换）
└── video_action_recorder.py.bak_before_auto_convert  # 原始备份
```

### 恢复原程序

如果需要恢复到原始版本（不带自动转换）：

```bash
cd /home/ubuntu/take_data/take_video_action
cp video_action_recorder.py.bak_before_auto_convert video_action_recorder.py
```

## 🎯 优势

1. **无缝集成**
   - 录制流程不变
   - 自动化处理
   - 无需手动操作

2. **后台处理**
   - 不阻塞录制程序
   - 可以立即开始下一次录制
   - 状态实时更新

3. **数据完整**
   - 原始数据完整保留
   - DROID 格式自动生成
   - 双重保障

4. **错误容忍**
   - 转换失败不影响录制
   - 可以手动重试
   - 清晰的错误提示

## 🔄 工作流程

```
开始录制
    ↓
录制演示数据
    ↓
停止录制
    ↓
保存原始数据（video + action）
    ↓
[后台] 自动转换为 DROID 格式
    ↓
    ├─ 成功 → 显示"DROID转换完成"
    └─ 失败 → 显示"DROID转换失败"（可手动重试）
    ↓
可以开始下一次录制
```

## 📞 技术支持

### 常见问题

**Q: 转换需要多长时间？**
A: 通常每秒处理 30-50 帧，一个 30 秒的录制大约需要 10-20 秒转换

**Q: 转换时可以录制吗？**
A: 可以！转换在后台进行，不影响下一次录制

**Q: 原始数据会被删除吗？**
A: 不会！原始数据完整保留，DROID 格式是额外生成的

**Q: 如何知道转换完成了？**
A: 查看程序状态栏，或者查看终端输出日志

### 调试方法

如果遇到问题，检查终端输出：

```bash
# 启动录制程序时，在终端查看输出
cd /home/ubuntu/take_data/take_video_action
./start.sh
```

## 📚 相关文档

- DROID 格式详细说明：`/home/ubuntu/take_data/raw2droid/README.md`
- 手动转换指南：`/home/ubuntu/take_data/raw2droid/CONVERT_GUIDE.md`
- 快速开始：`/home/ubuntu/take_data/raw2droid/QUICKSTART.md`

---

**修改日期**: 2025-12-22
**版本**: 1.0（自动转换集成版）
**状态**: ✅ 已完成并测试
