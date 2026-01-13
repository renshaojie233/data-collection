# 三相机视频录制系统

## 环境安装

### 1. 创建conda环境
```bash
cd /home/ubuntu/take_data/take_video
conda env create -f environment.yml
```

### 2. 激活环境
```bash
conda activate take_data
```

### 3. 如果需要更新环境
```bash
conda env update -f environment.yml --prune
```

## 使用方法

### 运行程序
```bash
conda activate take_data
python take_video.py
```

## 功能说明

1. **开始录制**: 点击"开始录制"按钮，三个相机同时开始录制视频
2. **停止录制**: 点击"停止录制"按钮，停止录制并保存视频文件
3. **实时预览**: 程序会显示三个相机的实时预览画面（水平拼接）
4. **录制时长**: GUI界面显示当前录制时长
5. **退出程序**: 点击"退出程序"按钮或关闭窗口

## 视频保存位置

- 相机1的视频: `/home/ubuntu/take_data/data/video/camera_1/video_时间戳.mp4`
- 相机2的视频: `/home/ubuntu/take_data/data/video/camera_2/video_时间戳.mp4`
- 相机3的视频: `/home/ubuntu/take_data/data/video/camera_3/video_时间戳.mp4`

## 注意事项

- 确保三个RealSense相机已正确连接
- 录制过程中会在预览窗口显示红色"REC"标识
- 可以按'q'键关闭预览窗口
- 退出程序前会自动停止录制并保存视频

## 卸载环境

如需删除conda环境：
```bash
conda deactivate
conda env remove -n take_data
```
