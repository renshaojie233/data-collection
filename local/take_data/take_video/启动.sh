#!/bin/bash
# 三相机视频录制系统 - 双击启动

# 确保在正确的目录
cd /home/ubuntu/take_data/take_video

# 激活conda环境并启动
source /home/ubuntu/anaconda3/bin/activate take_data
python3 take_video.py

# 如果程序关闭，保持终端打开
if [ $? -ne 0 ]; then
    echo ""
    echo "启动失败，按回车键退出..."
    read
fi
