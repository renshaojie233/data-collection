#!/bin/bash
# 三相机视频录制系统启动脚本

echo "========================================="
echo "    三相机视频录制系统"
echo "========================================="
echo ""

# 激活conda环境
source /home/ubuntu/anaconda3/bin/activate take_data

# 进入脚本目录
cd /home/ubuntu/take_data/take_video

# 运行程序
python take_video.py

# 退出时提示
echo ""
echo "程序已退出"
