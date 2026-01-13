#!/bin/bash
# 三相机视频录制系统 - 一键启动脚本

# 激活conda环境并启动图形化启动器
source /home/ubuntu/anaconda3/bin/activate take_data
cd /home/ubuntu/take_data/take_video
python3 启动器.py
