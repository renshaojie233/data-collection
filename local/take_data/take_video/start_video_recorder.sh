#!/bin/bash

# 三相机视频录制系统启动脚本
clear

echo "╔════════════════════════════════════════════════════╗"
echo "║        三相机视频录制系统                          ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""
echo "正在启动..."
echo ""

# 激活conda环境
source /home/ubuntu/anaconda3/bin/activate take_data

# 检查环境
if [ $? -ne 0 ]; then
    echo "❌ 错误: 无法激活conda环境 take_data"
    echo ""
    read -p "按回车键退出..."
    exit 1
fi

echo "✓ Conda环境已激活: take_data"
echo ""

# 进入脚本目录
cd /home/ubuntu/take_data/take_video

# 检查相机连接
echo "正在检测RealSense相机..."
python -c "import pyrealsense2 as rs; ctx = rs.context(); devices = ctx.query_devices(); print(f'✓ 检测到 {len(devices)} 个RealSense设备')" 2>/dev/null

if [ $? -ne 0 ]; then
    echo "❌ 警告: 无法检测相机，但仍然尝试启动..."
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "正在启动视频录制程序..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 运行程序
python take_video.py

# 程序退出后
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "程序已退出"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
read -p "按回车键关闭窗口..."
