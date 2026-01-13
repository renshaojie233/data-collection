#!/bin/bash
# GELLO udev 规则安装脚本
# 此脚本将为 GELLO 设备创建固定的设备符号链接 /dev/gello

set -e

echo "=================================================="
echo "GELLO USB 设备固定名称安装脚本"
echo "=================================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "错误: 此脚本需要 root 权限"
    echo "请使用以下命令运行:"
    echo "  sudo bash $0"
    exit 1
fi

RULE_FILE="/home/rsj/gello_software/99-gello.rules"
DEST_FILE="/etc/udev/rules.d/99-gello.rules"

if [ ! -f "$RULE_FILE" ]; then
    echo "错误: 找不到 udev 规则文件: $RULE_FILE"
    exit 1
fi

echo "1. 复制 udev 规则文件..."
cp "$RULE_FILE" "$DEST_FILE"
echo "   ✓ 已复制到 $DEST_FILE"

echo ""
echo "2. 重新加载 udev 规则..."
udevadm control --reload-rules
echo "   ✓ udev 规则已重新加载"

echo ""
echo "3. 触发 udev 事件..."
udevadm trigger
echo "   ✓ udev 事件已触发"

echo ""
echo "4. 等待设备重新识别..."
sleep 2

echo ""
echo "5. 检查设备..."
if [ -L "/dev/gello" ]; then
    echo "   ✓ 成功! /dev/gello 符号链接已创建"
    ls -la /dev/gello
    echo ""
    echo "   实际设备: $(readlink -f /dev/gello)"
else
    echo "   ⚠ 警告: /dev/gello 尚未创建"
    echo ""
    echo "   可能的原因:"
    echo "   1. GELLO 设备未连接"
    echo "   2. 设备序列号不匹配"
    echo ""
    echo "   请检查设备是否已连接,然后重新拔插 USB 或运行:"
    echo "     sudo udevadm trigger"
fi

echo ""
echo "=================================================="
echo "安装完成!"
echo "=================================================="
echo ""
echo "现在你可以在程序中使用 /dev/gello 作为串口设备"
echo "这个名称不会因为重启或拔插而改变。"
