# GELLO + Franka FR3 (ROS2) Setup/Usage Guide (Bilingual)
#
# 本文件包含安装、恢复与使用说明。双语版本。
#
# =========================================
# English
# =========================================
#
# Overview
# This bundle contains:
# - gello_software (this repo)
# - franka_ros2_ws/install (built ROS2 workspace)
# - ~/.local/libfranka-0.18.0 (local libfranka)
# - 99-gello-ftdi.rules (udev rule)
#
# Requirements
# - Ubuntu 22.04 (recommended)
# - ROS 2 Humble installed in /opt/ros/humble
# - Network access to Franka FR3 at 172.16.0.2
# - GELLO USB connected (FTDI device)
#
# Restore (on target machine)
# 1) Extract bundle to home directory:
#    tar -xzf gello_bundle_*.tar.gz -C ~
# 2) Install udev rule:
#    sudo cp ~/99-gello-ftdi.rules /etc/udev/rules.d/
#    sudo udevadm control --reload-rules
#    sudo udevadm trigger
# 3) (Optional) Ensure your user is in dialout:
#    sudo usermod -aG dialout $USER
#    newgrp dialout
#
# Quick Start (Real robot)
#   ~/gello_software/run_fr3_real_ros2.sh
#
# Quick Start (Sim)
#   ~/gello_software/run_fr3_sim.sh
#
# Useful checks
# - USB device:
#   ls -la /dev/gello
# - GELLO topic:
#   ros2 topic hz /gello/joint_states
#
# Notes
# - Do NOT run gello_control and gello_software at the same time.
# - If /gello/joint_states is not publishing, check USB power/connection.
#
# =========================================
# 中文
# =========================================
#
# 概览
# 该打包包含：
# - gello_software（本项目）
# - franka_ros2_ws/install（已编译的 ROS2 工作空间）
# - ~/.local/libfranka-0.18.0（本地编译的 libfranka）
# - 99-gello-ftdi.rules（udev 规则）
#
# 环境要求
# - Ubuntu 22.04（推荐）
# - ROS 2 Humble 已安装在 /opt/ros/humble
# - FR3 机械臂可访问：172.16.0.2
# - GELLO USB 已连接（FTDI 设备）
#
# 恢复步骤（目标机器）
# 1) 解压到家目录：
#    tar -xzf gello_bundle_*.tar.gz -C ~
# 2) 安装 udev 规则：
#    sudo cp ~/99-gello-ftdi.rules /etc/udev/rules.d/
#    sudo udevadm control --reload-rules
#    sudo udevadm trigger
# 3)（可选）确保当前用户在 dialout 组：
#    sudo usermod -aG dialout $USER
#    newgrp dialout
#
# 一键启动（真机）
#   ~/gello_software/run_fr3_real_ros2.sh
#
# 一键启动（仿真）
#   ~/gello_software/run_fr3_sim.sh
#
# 常用检查
# - USB 设备：
#   ls -la /dev/gello
# - GELLO 话题：
#   ros2 topic hz /gello/joint_states
#
# 备注
# - 不要同时运行 gello_control 和 gello_software。
# - 如果 /gello/joint_states 没有发布，请检查 USB 供电/连接。
