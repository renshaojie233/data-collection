#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""测试视频保存路径是否正确"""

import os
from datetime import datetime

# 测试路径
video_base_path = "/home/ubuntu/take_data/data/video"

print("=" * 60)
print("测试视频保存路径")
print("=" * 60)

# 检查基础路径
if os.path.exists(video_base_path):
    print(f"✓ 基础路径存在: {video_base_path}")
else:
    print(f"✗ 基础路径不存在: {video_base_path}")

# 检查相机文件夹
for i in range(1, 4):
    camera_folder = f"{video_base_path}/camera_{i}"
    if os.path.exists(camera_folder):
        print(f"✓ 相机{i}文件夹存在: {camera_folder}")
    else:
        print(f"✗ 相机{i}文件夹不存在: {camera_folder}")

# 模拟文件名
print("\n" + "=" * 60)
print("模拟视频文件路径:")
print("=" * 60)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
for i in range(1, 4):
    filename = f"{video_base_path}/camera_{i}/video_{timestamp}.mp4"
    print(f"相机{i}: {filename}")

print("\n" + "=" * 60)
print("测试完成！")
print("=" * 60)
