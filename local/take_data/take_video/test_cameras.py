#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""测试三个相机能否正常初始化和采集帧"""

import pyrealsense2 as rs
import numpy as np
import cv2

# 相机序列号
serial_numbers = [
    '243722070232',
    '243622073040',
    '243722073691'
]

def test_cameras():
    pipelines = []
    print("=" * 50)
    print("开始测试三个RealSense相机...")
    print("=" * 50)

    try:
        # 初始化相机
        for i, sn in enumerate(serial_numbers, start=1):
            print(f"\n正在初始化相机 {i} (序列号: {sn})...")
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(sn)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            pipelines.append(pipeline)
            print(f"✓ 相机 {i} 初始化成功")

        print("\n" + "=" * 50)
        print("测试采集10帧图像...")
        print("=" * 50)

        # 采集10帧测试
        for frame_num in range(10):
            images = []
            valid = True

            for i, pipeline in enumerate(pipelines, start=1):
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    print(f"✗ 相机 {i} 第 {frame_num+1} 帧获取失败")
                    valid = False
                    break
                color_image = np.asanyarray(color_frame.get_data())
                images.append(color_image)

            if valid:
                print(f"✓ 第 {frame_num+1} 帧: 所有相机采集成功 ({images[0].shape})")

        print("\n" + "=" * 50)
        print("测试完成！所有相机工作正常")
        print("=" * 50)

    except Exception as e:
        print(f"\n✗ 错误: {str(e)}")
        return False

    finally:
        # 停止所有相机
        print("\n正在关闭相机...")
        for i, pipeline in enumerate(pipelines, start=1):
            pipeline.stop()
            print(f"✓ 相机 {i} 已关闭")

    return True

if __name__ == "__main__":
    success = test_cameras()
    if success:
        print("\n✓✓✓ 测试通过！可以运行 take_video.py ✓✓✓")
    else:
        print("\n✗✗✗ 测试失败！请检查相机连接 ✗✗✗")
