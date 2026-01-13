#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三相机视频录制系统 - 集成GUI版本"""

import cv2
import pyrealsense2 as rs
import time
import numpy as np
import os
import shutil
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, font as tkfont
import threading
from PIL import Image, ImageTk

# 相机序列号
serial_numbers = [
    '243722070232',  # 相机1
    '243622073040',  # 相机2
    '243722073691'   # 相机3
]

class VideoRecorder:
    def __init__(self):
        self.pipelines = []
        self.is_recording = False
        self.writers = []
        self.running = True
        self.current_frame = None
        self.frame_lock = threading.Lock()  # 添加线程锁防止闪烁

        # 视频保存路径
        self.video_base_path = "/home/ubuntu/take_data/data/video"
        os.makedirs(self.video_base_path, exist_ok=True)
        self.migrate_legacy_folders()
        self.session_dir = None
        self.session_index = None
        self.session_timestamp = None

        # 创建GUI
        self.create_gui()

        # 初始化相机
        self.init_cameras()

        # 开始预览
        self.start_preview()

    def init_cameras(self):
        """初始化所有相机"""
        try:
            for sn in serial_numbers:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(sn)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(config)
                self.pipelines.append(pipeline)
                print(f"相机 {sn} 初始化成功")

            self.camera_status_label.config(text="相机状态: ✓ 3个相机已连接", fg="#27ae60")
        except Exception as e:
            self.camera_status_label.config(text=f"相机状态: ✗ {str(e)}", fg="red")
            messagebox.showerror("错误", f"相机初始化失败: {str(e)}")

    def get_chinese_font(self, size, weight="normal"):
        """获取中文字体"""
        if not hasattr(self, "_font_cache"):
            self._font_cache = {}

        cache_key = (size, weight)
        if cache_key in self._font_cache:
            return self._font_cache[cache_key]

        if not hasattr(self, "_font_family"):
            available = set(tkfont.families(self.root))
            preferred = [
                "Noto Sans CJK SC",
                "WenQuanYi Micro Hei",
                "Droid Sans Fallback",
                "fangsong ti",
                "song ti",
                "gothic",
                "mincho"
            ]
            self._font_family = next((f for f in preferred if f in available), None)

        if self._font_family:
            font_obj = tkfont.Font(
                root=self.root,
                family=self._font_family,
                size=size,
                weight=weight
            )
            actual = font_obj.actual()
            if actual.get("family") != self._font_family:
                font_obj = tkfont.Font(
                    root=self.root,
                    family=self._font_family,
                    size=size,
                    weight="normal"
                )
        else:
            font_obj = tkfont.Font(root=self.root, size=size, weight=weight)

        self._font_cache[cache_key] = font_obj
        return font_obj

    def create_gui(self):
        """创建集成GUI界面"""
        self.root = tk.Tk()
        self.root.title("三相机视频录制系统")
        self.root.geometry("1920x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.configure(bg="#2c3e50")

        # 中文字体
        font_title = self.get_chinese_font(18, "bold")
        font_large = self.get_chinese_font(14, "bold")
        font_medium = self.get_chinese_font(12)
        font_small = self.get_chinese_font(10)
        font_tiny = self.get_chinese_font(9)

        # ====================
        # 顶部标题区域
        # ====================
        top_frame = tk.Frame(self.root, bg="#34495e", height=80)
        top_frame.pack(fill="x", padx=0, pady=0)

        title_label = tk.Label(
            top_frame,
            text="三相机同步视频录制系统",
            font=font_title,
            fg="white",
            bg="#34495e"
        )
        title_label.pack(pady=20)

        # ====================
        # 中间内容区域
        # ====================
        content_frame = tk.Frame(self.root, bg="#2c3e50")
        content_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # 左侧：预览区域
        left_frame = tk.Frame(content_frame, bg="#34495e", relief="raised", borderwidth=2)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        preview_title = tk.Label(
            left_frame,
            text="实时预览（三相机拼接）",
            font=font_large,
            fg="white",
            bg="#34495e"
        )
        preview_title.pack(pady=10)

        # 预览画布
        self.preview_label = tk.Label(left_frame, bg="black")
        self.preview_label.pack(padx=10, pady=10)

        # 右侧：控制区域
        right_frame = tk.Frame(content_frame, bg="#34495e", relief="raised", borderwidth=2, width=350)
        right_frame.pack(side="right", fill="y", padx=(5, 0))
        right_frame.pack_propagate(False)

        control_title = tk.Label(
            right_frame,
            text="控制面板",
            font=font_large,
            fg="white",
            bg="#34495e"
        )
        control_title.pack(pady=15)

        # 状态信息框
        status_frame = tk.LabelFrame(
            right_frame,
            text="系统状态",
            font=font_medium,
            fg="white",
            bg="#34495e",
            padx=15,
            pady=10
        )
        status_frame.pack(padx=15, pady=10, fill="x")

        # 相机状态
        self.camera_status_label = tk.Label(
            status_frame,
            text="相机状态: 初始化中...",
            font=font_small,
            fg="#f39c12",
            bg="#34495e",
            anchor="w"
        )
        self.camera_status_label.pack(fill="x", pady=3)

        # 录制状态
        self.status_label = tk.Label(
            status_frame,
            text="录制状态: 等待录制",
            font=font_small,
            fg="#27ae60",
            bg="#34495e",
            anchor="w"
        )
        self.status_label.pack(fill="x", pady=3)

        # 录制时长
        self.time_label = tk.Label(
            status_frame,
            text="录制时长: 00:00:00",
            font=font_medium,
            fg="white",
            bg="#34495e",
            anchor="w"
        )
        self.time_label.pack(fill="x", pady=3)

        # 保存路径
        path_label = tk.Label(
            status_frame,
            text="保存路径:\n/home/ubuntu/take_data/\ndata/video/record_XXX/",
            font=font_tiny,
            fg="#95a5a6",
            bg="#34495e",
            anchor="w",
            justify="left"
        )
        path_label.pack(fill="x", pady=3)
        self.path_label = path_label

        # 控制按钮区域
        button_frame = tk.Frame(right_frame, bg="#34495e")
        button_frame.pack(pady=20, padx=15, fill="x")

        # 开始录制按钮
        self.start_button = tk.Button(
            button_frame,
            text="开始录制",
            font=font_large,
            bg="#27ae60",
            fg="white",
            width=18,
            height=2,
            command=self.start_recording,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.start_button.pack(pady=8)

        # 停止录制按钮
        self.stop_button = tk.Button(
            button_frame,
            text="停止录制",
            font=font_large,
            bg="#e74c3c",
            fg="white",
            width=18,
            height=2,
            command=self.stop_recording,
            state=tk.DISABLED,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.stop_button.pack(pady=8)

        # 退出按钮
        exit_button = tk.Button(
            button_frame,
            text="退出程序",
            font=font_medium,
            bg="#95a5a6",
            fg="white",
            width=18,
            command=self.on_closing,
            cursor="hand2"
        )
        exit_button.pack(pady=8)

        # 底部提示
        tip_label = tk.Label(
            right_frame,
            text="提示：\n点击'开始录制'后\n三个相机同步录制\n视频自动保存",
            font=font_tiny,
            fg="#bdc3c7",
            bg="#34495e",
            justify="center"
        )
        tip_label.pack(side="bottom", pady=15)

    def start_preview(self):
        """开始预览"""
        self.preview_thread = threading.Thread(target=self.update_preview, daemon=True)
        self.preview_thread.start()

    def update_preview(self):
        """更新预览画面"""
        last_update_time = 0
        update_interval = 1.0 / 30  # 30 FPS

        while self.running:
            current_time = time.time()

            # 控制帧率，减少闪烁
            if current_time - last_update_time < update_interval:
                time.sleep(0.001)
                continue

            images = []
            valid = True

            try:
                # 从所有相机获取帧
                for pipeline in self.pipelines:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        valid = False
                        break
                    color_image = np.asanyarray(color_frame.get_data())
                    images.append(color_image)

                if valid and len(images) == 3:
                    # 水平拼接三张图像
                    combined = cv2.hconcat(images)

                    # 如果正在录制，添加录制标识
                    if self.is_recording:
                        cv2.putText(combined, "REC", (10, 40),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                        cv2.circle(combined, (100, 30), 12, (0, 0, 255), -1)

                    # 添加相机标签
                    cv2.putText(combined, "Camera 1", (10, 470),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(combined, "Camera 2", (650, 470),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(combined, "Camera 3", (1290, 470),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    # 转换为tkinter可显示的格式
                    combined_rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)

                    # 缩放以适应窗口
                    h, w = combined_rgb.shape[:2]
                    scale = min(1500 / w, 500 / h)
                    new_w, new_h = int(w * scale), int(h * scale)
                    combined_resized = cv2.resize(combined_rgb, (new_w, new_h))

                    # 使用线程锁保护GUI更新
                    with self.frame_lock:
                        img = Image.fromarray(combined_resized)
                        imgtk = ImageTk.PhotoImage(image=img)

                        # 更新GUI（在主线程中）
                        self.root.after(0, self.update_preview_label, imgtk)

                    # 如果正在录制，写入视频
                    if self.is_recording and self.writers:
                        for writer, img in zip(self.writers, images):
                            writer.write(img)

                    last_update_time = current_time

            except Exception as e:
                print(f"预览错误: {str(e)}")
                time.sleep(0.1)

    def update_preview_label(self, imgtk):
        """在主线程中更新预览标签"""
        try:
            self.preview_label.imgtk = imgtk
            self.preview_label.configure(image=imgtk)
        except:
            pass

    def start_recording(self):
        """开始录制"""
        if self.is_recording:
            return

        try:
            # 创建本次录制的文件夹
            self.session_index = self.get_next_record_index()
            self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = os.path.join(
                self.video_base_path,
                f"record_{self.session_index:03d}"
            )
            os.makedirs(self.session_dir, exist_ok=True)

            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')

            for i in range(len(serial_numbers)):
                filename = os.path.join(
                    self.session_dir,
                    f"camera_{i+1}_{self.session_timestamp}.mp4"
                )
                writer = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))
                self.writers.append(writer)
                print(f"开始录制: {filename}")

            self.is_recording = True
            self.start_time = time.time()

            # 更新GUI
            self.status_label.config(text="录制状态: 正在录制", fg="#e74c3c")
            self.start_button.config(state=tk.DISABLED, bg="#95a5a6")
            self.stop_button.config(state=tk.NORMAL, bg="#e74c3c")
            self.path_label.config(
                text=f"保存路径:\n{self.session_dir}/"
            )

            # 开始更新录制时长
            self.update_recording_time()

            messagebox.showinfo("提示", "开始录制视频！\n三个相机同步录制中...")

        except Exception as e:
            messagebox.showerror("错误", f"开始录制失败: {str(e)}")

    def stop_recording(self):
        """停止录制"""
        if not self.is_recording:
            return

        self.is_recording = False

        # 释放视频写入器
        for writer in self.writers:
            writer.release()
        self.writers = []

        # 计算录制时长
        duration = time.time() - self.start_time
        duration_str = time.strftime("%H:%M:%S", time.gmtime(duration))

        # 更新GUI
        self.status_label.config(text="录制状态: 已停止录制", fg="#27ae60")
        self.start_button.config(state=tk.NORMAL, bg="#27ae60")
        self.stop_button.config(state=tk.DISABLED, bg="#95a5a6")
        self.time_label.config(text="录制时长: 00:00:00")

        messagebox.showinfo(
            "提示",
            f"录制完成！\n\n时长: {duration_str}\n\n视频已保存到:\n{self.session_dir}/"
        )
        print(f"录制完成，时长: {duration_str}")

    def get_next_record_index(self):
        """获取下一个录制序号"""
        max_index = 0
        for name in os.listdir(self.video_base_path):
            if name.startswith("record_"):
                suffix = name.split("record_", 1)[1]
                if suffix.isdigit():
                    max_index = max(max_index, int(suffix))
        return max_index + 1

    def migrate_legacy_folders(self):
        """将旧的 camera_1/2/3 结构迁移到按录制次数的文件夹"""
        legacy_dirs = [f"{self.video_base_path}/camera_{i}" for i in range(1, 4)]
        legacy_dirs = [d for d in legacy_dirs if os.path.isdir(d)]
        if not legacy_dirs:
            return

        recordings = {}
        for idx in range(1, 4):
            cam_dir = f"{self.video_base_path}/camera_{idx}"
            if not os.path.isdir(cam_dir):
                continue
            for name in os.listdir(cam_dir):
                if not (name.startswith("video_") and name.endswith(".mp4")):
                    continue
                timestamp = name[len("video_"):-len(".mp4")]
                recordings.setdefault(timestamp, []).append(
                    (idx, os.path.join(cam_dir, name))
                )

        if not recordings:
            return

        next_index = self.get_next_record_index()
        for timestamp in sorted(recordings.keys()):
            record_dir = os.path.join(
                self.video_base_path,
                f"record_{next_index:03d}"
            )
            os.makedirs(record_dir, exist_ok=True)
            for idx, src in recordings[timestamp]:
                dst = os.path.join(record_dir, f"camera_{idx}_{timestamp}.mp4")
                shutil.move(src, dst)
            next_index += 1

        for cam_dir in legacy_dirs:
            if not os.listdir(cam_dir):
                os.rmdir(cam_dir)

    def update_recording_time(self):
        """更新录制时长显示"""
        if self.is_recording:
            elapsed = time.time() - self.start_time
            time_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))
            self.time_label.config(text=f"录制时长: {time_str}")
            self.root.after(100, self.update_recording_time)

    def on_closing(self):
        """关闭程序"""
        if self.is_recording:
            if messagebox.askokcancel("警告", "正在录制中，确定要退出吗？\n录制将被停止并保存。"):
                self.stop_recording()
            else:
                return

        self.running = False
        time.sleep(0.1)

        # 停止相机
        for pipeline in self.pipelines:
            pipeline.stop()

        self.root.quit()
        self.root.destroy()

    def run(self):
        """运行GUI"""
        self.root.mainloop()

if __name__ == "__main__":
    recorder = VideoRecorder()
    recorder.run()
