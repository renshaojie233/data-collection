#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三相机视频录制系统 - 图形化启动器
双击此文件即可启动视频录制系统
"""

import tkinter as tk
from tkinter import messagebox, font as tkfont
import subprocess
import os
import sys

class Launcher:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("三相机视频录制系统启动器")
        self.root.geometry("500x400")
        self.root.resizable(False, False)

        # 设置中文字体
        font_title = self.get_chinese_font(20, "bold")
        font_normal = self.get_chinese_font(12)
        font_small = self.get_chinese_font(10)

        # 标题
        title_label = tk.Label(
            self.root,
            text="三相机视频录制系统",
            font=font_title,
            fg="#2c3e50"
        )
        title_label.pack(pady=30)

        # 状态框架
        status_frame = tk.LabelFrame(
            self.root,
            text="系统状态",
            font=font_normal,
            padx=20,
            pady=15
        )
        status_frame.pack(padx=20, pady=10, fill="both")

        # 环境状态
        env_label = tk.Label(
            status_frame,
            text="Conda环境: take_data",
            font=font_small,
            anchor="w"
        )
        env_label.pack(fill="x", pady=3)

        # 视频保存路径
        path_label = tk.Label(
            status_frame,
            text="保存路径: /home/ubuntu/take_data/data/video/",
            font=font_small,
            anchor="w"
        )
        path_label.pack(fill="x", pady=3)

        # 相机数量
        camera_label = tk.Label(
            status_frame,
            text="相机数量: 3个RealSense D435i",
            font=font_small,
            anchor="w"
        )
        camera_label.pack(fill="x", pady=3)

        # 按钮框架
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=30)

        # 启动按钮
        start_button = tk.Button(
            button_frame,
            text="启动录制程序",
            font=self.get_chinese_font(14, "bold"),
            bg="#27ae60",
            fg="white",
            width=15,
            height=2,
            command=self.start_program,
            cursor="hand2"
        )
        start_button.pack(pady=5)

        # 测试按钮
        test_button = tk.Button(
            button_frame,
            text="测试相机连接",
            font=font_normal,
            bg="#3498db",
            fg="white",
            width=15,
            command=self.test_cameras,
            cursor="hand2"
        )
        test_button.pack(pady=5)

        # 退出按钮
        exit_button = tk.Button(
            button_frame,
            text="退出",
            font=font_normal,
            bg="#95a5a6",
            fg="white",
            width=15,
            command=self.root.quit,
            cursor="hand2"
        )
        exit_button.pack(pady=5)

        # 底部说明
        info_label = tk.Label(
            self.root,
            text="提示：启动程序后会打开GUI界面和预览窗口",
            font=self.get_chinese_font(9),
            fg="#7f8c8d"
        )
        info_label.pack(side="bottom", pady=10)

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

    def start_program(self):
        """启动录制程序"""
        try:
            # 切换到程序目录
            os.chdir("/home/ubuntu/take_data/take_video")

            # 使用gnome-terminal启动
            cmd = [
                "gnome-terminal",
                "--",
                "bash", "-c",
                "source /home/ubuntu/anaconda3/bin/activate take_data && "
                "cd /home/ubuntu/take_data/take_video && "
                "python take_video.py; "
                "echo ''; echo '程序已退出'; read -p '按回车键关闭窗口...'"
            ]

            subprocess.Popen(cmd)

            messagebox.showinfo(
                "提示",
                "视频录制程序已启动！\n\n"
                "请在新打开的终端窗口中查看程序运行状态。\n"
                "GUI界面和预览窗口将自动打开。"
            )

            # 关闭启动器
            self.root.quit()

        except Exception as e:
            messagebox.showerror("错误", f"启动失败：{str(e)}")

    def test_cameras(self):
        """测试相机连接"""
        try:
            # 使用gnome-terminal启动测试
            cmd = [
                "gnome-terminal",
                "--",
                "bash", "-c",
                "source /home/ubuntu/anaconda3/bin/activate take_data && "
                "cd /home/ubuntu/take_data/take_video && "
                "python test_cameras.py; "
                "read -p '按回车键关闭窗口...'"
            ]

            subprocess.Popen(cmd)

            messagebox.showinfo(
                "提示",
                "相机测试程序已启动！\n\n"
                "请在新打开的终端窗口中查看测试结果。"
            )

        except Exception as e:
            messagebox.showerror("错误", f"测试失败：{str(e)}")

    def run(self):
        """运行启动器"""
        self.root.mainloop()

if __name__ == "__main__":
    launcher = Launcher()
    launcher.run()
