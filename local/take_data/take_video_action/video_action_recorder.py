#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视频+动作数据同步录制系统
Integrated Video and Robot Action Recording System
"""

import cv2
import pyrealsense2 as rs
import time
import numpy as np
import os
import shutil
import socket
import json
import h5py
import copy
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, font as tkfont
import threading
from PIL import Image, ImageTk
import subprocess
import signal
import sys
import atexit
import shlex
import re
from collections import deque

# 相机序列号
CAMERA_SERIAL_NUMBERS = [
    '243722070232',  # 相机1
    '243622073040',  # 相机2
    '243722073691'   # 相机3
]

# 远程机器人配置
REMOTE_HOST = "172.16.1.2"
REMOTE_USER = "rsj"
REMOTE_PASSWORD = os.environ.get("REMOTE_PASSWORD", "")
REMOTE_PORT = 9999
REMOTE_ABSOLUTE_SCRIPT = "/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh"
REMOTE_RELATIVE_SCRIPT = "/home/rsj/gello_software/start_relative_gello_control.sh"
REMOTE_RELATIVE_CONFIG = "/home/rsj/gello_software/configs/relative_gello_control.yaml"
REMOTE_BRIDGE_DIR = "/tmp/robot_data_bridge"
# 远程回放配置
REMOTE_STOP_SCRIPT = "/home/rsj/gello_software/kill_ros2_stale.sh"
REMOTE_REPLAY_SCRIPT = "/home/rsj/franka_cpp_control/run_track_json_impedance_gripper_robotiq.sh"
REMOTE_REPLAY_DIR = "/home/rsj/franka_cpp_control/replay_data"
REMOTE_REPLAY_ACTION_DIR = f"{REMOTE_REPLAY_DIR}/action"
REMOTE_REPLAY_VIDEO_DIR = f"{REMOTE_REPLAY_DIR}/video"
REMOTE_ROBOT_IP = "172.16.0.2"
GRIPPER_MAX_WIDTH = 0.085
FR3_JOINT_ORDER = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]
JOINT_NUMBER_RE = re.compile(r"joint([1-7])$", re.IGNORECASE)


class VideoActionRecorder:
    def __init__(self):
        # 基础路径
        self.base_path = "/home/ubuntu/take_data/data"
        self.record_base_path = self.base_path
        self.legacy_video_base_path = os.path.join(self.base_path, "video")
        self.legacy_action_base_path = os.path.join(self.base_path, "action")
        os.makedirs(self.record_base_path, exist_ok=True)
        self.current_video_path = self.record_base_path
        self.current_action_path = self.record_base_path

        # 录制状态
        self.is_recording = False
        self.running = True
        self.session_dir = None
        self.session_index = None
        self.record_start_time = None
        self.record_lock = threading.Lock()
        self.replay_in_progress = False

        # 视频相关
        self.pipelines = []
        self.video_writers = []
        self.frame_lock = threading.Lock()
        self.preview_fps = 30.0
        self.record_fps = 15.0
        self.frame_dirs = []
        self.video_output_paths = []
        self.frame_index = 0

        # 动作数据相关
        self.action_socket = None
        self.action_data_buffer = []
        self.action_thread = None
        self.robot_ready = False
        self.latest_action_data = None
        self.action_sample_lock = threading.Lock()
        self.action_sample_buffer = deque()
        self.action_sample_window_sec = 5.0
        self.action_time_offsets = deque(maxlen=200)
        self.action_time_offset = None
        self.control_mode_var = None
        self.active_control_mode = None
        self.relative_zero_timeout_sec = 10.0
        self.relative_zero_tolerance = 0.05
        self._action_redraw_pending = False
        self.action_redraw_interval_ms = 80
        self.action_history = deque()
        self.action_history_lock = threading.Lock()
        self._joint_order_cache = {}
        self.history_window_sec = 10.0
        self.preview_size = (0, 0)
        self.ui_state_path = os.path.join(
            os.path.expanduser("~"),
            ".config",
            "video_action_recorder",
            "ui_state.json"
        )
        self.ui_state = self.load_ui_state()
        self._splitter_restored = False

        # 创建GUI
        self.create_gui()

        # 初始化相机
        self.init_cameras()

        # 设置清理钩子
        self.setup_cleanup_hooks()

        # 开始预览
        self.start_preview()

        # 启动远程数据流
        self.start_action_stream()

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
                "思源黑体",
                "Source Han Sans SC",
                "Source Han Sans CN",
                "Source Han Sans",
                "Noto Sans CJK SC",
                "Noto Sans CJK",
                "WenQuanYi Zen Hei",
                "WenQuanYi Micro Hei",
                "Droid Sans Fallback",
                "Microsoft YaHei",
                "PingFang SC",
                "SimHei"
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

    def setup_cleanup_hooks(self):
        """设置清理钩子，确保程序退出时清理远程进程"""
        atexit.register(self.cleanup_remote_processes)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """信号处理器"""
        print("\n接收到退出信号，正在清理...")
        self.cleanup_and_exit()

    def load_ui_state(self):
        """加载界面布局状态"""
        try:
            if os.path.exists(self.ui_state_path):
                with open(self.ui_state_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    return data
        except Exception:
            pass
        return {}

    def _set_label_text(self, label, text, fg=None):
        """线程安全更新标签文字"""
        def _update():
            if fg is None:
                label.config(text=text)
            else:
                label.config(text=text, fg=fg)

        try:
            self.root.after(0, _update)
        except Exception:
            pass

    def _on_content_resize(self, event):
        """保持上下区域50/50高度"""
        total = event.height
        if total <= 0:
            return
        half = max(int(total / 2), 1)
        event.widget.grid_rowconfigure(0, minsize=half)
        event.widget.grid_rowconfigure(1, minsize=half)

    def _build_action_canvases(self):
        """创建动作曲线画布"""
        self.action_canvases = []
        self.action_canvas_specs = [
            {"key": "pos_x", "title": "位置 X (m)", "color": "#e74c3c", "ymin": -1.0, "ymax": 1.0, "min_span": 0.02},
            {"key": "pos_y", "title": "位置 Y (m)", "color": "#2ecc71", "ymin": -1.0, "ymax": 1.0, "min_span": 0.02},
            {"key": "pos_z", "title": "位置 Z (m)", "color": "#3498db", "ymin": -1.0, "ymax": 1.0, "min_span": 0.02},
            {"key": "grip", "title": "夹爪开合 (m)", "color": "#ecf0f1", "ymin": 0.0, "ymax": 0.085, "min_span": 0.01},
            {"key": "ori_x", "title": "姿态 Qx", "color": "#9b59b6", "ymin": -1.0, "ymax": 1.0, "min_span": 0.1},
            {"key": "ori_y", "title": "姿态 Qy", "color": "#1abc9c", "ymin": -1.0, "ymax": 1.0, "min_span": 0.1},
            {"key": "ori_z", "title": "姿态 Qz", "color": "#f1c40f", "ymin": -1.0, "ymax": 1.0, "min_span": 0.1},
            {"key": "ori_w", "title": "姿态 Qw", "color": "#e67e22", "ymin": -1.0, "ymax": 1.0, "min_span": 0.1},
        ]

        cols = 2
        rows = (len(self.action_canvas_specs) + cols - 1) // cols
        for i in range(len(self.action_canvas_specs)):
            canvas = tk.Canvas(self.action_canvas_frame, bg="#1f2d3a", highlightthickness=0)
            row = i % rows
            col = i // rows
            canvas.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
            self.action_canvas_frame.grid_rowconfigure(row, weight=1)
            self.action_canvases.append(canvas)
        for col in range(cols):
            self.action_canvas_frame.grid_columnconfigure(col, weight=1)

    def _compute_series_range(self, points, default_min, default_max, min_span):
        """根据数据自动计算显示范围"""
        min_v = None
        max_v = None
        for _, v in points:
            try:
                v = float(v)
            except Exception:
                continue
            if min_v is None or v < min_v:
                min_v = v
            if max_v is None or v > max_v:
                max_v = v
        if min_v is None or max_v is None:
            return default_min, default_max

        span = max_v - min_v
        if span < min_span:
            center = (min_v + max_v) / 2
            span = min_span
            min_v = center - span / 2
            max_v = center + span / 2
        else:
            pad = span * 0.1
            min_v -= pad
            max_v += pad
        return min_v, max_v

    def save_ui_state(self):
        """保存界面布局状态"""
        try:
            os.makedirs(os.path.dirname(self.ui_state_path), exist_ok=True)
            with open(self.ui_state_path, "w", encoding="utf-8") as f:
                json.dump(self.ui_state, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _format_path_label(self, prefix, path):
        safe_path = (path or "").rstrip("/")
        if safe_path:
            return f"{prefix}: {safe_path}/"
        return f"{prefix}: --"

    def set_current_paths(self, video_path=None, action_path=None):
        """更新当前路径并刷新标签"""
        if video_path:
            self.current_video_path = video_path
            if hasattr(self, "video_path_label"):
                self.video_path_label.config(
                    text=self._format_path_label("视频", video_path)
                )
        if action_path:
            self.current_action_path = action_path
            if hasattr(self, "action_path_label"):
                self.action_path_label.config(
                    text=self._format_path_label("动作", action_path)
                )

    def open_path_in_file_manager(self, path):
        """打开保存路径目录"""
        if not path:
            messagebox.showinfo("提示", "暂无可打开的路径")
            return
        abs_path = os.path.abspath(path)
        if not os.path.exists(abs_path):
            messagebox.showwarning("提示", f"路径不存在: {abs_path}")
            return
        try:
            subprocess.Popen(["xdg-open", abs_path])
        except Exception as e:
            messagebox.showerror("错误", f"无法打开路径: {str(e)}")

    def _get_split_ratio(self):
        """获取分割条当前位置占比"""
        try:
            total = self.main_paned.winfo_height()
            if total <= 0:
                return None
            _, sash_y = self.main_paned.sash_coord(0)
            ratio = sash_y / total
            return max(0.2, min(0.85, ratio))
        except Exception:
            return None

    def _apply_split_ratio(self, ratio):
        """按占比设置分割条位置"""
        try:
            total = self.main_paned.winfo_height()
            if total <= 0:
                return
            sash_y = int(total * ratio)
            self.main_paned.sash_place(0, 0, sash_y)
        except Exception:
            pass

    def _restore_splitter(self):
        """恢复分割条位置"""
        ratio = self.ui_state.get("split_ratio", 0.75)
        self._apply_split_ratio(ratio)

    def _on_splitter_release(self, event):
        """拖动分割条后自动保存"""
        ratio = self._get_split_ratio()
        if ratio is not None:
            self.ui_state["split_ratio"] = ratio
            self.save_ui_state()

    def _on_paned_configure(self, event):
        """首次布局完成后恢复分割条位置"""
        if not self._splitter_restored:
            self._splitter_restored = True
            self._restore_splitter()

    def start_action_stream(self):
        """启动远程系统并持续接收动作数据（后台运行）"""
        def background_start():
            if self.robot_ready and self.action_socket:
                return
            if not self.start_remote_robot(control_mode=self._get_selected_control_mode()):
                return
            if not self.action_thread or not self.action_thread.is_alive():
                self.action_thread = threading.Thread(target=self.receive_action_data, daemon=True)
                self.action_thread.start()

        # 在后台线程启动，不阻塞GUI
        startup_thread = threading.Thread(target=background_start, daemon=True)
        startup_thread.start()

    def _validate_replay_index(self, value):
        if value == "":
            return True
        return value.isdigit()

    def _on_replay_index_change(self):
        try:
            value = int(self.replay_index_var.get())
        except (tk.TclError, ValueError):
            return
        if value < 1:
            self.replay_index_var.set(1)

    def _update_replay_controls(self):
        last_index = self.get_last_record_index()
        if last_index:
            self.replay_spinbox.config(to=last_index)
            if self.replay_index_var.get() > last_index:
                self.replay_index_var.set(last_index)
        else:
            self.replay_spinbox.config(to=1)
            self.replay_index_var.set(1)

        if self.replay_in_progress or self.is_recording or not last_index:
            self.replay_button.config(state=tk.DISABLED)
        else:
            self.replay_button.config(state=tk.NORMAL)

        spinbox_state = tk.DISABLED if self.replay_in_progress else tk.NORMAL
        self.replay_spinbox.config(state=spinbox_state)

    def _get_selected_control_mode(self):
        if self.control_mode_var is None:
            return "relative"
        value = self.control_mode_var.get()
        return value if value in ("absolute", "relative") else "relative"

    def _set_mode_controls_state(self, state):
        if hasattr(self, "mode_abs_radio"):
            self.mode_abs_radio.config(state=state)
        if hasattr(self, "mode_rel_radio"):
            self.mode_rel_radio.config(state=state)

    def _on_control_mode_change(self):
        new_mode = self._get_selected_control_mode()
        if self.is_recording or self.replay_in_progress:
            messagebox.showwarning("提示", "录制或回放中无法切换控制模式")
            if self.active_control_mode:
                self.control_mode_var.set(self.active_control_mode)
            return
        if self.active_control_mode == new_mode:
            return
        self._set_mode_controls_state(tk.DISABLED)
        switch_thread = threading.Thread(
            target=self._switch_control_mode_worker,
            args=(new_mode,),
            daemon=True,
        )
        switch_thread.start()

    def _switch_control_mode_worker(self, new_mode):
        try:
            self._set_label_text(self.robot_ready_label, "机器人状态: 切换控制模式...", "#f39c12")
            if self.robot_ready or self.action_socket:
                try:
                    self._pause_remote_control()
                except Exception as e:
                    self._set_label_text(self.robot_ready_label, f"机器人状态: {e}", "#e74c3c")
            if not self.start_remote_robot(control_mode=new_mode):
                raise RuntimeError("切换控制模式失败")
            if new_mode == "relative":
                self._set_relative_control_enabled(False)
            self.active_control_mode = new_mode
        except Exception as e:
            def _revert():
                if self.active_control_mode:
                    self.control_mode_var.set(self.active_control_mode)
                messagebox.showerror("错误", f"切换控制模式失败: {e}")
            self.root.after(0, _revert)
        finally:
            self.root.after(0, lambda: self._set_mode_controls_state(tk.NORMAL))

    def _get_replay_index(self):
        try:
            value = int(self.replay_index_var.get())
        except (tk.TclError, ValueError):
            return None
        if value < 1:
            return None
        return value

    def _find_action_json(self, action_dir):
        candidates = []
        for name in os.listdir(action_dir):
            if name.startswith("action_data_") and name.endswith(".json"):
                candidates.append(os.path.join(action_dir, name))
        if not candidates:
            return None
        return max(candidates, key=os.path.getmtime)

    def _get_replay_sources(self, session_index):
        record_name = f"record_{session_index:03d}"
        record_dir = os.path.join(self.record_base_path, record_name)
        action_dir = os.path.join(record_dir, "action")
        video_dir = os.path.join(record_dir, "video")
        if not os.path.isdir(action_dir):
            action_dir = os.path.join(self.legacy_action_base_path, record_name)
        if not os.path.isdir(video_dir):
            video_dir = os.path.join(self.legacy_video_base_path, record_name)
        if not os.path.isdir(action_dir):
            raise FileNotFoundError(f"未找到动作数据目录: {record_name}")
        if not os.path.isdir(video_dir):
            raise FileNotFoundError(f"未找到视频数据目录: {record_name}")

        json_path = self._find_action_json(action_dir)
        if not json_path:
            raise FileNotFoundError(f"{record_name} 中未找到动作JSON文件")

        with open(json_path, "r", encoding="utf-8") as f:
            payload = json.load(f)

        frames = payload.get("data") or []
        if not frames:
            raise ValueError(f"{record_name} 动作数据为空")
        return json_path, action_dir, video_dir

    def _ensure_remote_replay_script(self):
        """确保远程回放脚本与目录存在"""
        # 检查远程回放脚本是否存在
        result = subprocess.run(
            self._ssh_base_cmd()
            + [
                f"{REMOTE_USER}@{REMOTE_HOST}",
                f"test -f {REMOTE_REPLAY_SCRIPT} && echo 'exists' || echo 'missing'",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if "missing" in result.stdout:
            raise FileNotFoundError(
                f"远程回放脚本缺失: {REMOTE_REPLAY_SCRIPT}\n"
                "请确认 franka_cpp_control 已部署"
            )
        subprocess.run(
            self._ssh_base_cmd()
            + [
                f"{REMOTE_USER}@{REMOTE_HOST}",
                f"mkdir -p {REMOTE_REPLAY_DIR}",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )

    def _prepare_replay_json(self, json_path):
        """补齐回放所需的夹爪关节字段"""
        with open(json_path, "r", encoding="utf-8") as f:
            payload = json.load(f)

        frames = payload.get("data") or []
        if not frames:
            return json_path

        needs_rewrite = False
        for frame in frames:
            gripper_data = frame.get("gripper_joints") or {}
            names = gripper_data.get("names") or []
            positions = gripper_data.get("position") or []

            has_canonical = (
                isinstance(names, list)
                and "fr3_finger_joint1" in names
                and "fr3_finger_joint2" in names
                and isinstance(positions, list)
                and len(positions) >= 2
            )
            if has_canonical:
                continue

            width = None
            if frame.get("gripper_command") is not None:
                try:
                    width = float(frame["gripper_command"]) * GRIPPER_MAX_WIDTH
                except Exception:
                    width = None
            if width is None:
                width = self._extract_gripper_width(names, positions)
            if width is None:
                width = 0.0

            half = float(width) / 2.0
            frame["gripper_joints"] = {
                "names": ["fr3_finger_joint1", "fr3_finger_joint2"],
                "position": [half, half],
                "velocity": [0.0, 0.0],
                "effort": [0.0, 0.0],
            }
            needs_rewrite = True

        if not needs_rewrite:
            return json_path

        payload["packet_count"] = len(frames)
        base, _ = os.path.splitext(json_path)
        replay_path = f"{base}_replay.json"
        with open(replay_path, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        return replay_path
    def _run_remote_replay(self, remote_json_path):
        """使用远端阻抗回放脚本执行轨迹"""
        cmd = (
            f"bash {shlex.quote(REMOTE_REPLAY_SCRIPT)} "
            f"{shlex.quote(REMOTE_ROBOT_IP)} "
            f"{shlex.quote(remote_json_path)}"
        )
        result = subprocess.run(
            self._ssh_base_cmd() + [f"{REMOTE_USER}@{REMOTE_HOST}", cmd],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            error_msg = result.stderr.strip() or result.stdout.strip() or "回放失败"
            raise RuntimeError(f"远程回放失败:\n{error_msg}")

    def _remote_process_exists(self, pattern):
        cmd = f"pgrep -f {shlex.quote(pattern)} >/dev/null 2>&1"
        result = subprocess.run(
            self._ssh_base_cmd() + [f"{REMOTE_USER}@{REMOTE_HOST}", cmd],
            capture_output=True,
            text=True,
            timeout=10,
        )
        return result.returncode == 0

    def _remote_any_process_exists(self, patterns):
        if not patterns:
            return False
        escaped = [re.escape(p) for p in patterns]
        regex = "|".join(escaped)
        return self._remote_process_exists(regex)

    def _wait_for_remote_process(self, patterns, timeout, status_prefix):
        start = time.monotonic()
        if isinstance(patterns, str):
            patterns = [patterns]
        while time.monotonic() - start < timeout:
            if self._remote_any_process_exists(patterns):
                return True
            elapsed = int(time.monotonic() - start)
            self._set_label_text(
                self.robot_ready_label,
                f"{status_prefix} ({elapsed}s)",
                "#f39c12",
            )
            time.sleep(1)
        return False

    def _pause_remote_control(self):
        """暂停远端控制，释放机器人控制权"""
        self._set_label_text(self.robot_ready_label, "机器人状态: 暂停控制...", "#f39c12")
        self._set_label_text(self.robot_connection_label, "机器人连接: 断开中...", "#f39c12")
        if self.action_socket:
            try:
                self.action_socket.close()
            except Exception:
                pass
            self.action_socket = None
        self.robot_ready = False

        cmd = (
            f"test -f {shlex.quote(REMOTE_STOP_SCRIPT)} || "
            f"(echo 'missing' && exit 1); "
            f"bash {shlex.quote(REMOTE_STOP_SCRIPT)}; "
            "pkill -f '[r]emote_data_bridge.py' 2>/dev/null || true; "
            "pkill -f '[s]tart_relative_gello_control.sh' 2>/dev/null || true; "
            "pkill -f '[g]ello_relative_publisher' 2>/dev/null || true"
        )
        result = subprocess.run(
            self._ssh_base_cmd() + [f"{REMOTE_USER}@{REMOTE_HOST}", cmd],
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode != 0:
            msg = result.stderr.strip() or result.stdout.strip() or "暂停远端控制失败"
            if "missing" in result.stdout:
                msg = f"远端停控脚本缺失: {REMOTE_STOP_SCRIPT}"
            raise RuntimeError(msg)
        self._cleanup_remote_for_start()
        self._set_label_text(self.robot_connection_label, "机器人连接: 已暂停", "#95a5a6")

    def start_replay(self):
        """开始回放"""
        if self.replay_in_progress:
            return
        if self.is_recording:
            messagebox.showwarning("提示", "录制中无法回放")
            return
        if not self.robot_ready or not self.action_socket:
            messagebox.showwarning("提示", "机器人未就绪，请稍后再试")
            return

        session_index = self._get_replay_index()
        if not session_index:
            messagebox.showwarning("提示", "请输入有效的录制序号")
            return

        self.replay_in_progress = True
        self._set_label_text(self.status_label, "录制状态: 回放中", "#2980b9")
        self._set_label_text(self.robot_ready_label, "机器人状态: 回放中", "#2980b9")
        self._update_replay_controls()
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.delete_last_button.config(state=tk.DISABLED)

        replay_thread = threading.Thread(
            target=self._replay_worker,
            args=(session_index,),
            daemon=True,
        )
        replay_thread.start()

    def _replay_worker(self, session_index):
        error = None
        resume_ok = True
        try:
            self._pause_remote_control()
            local_json, action_dir, video_dir = self._get_replay_sources(session_index)
            replay_json = self._prepare_replay_json(local_json)
            self._ensure_remote_replay_script()
            subprocess.run(
                self._ssh_base_cmd()
                + [
                    f"{REMOTE_USER}@{REMOTE_HOST}",
                    f"rm -rf {REMOTE_REPLAY_ACTION_DIR} {REMOTE_REPLAY_VIDEO_DIR} && "
                    f"mkdir -p {REMOTE_REPLAY_DIR}",
                ],
                check=True,
                capture_output=True,
                timeout=10,
            )
            subprocess.run(
                self._scp_base_cmd()
                + [
                    "-r",
                    action_dir,
                    f"{REMOTE_USER}@{REMOTE_HOST}:{REMOTE_REPLAY_DIR}",
                ],
                check=True,
                capture_output=True,
                timeout=180,
            )
            subprocess.run(
                self._scp_base_cmd()
                + [
                    "-r",
                    video_dir,
                    f"{REMOTE_USER}@{REMOTE_HOST}:{REMOTE_REPLAY_DIR}",
                ],
                check=True,
                capture_output=True,
                timeout=600,
            )
            remote_json = f"{REMOTE_REPLAY_ACTION_DIR}/{os.path.basename(replay_json)}"
            self._run_remote_replay(remote_json)
        except Exception as exc:
            error = str(exc)
        finally:
            if self.running:
                self._set_label_text(self.robot_ready_label, "机器人状态: 恢复控制中...", "#f39c12")
                resume_ok = self.start_remote_robot(control_mode=self._get_selected_control_mode())
                if resume_ok and (not self.action_thread or not self.action_thread.is_alive()):
                    self.action_thread = threading.Thread(
                        target=self.receive_action_data,
                        daemon=True,
                    )
                    self.action_thread.start()
                if not resume_ok:
                    if error:
                        error = f"{error}\n远端系统恢复失败"
                    else:
                        error = "远端系统恢复失败"
            else:
                resume_ok = False

        def finish():
            self.replay_in_progress = False
            self._set_label_text(self.status_label, "录制状态: 等待录制", "#27ae60")
            if resume_ok:
                self._set_label_text(self.robot_ready_label, "机器人状态: 就绪", "#27ae60")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)
            self.delete_last_button.config(state=tk.NORMAL)
            self._update_replay_controls()
            if error:
                messagebox.showerror("回放失败", error)
            else:
                messagebox.showinfo("完成", "回放完成")

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title("视频+动作数据同步录制系统")
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        window_w = min(1920, max(1, screen_w - 40))
        window_h = max(1, screen_h - 40)
        self.root.geometry(f"{window_w}x{window_h}")
        self.root.configure(bg="#2c3e50")

        # 设置窗口关闭协议
        self.root.protocol("WM_DELETE_WINDOW", self.on_window_close)

        # 字体
        self.font_title = self.get_chinese_font(18, "bold")
        self.font_large = self.get_chinese_font(14, "bold")
        self.font_medium = self.get_chinese_font(12)
        self.font_small = self.get_chinese_font(10)
        self.font_tiny = self.get_chinese_font(9)
        self.font_panel_title = self.get_chinese_font(13, "bold")
        self.font_panel_section = self.get_chinese_font(11, "bold")
        self.font_panel_text = self.get_chinese_font(10)
        self.font_panel_small = self.get_chinese_font(9)

        # ====================
        # 中间内容区域
        # ====================
        content_frame = tk.Frame(self.root, bg="#2c3e50")
        content_frame.pack(fill="both", expand=True, padx=8, pady=8)
        content_frame.grid_rowconfigure(0, weight=1, uniform="main")
        content_frame.grid_rowconfigure(1, weight=1, uniform="main")
        content_frame.grid_columnconfigure(0, weight=1)
        content_frame.bind("<Configure>", self._on_content_resize)

        # 上方：预览区域
        video_frame = tk.Frame(content_frame, bg="#34495e", relief="raised", borderwidth=2)
        video_frame.grid(row=0, column=0, sticky="nsew")
        video_frame.grid_rowconfigure(0, weight=0)
        video_frame.grid_rowconfigure(1, weight=1)
        video_frame.grid_columnconfigure(0, weight=1)

        preview_title = tk.Label(
            video_frame,
            text="实时预览（三相机拼接）",
            font=self.font_large,
            fg="white",
            bg="#34495e"
        )
        preview_title.grid(row=0, column=0, pady=6)

        preview_container = tk.Frame(video_frame, bg="#34495e")
        preview_container.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        # 预览画布
        self.preview_label = tk.Label(preview_container, bg="#34495e", anchor="center")
        self.preview_label.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.preview_label.bind("<Configure>", self._on_preview_resize)

        # 下方：动作可视化 + 控制面板
        bottom_frame = tk.Frame(content_frame, bg="#2c3e50")
        bottom_frame.grid(row=1, column=0, sticky="nsew")
        bottom_frame.grid_rowconfigure(0, weight=1)
        bottom_frame.grid_columnconfigure(0, weight=1)
        bottom_frame.grid_columnconfigure(1, weight=0, minsize=420)

        action_frame = tk.Frame(bottom_frame, bg="#34495e", relief="raised", borderwidth=2)
        action_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        action_frame.grid_rowconfigure(0, weight=0)
        action_frame.grid_rowconfigure(1, weight=1)
        action_frame.grid_columnconfigure(0, weight=1)

        action_title = tk.Label(
            action_frame,
            text="动作可视化",
            font=self.font_large,
            fg="white",
            bg="#34495e"
        )
        action_title.grid(row=0, column=0, pady=6)

        self.action_canvas_frame = tk.Frame(action_frame, bg="#1f2d3a")
        self.action_canvas_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        self._build_action_canvases()
        self._schedule_action_redraw()

        # 右下：控制区域
        right_frame = tk.Frame(bottom_frame, bg="#34495e", relief="raised", borderwidth=2, width=420)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        right_frame.grid_propagate(False)
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_columnconfigure(0, weight=1)

        control_canvas = tk.Canvas(
            right_frame,
            bg="#34495e",
            highlightthickness=0,
            borderwidth=0
        )
        control_scrollbar = tk.Scrollbar(
            right_frame,
            orient="vertical",
            command=control_canvas.yview
        )
        control_canvas.configure(yscrollcommand=control_scrollbar.set)
        control_canvas.grid(row=0, column=0, sticky="nsew")
        control_scrollbar.grid(row=0, column=1, sticky="ns")

        control_content = tk.Frame(control_canvas, bg="#34495e")
        control_window_id = control_canvas.create_window(
            (0, 0),
            window=control_content,
            anchor="nw"
        )

        def _on_control_canvas_configure(event):
            control_canvas.configure(scrollregion=control_canvas.bbox("all"))

        def _on_control_canvas_width(event):
            control_canvas.itemconfigure(control_window_id, width=event.width)

        control_content.bind("<Configure>", _on_control_canvas_configure)
        control_canvas.bind("<Configure>", _on_control_canvas_width)

        control_title = tk.Label(
            control_content,
            text="控制面板",
            font=self.font_large,
            fg="white",
            bg="#34495e"
        )
        control_title.pack(pady=6)

        # ====================
        # 系统状态框
        # ====================
        status_frame = tk.LabelFrame(
            control_content,
            text="系统状态",
            font=self.font_panel_section,
            fg="white",
            bg="#34495e",
            padx=8,
            pady=6
        )
        status_frame.pack(padx=10, pady=5, fill="x")
        status_frame.grid_columnconfigure(0, weight=1)
        status_frame.grid_columnconfigure(1, weight=1)

        # 相机状态
        self.camera_status_label = tk.Label(
            status_frame,
            text="相机: 初始化中...",
            font=self.font_panel_text,
            fg="#f39c12",
            bg="#34495e",
            anchor="w"
        )
        self.camera_status_label.grid(row=0, column=0, sticky="w", padx=(0, 6), pady=2)

        # 机器人连接状态
        self.robot_connection_label = tk.Label(
            status_frame,
            text="机器人连接: 未连接",
            font=self.font_panel_text,
            fg="#95a5a6",
            bg="#34495e",
            anchor="w"
        )
        self.robot_connection_label.grid(row=0, column=1, sticky="w", padx=(6, 0), pady=2)

        # 机器人准备状态
        self.robot_ready_label = tk.Label(
            status_frame,
            text="机器人状态: 待机中",
            font=self.font_panel_text,
            fg="#95a5a6",
            bg="#34495e",
            anchor="w"
        )
        self.robot_ready_label.grid(row=1, column=0, sticky="w", padx=(0, 6), pady=2)

        # 录制状态
        self.status_label = tk.Label(
            status_frame,
            text="录制状态: 等待录制",
            font=self.font_panel_text,
            fg="#27ae60",
            bg="#34495e",
            anchor="w"
        )
        self.status_label.grid(row=1, column=1, sticky="w", padx=(6, 0), pady=2)

        # 控制模式
        if self.control_mode_var is None:
            self.control_mode_var = tk.StringVar(value="relative")
        mode_row = tk.Frame(status_frame, bg="#34495e")
        mode_row.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 2))
        mode_row.grid_columnconfigure(1, weight=1)

        mode_label = tk.Label(
            mode_row,
            text="控制模式:",
            font=self.font_panel_text,
            fg="#bdc3c7",
            bg="#34495e",
            anchor="w"
        )
        mode_label.grid(row=0, column=0, sticky="w")

        self.mode_abs_radio = tk.Radiobutton(
            mode_row,
            text="绝对",
            value="absolute",
            variable=self.control_mode_var,
            command=self._on_control_mode_change,
            font=self.font_panel_text,
            fg="white",
            bg="#34495e",
            selectcolor="#34495e",
            activebackground="#34495e",
            activeforeground="white"
        )
        self.mode_abs_radio.grid(row=0, column=1, sticky="w", padx=(6, 8))

        self.mode_rel_radio = tk.Radiobutton(
            mode_row,
            text="相对",
            value="relative",
            variable=self.control_mode_var,
            command=self._on_control_mode_change,
            font=self.font_panel_text,
            fg="white",
            bg="#34495e",
            selectcolor="#34495e",
            activebackground="#34495e",
            activeforeground="white"
        )
        self.mode_rel_radio.grid(row=0, column=2, sticky="w")

        # ====================
        # 录制信息框
        # ====================
        info_frame = tk.LabelFrame(
            control_content,
            text="录制信息",
            font=self.font_panel_section,
            fg="white",
            bg="#34495e",
            padx=8,
            pady=6
        )
        info_frame.pack(padx=10, pady=5, fill="x")

        # 录制时长
        time_row = tk.Frame(info_frame, bg="#34495e")
        time_row.pack(fill="x", pady=2)
        time_row.grid_columnconfigure(0, weight=1)
        time_row.grid_columnconfigure(1, weight=1)

        self.time_label = tk.Label(
            time_row,
            text="录制时长: 00:00:00",
            font=self.font_panel_text,
            fg="white",
            bg="#34495e",
            anchor="w"
        )
        self.time_label.grid(row=0, column=0, sticky="w")

        # 录制序号
        self.session_label = tk.Label(
            time_row,
            text="录制序号: --",
            font=self.font_panel_text,
            fg="#ecf0f1",
            bg="#34495e",
            anchor="e"
        )
        self.session_label.grid(row=0, column=1, sticky="e")

        # 保存路径
        self.path_title_label = tk.Label(
            info_frame,
            text="保存路径:",
            font=self.font_panel_text,
            fg="#95a5a6",
            bg="#34495e",
            anchor="w"
        )
        self.path_title_label.pack(fill="x", pady=(6, 2))

        self.video_path_label = tk.Label(
            info_frame,
            text=self._format_path_label("视频", self.current_video_path),
            font=self.font_panel_text,
            fg="#5dade2",
            bg="#34495e",
            anchor="w",
            cursor="hand2"
        )
        self.video_path_label.pack(fill="x", pady=2)

        self.action_path_label = tk.Label(
            info_frame,
            text=self._format_path_label("动作", self.current_action_path),
            font=self.font_panel_text,
            fg="#5dade2",
            bg="#34495e",
            anchor="w",
            cursor="hand2"
        )
        self.action_path_label.pack(fill="x", pady=(2, 4))
        self.video_path_label.bind(
            "<Button-1>",
            lambda event: self.open_path_in_file_manager(self.current_video_path)
        )
        self.action_path_label.bind(
            "<Button-1>",
            lambda event: self.open_path_in_file_manager(self.current_action_path)
        )

        # ====================
        # 控制按钮区域
        # ====================
        last_index = self.get_last_record_index()
        default_replay_index = last_index if last_index else 1
        self.replay_index_var = tk.IntVar(value=default_replay_index)

        button_frame = tk.Frame(control_content, bg="#34495e")
        button_frame.pack(pady=4, padx=10, fill="x")
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=0)

        # 开始录制按钮
        self.start_button = tk.Button(
            button_frame,
            text="开始录制",
            font=self.font_panel_section,
            bg="#27ae60",
            fg="white",
            height=1,
            command=self.start_recording,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.start_button.grid(row=0, column=0, sticky="ew", pady=(4, 4))

        # 停止录制按钮
        self.stop_button = tk.Button(
            button_frame,
            text="停止录制",
            font=self.font_panel_section,
            bg="#e74c3c",
            fg="white",
            height=1,
            state=tk.DISABLED,
            command=self.stop_recording,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.stop_button.grid(row=1, column=0, sticky="ew", pady=(0, 4))

        # 删除上次录制按钮
        self.delete_last_button = tk.Button(
            button_frame,
            text="删除上次录制",
            font=self.font_panel_text,
            bg="#f39c12",
            fg="white",
            height=1,
            command=self.delete_last_recording,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.delete_last_button.grid(row=2, column=0, sticky="ew", pady=(0, 4))

        # 重放按钮与选择序号
        self.replay_button = tk.Button(
            button_frame,
            text="重放",
            font=self.font_panel_section,
            bg="#2980b9",
            fg="white",
            height=1,
            command=self.start_replay,
            cursor="hand2",
            relief="raised",
            borderwidth=3
        )
        self.replay_button.grid(row=3, column=0, sticky="ew", pady=(0, 4))

        validate_cmd = (self.root.register(self._validate_replay_index), "%P")
        self.replay_spinbox = tk.Spinbox(
            button_frame,
            from_=1,
            to=999,
            width=6,
            textvariable=self.replay_index_var,
            font=self.font_panel_text,
            justify="center",
            command=self._on_replay_index_change,
            validate="key",
            validatecommand=validate_cmd
        )
        self.replay_spinbox.grid(row=3, column=1, sticky="e", pady=(0, 4))
        self._update_replay_controls()


    def init_cameras(self):
        """初始化所有相机"""
        try:
            for sn in CAMERA_SERIAL_NUMBERS:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(sn)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(config)
                self.pipelines.append(pipeline)
                print(f"相机 {sn} 初始化成功")

            self.camera_status_label.config(
                text=f"相机: 已连接 {len(CAMERA_SERIAL_NUMBERS)} 台",
                fg="#27ae60"
            )
        except Exception as e:
            self.camera_status_label.config(
                text=f"相机: 初始化失败 {str(e)}",
                fg="#e74c3c"
            )
            messagebox.showerror("错误", f"相机初始化失败: {str(e)}")

    def start_preview(self):
        """开始视频预览"""
        def preview_loop():
            last_update_time = 0
            last_record_time = 0
            update_interval = 1.0 / self.preview_fps
            record_interval = 1.0 / self.record_fps

            while self.running:
                current_time = time.time()

                # 控制帧率
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

                    if valid and len(images) == len(self.pipelines):
                        # 如果正在录制，保存原始帧（无REC标识）并记录对应动作
                        if self.is_recording:
                            frame_timestamp = time.time()
                            if frame_timestamp - last_record_time >= record_interval:
                                with self.record_lock:
                                    if self.is_recording:
                                        for i, color_image in enumerate(images):
                                            if i < len(self.frame_dirs):
                                                frame_path = os.path.join(
                                                    self.frame_dirs[i],
                                                    f"frame_{self.frame_index:06d}.jpg"
                                                )
                                                cv2.imwrite(frame_path, color_image)
                                        self._capture_action_sample(frame_timestamp)
                                        self.frame_index += 1
                                        last_record_time = frame_timestamp

                        # 为预览添加录制标识（不影响保存的视频）
                        if self.is_recording:
                            for color_image in images:
                                cv2.circle(color_image, (30, 30), 15, (0, 0, 255), -1)
                                cv2.putText(color_image, "REC", (55, 40),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    if valid and len(images) == 3:
                        # 水平拼接三张图像，增加间隔便于区分
                        combined = self._compose_preview_images(images)

                        # 转换为RGB用于显示
                        combined_rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)

                        # 转换为PIL Image
                        pil_img = Image.fromarray(combined_rgb)

                        # 在主线程中更新GUI
                        with self.frame_lock:
                            self.root.after(0, self.update_preview_label, pil_img)

                        last_update_time = current_time

                except Exception as e:
                    print(f"预览错误: {e}")
                    time.sleep(0.1)

        preview_thread = threading.Thread(target=preview_loop, daemon=True)
        preview_thread.start()

    def _on_preview_resize(self, event):
        """缓存预览区域尺寸，避免线程中调用Tk方法"""
        if event.width > 0 and event.height > 0:
            self.preview_size = (event.width, event.height)

    def _compose_preview_images(self, images):
        """拼接预览画面并添加间隔"""
        if not images:
            return None
        if len(images) != 3:
            try:
                return cv2.hconcat(images)
            except Exception:
                return images[0]

        h, w = images[0].shape[:2]
        gap = max(8, int(w * 0.02))
        cache = getattr(self, "_preview_gap_cache", {})
        if (
            cache.get("h") != h
            or cache.get("gap") != gap
            or cache.get("dtype") != images[0].dtype
        ):
            # 使用与预览背景一致的颜色（RGB #34495e -> BGR 94,73,52）
            gap_color = (94, 73, 52)
            gap_img = np.full((h, gap, 3), gap_color, dtype=images[0].dtype)
            cache = {"h": h, "gap": gap, "dtype": images[0].dtype, "img": gap_img}
            self._preview_gap_cache = cache

        gap_img = cache["img"]
        try:
            return cv2.hconcat([gap_img, images[0], gap_img, images[1], gap_img, images[2], gap_img])
        except Exception:
            return cv2.hconcat(images)

    def update_preview_label(self, pil_img):
        """在主线程中更新预览标签"""
        try:
            target_w, target_h = self.preview_size
            if target_w <= 0 or target_h <= 0:
                return

            img_w, img_h = pil_img.size
            scale = min(target_w / img_w, target_h / img_h)
            new_w = max(int(img_w * scale), 1)
            new_h = max(int(img_h * scale), 1)
            if new_w != img_w or new_h != img_h:
                pil_img = pil_img.resize((new_w, new_h), Image.LANCZOS)
            imgtk = ImageTk.PhotoImage(image=pil_img)
            self.preview_label.imgtk = imgtk
            self.preview_label.configure(image=imgtk)
        except:
            pass

    def request_action_redraw(self):
        """请求动作可视化刷新"""
        return

    def _schedule_action_redraw(self):
        """按固定间隔刷新动作曲线，避免高频触发导致卡顿"""
        if self._action_redraw_pending:
            return
        self._action_redraw_pending = True
        try:
            self.root.after(self.action_redraw_interval_ms, self._action_redraw_loop)
        except Exception:
            self._action_redraw_pending = False

    def _action_redraw_loop(self):
        """动作曲线刷新循环"""
        self._action_redraw_pending = False
        self.update_action_canvas()
        if self.running:
            self._schedule_action_redraw()

    def _extract_gripper_width(self, joint_names, positions):
        """提取夹爪开合"""
        if not positions:
            return None
        try:
            if not joint_names:
                if len(positions) >= 2:
                    return float(positions[0]) + float(positions[1])
                return float(positions[0])
            lower_names = [name.lower() for name in joint_names]
            left_idx = next(
                (
                    i for i, name in enumerate(lower_names)
                    if "left" in name and any(token in name for token in ("finger", "gripper", "knuckle"))
                ),
                None
            )
            right_idx = next(
                (
                    i for i, name in enumerate(lower_names)
                    if "right" in name and any(token in name for token in ("finger", "gripper", "knuckle"))
                ),
                None
            )
            if left_idx is not None and right_idx is not None and left_idx != right_idx:
                return float(positions[left_idx]) + float(positions[right_idx])

            indices = [
                i for i, name in enumerate(lower_names)
                if any(token in name for token in ("finger", "gripper", "knuckle"))
            ]
            if not indices:
                return None
            if len(indices) >= 2:
                return float(positions[indices[0]]) + float(positions[indices[1]])
            return float(positions[indices[0]])
        except Exception:
            return None

    def _append_action_history(self, data):
        """追加动作历史数据"""
        try:
            ts = data.get('timestamp') or time.time()
            franka = data.get('franka_joints') or {}
            joints = franka.get('position') or []

            # 从gripper_joints获取夹爪数据
            gripper_data = data.get('gripper_joints') or {}
            gripper_positions = gripper_data.get('position') or []
            gripper_names = gripper_data.get('names')
            gripper = self._extract_gripper_width(gripper_names, gripper_positions)
            if gripper is None and data.get('gripper_command') is not None:
                try:
                    gripper = float(data['gripper_command']) * GRIPPER_MAX_WIDTH
                except Exception:
                    gripper = None

            ee = data.get('end_effector_pose') or {}
            pos = ee.get('position') or {}
            ori = ee.get('orientation') or {}

            sample = {
                "t": float(ts),
                "joints": list(joints[:7]) if joints else [],
                "pos": (
                    float(pos.get('x', 0.0)),
                    float(pos.get('y', 0.0)),
                    float(pos.get('z', 0.0))
                ),
                "ori": (
                    float(ori.get('x', 0.0)),
                    float(ori.get('y', 0.0)),
                    float(ori.get('z', 0.0)),
                    float(ori.get('w', 0.0))
                ),
                "gripper": None if gripper is None else float(gripper)
            }
        except Exception:
            return

        with self.action_history_lock:
            self.action_history.append(sample)
    def _draw_time_series_canvas(self, canvas, title, points, color, y_min, y_max, t0, t1, show_time=False, time_text=None):
        """绘制单条时间序列"""
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w <= 10 or h <= 10:
            return

        pad = 6
        canvas.create_text(
            pad, pad,
            anchor="nw",
            text=title,
            font=self.font_small,
            fill="#ecf0f1"
        )
        if show_time and time_text:
            canvas.create_text(
                w - pad, pad,
                anchor="ne",
                text=f"时间: {time_text}",
                font=self.font_small,
                fill="#bdc3c7"
            )

        title_h = 18
        left_pad = 40
        right_pad = 10
        top = pad + title_h
        bottom = h - pad
        left = pad + left_pad
        right = w - right_pad
        if bottom <= top or right <= left:
            return

        canvas.create_rectangle(
            left, top,
            right, bottom,
            outline="#3b5166",
            fill="#1f2d3a"
        )

        if y_max <= y_min:
            return

        if y_min < 0 < y_max:
            zero_y = top + (1.0 - ((0 - y_min) / (y_max - y_min))) * (bottom - top)
            canvas.create_line(left, zero_y, right, zero_y, fill="#7f8c8d")

        canvas.create_text(
            pad, top - 2,
            anchor="nw",
            text=f"{y_max:.2f}",
            font=self.font_tiny,
            fill="#bdc3c7"
        )
        canvas.create_text(
            pad, bottom - 12,
            anchor="nw",
            text=f"{y_min:.2f}",
            font=self.font_tiny,
            fill="#bdc3c7"
        )

        if not points:
            return

        window = max(t1 - t0, 1e-6)
        width = right - left
        height = bottom - top

        max_points = 240
        step = max(1, len(points) // max_points)
        coords = []
        for t, v in points[::step]:
            if t < t0 or t > t1:
                continue
            try:
                v = float(v)
            except Exception:
                continue
            x_pos = left + ((t - t0) / window) * width
            y_pos = top + (1.0 - ((v - y_min) / (y_max - y_min))) * height
            coords.extend([x_pos, y_pos])

        if len(coords) >= 4:
            canvas.create_line(coords, fill=color, width=1.5)

    def update_action_canvas(self):
        """更新动作可视化"""
        if not self.action_canvases:
            return
        if not self.action_canvases[0].winfo_exists():
            return

        with self.action_history_lock:
            if self.action_history:
                t1 = self.action_history[-1]["t"]
            else:
                t1 = time.time()
            t0 = t1 - self.history_window_sec
            while self.action_history and self.action_history[0]["t"] < t0:
                self.action_history.popleft()
            samples = list(self.action_history)

        if not samples:
            for canvas in self.action_canvases:
                canvas.delete("all")
                w = canvas.winfo_width()
                h = canvas.winfo_height()
                if w <= 10 or h <= 10:
                    continue
                canvas.create_text(
                    w / 2, h / 2,
                    text="等待动作数据...",
                    font=self.font_medium,
                    fill="#bdc3c7"
                )
            return

        time_text = time.strftime("%H:%M:%S", time.localtime(t1))
        series_map = {
            "pos_x": [],
            "pos_y": [],
            "pos_z": [],
            "ori_x": [],
            "ori_y": [],
            "ori_z": [],
            "ori_w": [],
            "grip": []
        }

        for sample in samples:
            t = sample.get("t")
            if t is None:
                continue
            pos = sample.get("pos") or (0.0, 0.0, 0.0)
            ori = sample.get("ori") or (0.0, 0.0, 0.0, 1.0)
            grip = sample.get("gripper")

            series_map["pos_x"].append((t, pos[0]))
            series_map["pos_y"].append((t, pos[1]))
            series_map["pos_z"].append((t, pos[2]))
            series_map["ori_x"].append((t, ori[0]))
            series_map["ori_y"].append((t, ori[1]))
            series_map["ori_z"].append((t, ori[2]))
            series_map["ori_w"].append((t, ori[3]))
            if grip is not None:
                series_map["grip"].append((t, grip))

        for idx, spec in enumerate(self.action_canvas_specs):
            canvas = self.action_canvases[idx]
            key = spec["key"]
            points = series_map.get(key, [])
            y_min, y_max = self._compute_series_range(
                points,
                spec["ymin"],
                spec["ymax"],
                spec["min_span"]
            )
            show_time = idx == 0
            self._draw_time_series_canvas(
                canvas,
                spec["title"],
                points,
                spec["color"],
                y_min,
                y_max,
                t0,
                t1,
                show_time=show_time,
                time_text=time_text
            )

    def get_next_record_index(self):
        """获取下一个录制序号"""
        last_index = self.get_last_record_index()
        return (last_index or 0) + 1

    def get_last_record_index(self):
        """获取最后一次录制序号"""
        max_index = 0

        # 新结构: /data/record_XXX/
        if os.path.exists(self.record_base_path):
            for name in os.listdir(self.record_base_path):
                if not name.startswith("record_"):
                    continue
                path = os.path.join(self.record_base_path, name)
                if not os.path.isdir(path):
                    continue
                try:
                    idx = int(name.split("_")[1])
                    max_index = max(max_index, idx)
                except Exception:
                    pass

        # 兼容旧结构: /data/video/record_XXX 和 /data/action/record_XXX
        for legacy_base in (self.legacy_video_base_path, self.legacy_action_base_path):
            if os.path.exists(legacy_base):
                for name in os.listdir(legacy_base):
                    if name.startswith("record_"):
                        try:
                            idx = int(name.split("_")[1])
                            max_index = max(max_index, idx)
                        except Exception:
                            pass

        if max_index <= 0:
            return None
        return max_index

    def delete_last_recording(self):
        """删除上一次录制数据"""
        if self.is_recording:
            messagebox.showwarning("提示", "录制中无法删除上次数据")
            return
        if self.replay_in_progress:
            messagebox.showwarning("提示", "回放中无法删除上次数据")
            return

        last_index = self.get_last_record_index()
        if not last_index:
            messagebox.showinfo("提示", "没有找到可删除的录制数据")
            return

        record_name = f"record_{last_index:03d}"
        record_dir = os.path.join(self.record_base_path, record_name)
        video_dir = os.path.join(record_dir, "video")
        action_dir = os.path.join(record_dir, "action")
        legacy_video_dir = os.path.join(self.legacy_video_base_path, record_name)
        legacy_action_dir = os.path.join(self.legacy_action_base_path, record_name)

        confirm = messagebox.askyesno(
            "确认删除",
            f"确定删除上一次录制数据？\n{record_name}"
        )
        if not confirm:
            return

        errors = []
        deleted_any = False

        # 删除原始数据（视频 + 动作）
        paths_to_delete = []
        if os.path.exists(record_dir):
            paths_to_delete.append(record_dir)
        else:
            if os.path.exists(legacy_video_dir):
                paths_to_delete.append(legacy_video_dir)
            if os.path.exists(legacy_action_dir):
                paths_to_delete.append(legacy_action_dir)

        for path in paths_to_delete:
            try:
                shutil.rmtree(path)
                deleted_any = True
            except Exception as e:
                errors.append(f"{path}: {e}")

        if errors:
            messagebox.showerror("错误", "删除失败:\n" + "\n".join(errors))
            return

        if not deleted_any:
            messagebox.showinfo("提示", "未找到需要删除的数据")
            return

        new_last = self.get_last_record_index()
        if new_last:
            new_record = f"record_{new_last:03d}"
            self.session_label.config(text=f"录制序号: {new_record}")
            record_dir = os.path.join(self.record_base_path, new_record)
            if os.path.isdir(record_dir):
                video_path = os.path.join(record_dir, "video")
                action_path = os.path.join(record_dir, "action")
            else:
                video_path = os.path.join(self.legacy_video_base_path, new_record)
                action_path = os.path.join(self.legacy_action_base_path, new_record)
            self.set_current_paths(video_path, action_path)
            self.session_index = new_last
        else:
            self.session_label.config(text="录制序号: --")
            self.set_current_paths(self.record_base_path, self.record_base_path)
            self.session_index = None

        if new_last:
            self.replay_index_var.set(new_last)
        self._update_replay_controls()

        messagebox.showinfo("完成", f"已删除 {record_name} 的数据")

    def _get_remote_script_for_mode(self, control_mode):
        return REMOTE_RELATIVE_SCRIPT if control_mode == "relative" else REMOTE_ABSOLUTE_SCRIPT

    def _ssh_base_cmd(self):
        base = ["ssh", "-o", "StrictHostKeyChecking=no"]
        if REMOTE_PASSWORD:
            return ["sshpass", "-p", REMOTE_PASSWORD] + base
        return base

    def _scp_base_cmd(self):
        base = ["scp", "-o", "StrictHostKeyChecking=no"]
        if REMOTE_PASSWORD:
            return ["sshpass", "-p", REMOTE_PASSWORD] + base
        return base

    def _run_remote_command(self, command, timeout=10, check=False):
        return subprocess.run(
            self._ssh_base_cmd() + [f"{REMOTE_USER}@{REMOTE_HOST}", command],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=check,
        )

    def _cleanup_remote_for_start(self):
        cleanup_cmd = (
            "pkill -9 -f '[r]emote_data_bridge.py' 2>/dev/null || true; "
            "pkill -9 -f '[r]un_fr3_real_ros2_robotiq.sh' 2>/dev/null || true; "
            "pkill -9 -f '[s]tart_relative_gello_control.sh' 2>/dev/null || true; "
            "pkill -9 -f '[g]ello_relative_publisher' 2>/dev/null || true; "
            "pkill -9 -f '[g]ello_publisher' 2>/dev/null || true; "
            "pkill -9 -f '[r]os2_control_node' 2>/dev/null || true; "
            "pkill -9 -f '[f]ranka_gripper' 2>/dev/null || true; "
            "pkill -9 -f '[f]ranka_fr3' 2>/dev/null || true"
        )
        return self._run_remote_command(cleanup_cmd, timeout=10)

    def _parse_number_list(self, text):
        if not text:
            return None
        values = []
        for token in re.split(r"[\s,]+", text.strip()):
            if not token:
                continue
            try:
                values.append(float(token))
            except ValueError:
                continue
        return values or None

    def _get_remote_robot_zero(self):
        result = self._run_remote_command(f"cat {shlex.quote(REMOTE_RELATIVE_CONFIG)}", timeout=10)
        if result.returncode != 0:
            return None
        content = result.stdout
        if not content:
            return None

        inline = re.search(r"robot_zero\s*:\s*\[(.*?)\]", content)
        if inline:
            values = self._parse_number_list(inline.group(1))
            if values and len(values) >= 7:
                return values[:7]

        values = []
        in_block = False
        for line in content.splitlines():
            line = line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if not in_block:
                if re.match(r"^\s*robot_zero\s*:\s*$", line):
                    in_block = True
                continue
            if re.match(r"^\S", line):
                break
            item_match = re.match(r"^\s*-\s*([-+0-9.eE]+)", line)
            if item_match:
                try:
                    values.append(float(item_match.group(1)))
                except ValueError:
                    pass
            elif line.strip() and not line.lstrip().startswith("-"):
                break
        if values and len(values) >= 7:
            return values[:7]
        return None

    def _extract_franka_positions(self, data):
        if not isinstance(data, dict):
            return None
        franka = data.get("franka_joints")
        if not isinstance(franka, dict):
            return None
        positions = franka.get("position")
        if not positions:
            return None
        names = franka.get("names") or []
        order = self._get_franka_order(names)
        if order and len(positions) >= len(order):
            return [positions[i] for i in order]
        return list(positions[:7])

    def _wait_for_robot_zero(self, robot_zero, timeout_sec):
        if not robot_zero or len(robot_zero) < 7:
            return False
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            with self.action_sample_lock:
                latest = copy.deepcopy(self.latest_action_data)
            positions = self._extract_franka_positions(latest)
            if positions:
                diffs = [abs(p - z) for p, z in zip(positions, robot_zero)]
                max_diff = max(diffs) if diffs else None
                if max_diff is not None and max_diff <= self.relative_zero_tolerance:
                    return True
                if max_diff is not None:
                    self._set_label_text(
                        self.robot_ready_label,
                        f"机器人状态: 等待回零 (偏差 {max_diff:.3f}rad)",
                        "#f39c12",
                    )
            time.sleep(0.1)
        return False

    def _set_relative_control_enabled(self, enabled, retries=5, wait_sec=1.0, raise_on_failure=False):
        data_value = "true" if enabled else "false"
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/franka_ros2_ws/install/setup.bash && "
            "source ~/gello_software/ros2/install/setup.bash && "
            "ros2 service call /gello_relative/enable std_srvs/srv/SetBool "
            f"\"{{data: {data_value}}}\""
        )
        last_error = None
        for _ in range(retries):
            try:
                result = self._run_remote_command(cmd, timeout=10)
            except Exception as exc:
                last_error = str(exc)
                result = None
            if result and result.returncode == 0:
                return True
            if result:
                last_error = result.stderr.strip() or result.stdout.strip()
            time.sleep(wait_sec)
        if raise_on_failure:
            raise RuntimeError(last_error or "相对控制切换失败")
        return False

    def start_remote_robot(self, control_mode=None):
        """启动远程机器人系统"""
        try:
            if control_mode not in ("absolute", "relative"):
                control_mode = "absolute"
            print("启动远程ROS2系统...")
            self._set_label_text(self.robot_connection_label, "机器人连接: 正在连接...", "#f39c12")
            self._set_label_text(self.robot_ready_label, "机器人状态: 清理旧进程...", "#f39c12")

            # 先清理所有旧的远程进程，确保干净的启动状态
            print("清理所有旧的远程进程...")
            self._cleanup_remote_for_start()
            time.sleep(2)

            # 上传数据桥接脚本
            bridge_script = "/home/ubuntu/take_data/take_action/scripts/remote_data_bridge.py"

            # 创建远程目录
            subprocess.run([
                *self._ssh_base_cmd(),
                f"{REMOTE_USER}@{REMOTE_HOST}",
                f"mkdir -p {REMOTE_BRIDGE_DIR}"
            ], check=False, capture_output=True)

            # 上传桥接脚本
            subprocess.run([
                *self._scp_base_cmd(),
                bridge_script,
                f"{REMOTE_USER}@{REMOTE_HOST}:{REMOTE_BRIDGE_DIR}/remote_data_bridge.py"
            ], check=False, capture_output=True)

            # 启动ROS2系统（每次都重新启动，确保状态干净）
            self._set_label_text(self.robot_ready_label, "机器人状态: 启动ROS2系统...", "#f39c12")

            remote_script = self._get_remote_script_for_mode(control_mode)
            log_basename = os.path.splitext(os.path.basename(remote_script))[0]
            ros_cmd = f"nohup {remote_script} > /tmp/{log_basename}.log 2>&1 &"
            subprocess.Popen([
                *self._ssh_base_cmd(),
                f"{REMOTE_USER}@{REMOTE_HOST}",
                ros_cmd
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # 等待ROS2启动，显示倒计时
            for i in range(15, 0, -1):
                self._set_label_text(
                    self.robot_ready_label,
                    f"机器人状态: ROS2启动中 ({i}秒)...",
                    "#f39c12"
                )
                time.sleep(1)

            # 启动数据桥接
            self._set_label_text(self.robot_ready_label, "机器人状态: 启动数据桥接...", "#f39c12")
            bridge_cmd = (
                f"cd {REMOTE_BRIDGE_DIR} && "
                "source /opt/ros/humble/setup.bash && "
                "source ~/franka_ros2_ws/install/setup.bash && "
                "source ~/gello_software/ros2/install/setup.bash && "
                "nohup python3 remote_data_bridge.py > /tmp/bridge.log 2>&1 &"
            )
            subprocess.Popen([
                *self._ssh_base_cmd(),
                f"{REMOTE_USER}@{REMOTE_HOST}",
                bridge_cmd
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # 等待桥接启动
            for i in range(5, 0, -1):
                self._set_label_text(
                    self.robot_ready_label,
                    f"机器人状态: 桥接启动中 ({i}秒)...",
                    "#f39c12"
                )
                time.sleep(1)

            # 连接到数据桥接（带重试机制）
            self._set_label_text(self.robot_ready_label, "机器人状态: 连接数据流...", "#f39c12")

            self.action_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.action_socket.settimeout(2)  # 设置2秒超时

            # 重试连接最多10次，每次间隔1秒
            connected = False
            for attempt in range(10):
                try:
                    self.action_socket.connect((REMOTE_HOST, REMOTE_PORT))
                    connected = True
                    break
                except (socket.timeout, ConnectionRefusedError, OSError) as e:
                    if attempt < 9:  # 不是最后一次尝试
                        self._set_label_text(
                            self.robot_ready_label,
                            f"机器人状态: 连接中... ({attempt+1}/10)",
                            "#f39c12"
                        )
                        time.sleep(1)
                        if self.action_socket:
                            self.action_socket.close()
                        self.action_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.action_socket.settimeout(2)
                    else:
                        raise e

            if not connected:
                raise Exception("无法连接到数据桥接服务器")

            self.action_socket.settimeout(None)  # 恢复为阻塞模式

            if control_mode == "relative":
                if not self._set_relative_control_enabled(False):
                    print("警告: 未能关闭相对控制")

            self._set_label_text(self.robot_connection_label, "机器人连接: 已连接", "#27ae60")
            self._set_label_text(self.robot_ready_label, "机器人状态: 等待数据...", "#f39c12")
            self.active_control_mode = control_mode
            print("远程机器人系统启动成功")
            return True

        except Exception as e:
            if self.action_socket:
                try:
                    self.action_socket.close()
                except Exception:
                    pass
                self.action_socket = None
            self._set_label_text(self.robot_connection_label, "机器人连接: 连接失败", "#e74c3c")
            self._set_label_text(self.robot_ready_label, f"机器人状态: {str(e)}", "#e74c3c")
            self.robot_ready = False
            print(f"远程机器人启动失败: {e}")
            return False

    def receive_action_data(self):
        """接收动作数据"""
        sock = self.action_socket
        if not sock:
            return
        buffer = ''
        while self.running and sock:
            try:
                chunk = sock.recv(4096).decode('utf-8')
                if not chunk:
                    break

                buffer += chunk

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            recv_ts = time.time()
                            data = json.loads(line)
                            with self.action_sample_lock:
                                self.latest_action_data = data
                                remote_ts = data.get("timestamp")
                                if isinstance(remote_ts, (int, float)):
                                    offset = recv_ts - remote_ts
                                    self.action_time_offsets.append(offset)
                                    self.action_time_offset = min(self.action_time_offsets)
                                self.action_sample_buffer.append({
                                    "remote_ts": remote_ts if isinstance(remote_ts, (int, float)) else None,
                                    "local_recv_ts": recv_ts,
                                    "data": data,
                                })
                                cutoff = recv_ts - self.action_sample_window_sec
                                while (
                                    self.action_sample_buffer
                                    and self.action_sample_buffer[0]["local_recv_ts"] < cutoff
                                ):
                                    self.action_sample_buffer.popleft()
                            self._append_action_history(data)
                            self.request_action_redraw()
                            if not self.robot_ready:
                                self.robot_ready = True
                                self._set_label_text(
                                    self.robot_ready_label,
                                    "机器人状态: 就绪",
                                    "#27ae60"
                                )
                        except json.JSONDecodeError:
                            pass

            except Exception as e:
                print(f"接收数据错误: {e}")
                break
        try:
            sock.close()
        except Exception:
            pass
        if self.action_socket is sock:
            self.action_socket = None
            self.robot_ready = False
            self._set_label_text(self.robot_connection_label, "机器人连接: 未连接", "#95a5a6")
            self._set_label_text(self.robot_ready_label, "机器人状态: 待机中", "#95a5a6")

    def _get_aligned_action_time(self, sample):
        """Estimate local time for an action sample."""
        remote_ts = sample.get("remote_ts")
        if remote_ts is None or self.action_time_offset is None:
            return sample["local_recv_ts"]
        return remote_ts + self.action_time_offset

    def _capture_action_sample(self, frame_timestamp):
        """记录与当前视频帧对应的动作数据快照"""
        with self.action_sample_lock:
            if not self.action_sample_buffer:
                latest = self.latest_action_data
                if not latest:
                    return
                chosen = latest
            else:
                best_sample = None
                best_diff = None
                for sample in self.action_sample_buffer:
                    aligned_ts = self._get_aligned_action_time(sample)
                    diff = abs(aligned_ts - frame_timestamp)
                    if best_diff is None or diff < best_diff:
                        best_diff = diff
                        best_sample = sample
                if not best_sample:
                    return
                chosen = best_sample["data"]
        sample = copy.deepcopy(chosen)
        sample["frame_timestamp"] = frame_timestamp
        self.action_data_buffer.append(sample)

    def _reorder_franka_block(self, joint_block):
        """Return a reordered Franka joint block for saving."""
        names = joint_block.get("names") or []
        order = self._get_franka_order(names)
        if not order:
            return joint_block, order, False

        new_block = joint_block.copy()
        for field in ("position", "velocity", "effort"):
            values = joint_block.get(field)
            if values is None:
                continue
            if len(values) < len(order):
                continue
            new_block[field] = [values[i] for i in order]

        new_block["names"] = list(FR3_JOINT_ORDER)
        return new_block, order, new_block != joint_block

    def _get_franka_order(self, names):
        if not names:
            return None
        key = tuple(names)
        if key in self._joint_order_cache:
            return self._joint_order_cache[key]

        order = None
        if all(name in names for name in FR3_JOINT_ORDER):
            order = [names.index(name) for name in FR3_JOINT_ORDER]
        else:
            index_map = {}
            for idx, name in enumerate(names):
                match = JOINT_NUMBER_RE.search(name)
                if not match:
                    continue
                joint_num = int(match.group(1))
                if 1 <= joint_num <= 7 and joint_num not in index_map:
                    index_map[joint_num] = idx
            if len(index_map) == 7:
                order = [index_map[i] for i in range(1, 8)]

        self._joint_order_cache[key] = order
        return order

    def start_recording(self):
        """开始录制"""
        if self.is_recording:
            return
        if self.replay_in_progress:
            messagebox.showwarning("提示", "回放中无法录制")
            return

        try:
            if not self.robot_ready or not self.action_socket:
                messagebox.showwarning("提示", "机器人未就绪，请稍后再试")
                return
            if self.active_control_mode and self.active_control_mode != self._get_selected_control_mode():
                messagebox.showwarning("提示", "控制模式切换中，请稍后再试")
                return

            if self._get_selected_control_mode() == "relative":
                self.start_button.config(state=tk.DISABLED)
                self._set_mode_controls_state(tk.DISABLED)
                prep_thread = threading.Thread(
                    target=self._start_relative_recording_worker,
                    daemon=True
                )
                prep_thread.start()
                return

            self._start_recording_core()

        except Exception as e:
            messagebox.showerror("错误", f"启动录制失败: {str(e)}")

    def _start_recording_core(self):
        """实际执行录制启动逻辑"""
        try:
            # 获取录制序号
            self.session_index = self.get_next_record_index()
            self.record_start_time = time.time()

            # 创建录制目录
            record_dir = os.path.join(
                self.record_base_path,
                f"record_{self.session_index:03d}"
            )
            video_session_dir = os.path.join(record_dir, "video")
            action_session_dir = os.path.join(record_dir, "action")

            os.makedirs(video_session_dir, exist_ok=True)
            os.makedirs(action_session_dir, exist_ok=True)

            self.session_dir = {
                'video': video_session_dir,
                'action': action_session_dir
            }

            # 更新UI
            record_name = f"record_{self.session_index:03d}"
            self.session_label.config(text=f"录制序号: {record_name}")
            self.set_current_paths(video_session_dir, action_session_dir)

            # 清空数据缓冲
            self.action_data_buffer = []

            # 创建帧目录
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.frame_dirs = []
            self.video_output_paths = []
            self.frame_index = 0

            for i in range(len(self.pipelines)):
                frame_dir = os.path.join(
                    video_session_dir,
                    f"camera_{i+1}_{timestamp}_frames"
                )
                os.makedirs(frame_dir, exist_ok=True)
                self.frame_dirs.append(frame_dir)
                video_path = os.path.join(
                    video_session_dir,
                    f"camera_{i+1}_{timestamp}.mp4"
                )
                self.video_output_paths.append(video_path)
                print(f"创建帧目录: {frame_dir}")

            # 开始录制
            with self.record_lock:
                self.is_recording = True

            # 更新UI
            self.status_label.config(text="录制状态: 正在录制", fg="#e74c3c")
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.delete_last_button.config(state=tk.DISABLED)
            self._update_replay_controls()

            # 开始更新录制时长
            self.update_duration()

            print(f"开始录制: record_{self.session_index:03d}")
            return True

        except Exception as e:
            messagebox.showerror("错误", f"启动录制失败: {str(e)}")
            if self.is_recording:
                self.stop_recording()
            return False

    def _start_recording_with_relative(self):
        """相对控制准备完成后执行录制"""
        ok = self._start_recording_core()
        if not ok:
            self._set_relative_control_enabled(False)
            self.start_button.config(state=tk.NORMAL)
            self._set_mode_controls_state(tk.NORMAL)

    def _start_relative_recording_worker(self):
        error = None
        try:
            self._set_label_text(self.robot_ready_label, "机器人状态: 相对模式准备中...", "#f39c12")
            if not self._set_relative_control_enabled(False, raise_on_failure=True):
                raise RuntimeError("关闭相对控制失败")
            robot_zero = self._get_remote_robot_zero()
            if not robot_zero:
                raise RuntimeError("无法读取相对控制初始位姿")
            if not self._wait_for_robot_zero(robot_zero, self.relative_zero_timeout_sec):
                raise RuntimeError("等待机器人回到初始位姿超时")
            if not self._set_relative_control_enabled(True, raise_on_failure=True):
                raise RuntimeError("启用相对控制失败")
            self.root.after(0, self._start_recording_with_relative)
            return
        except Exception as exc:
            error = str(exc)
        finally:
            if error:
                def _fail():
                    messagebox.showerror("错误", f"相对控制准备失败: {error}")
                    self.start_button.config(state=tk.NORMAL)
                self.root.after(0, _fail)
                try:
                    self._set_relative_control_enabled(False)
                except Exception:
                    pass
            self.root.after(0, lambda: self._set_mode_controls_state(tk.NORMAL))

    def stop_recording(self):
        """停止录制"""
        if not self.is_recording:
            return

        try:
            if self.active_control_mode == "relative":
                self._set_relative_control_enabled(False)

            # 关闭录制
            with self.record_lock:
                self.is_recording = False

            # 由帧生成视频
            self._convert_frames_to_videos()

            # 保存动作数据
            if self.action_data_buffer and self.session_dir:
                self.save_action_data()

            # 更新UI
            self.status_label.config(text="录制状态: 录制已停止", fg="#95a5a6")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)
            self.delete_last_button.config(state=tk.NORMAL)
            if self.session_index:
                self.replay_index_var.set(self.session_index)
            self._update_replay_controls()

            messagebox.showinfo("完成", f"录制完成！\n已保存到: record_{self.session_index:03d}")
            print(f"录制完成: record_{self.session_index:03d}")

        except Exception as e:
            messagebox.showerror("错误", f"停止录制失败: {str(e)}")

    def _convert_frames_to_videos(self):
        """将帧目录转换为视频文件"""
        if not self.frame_dirs:
            return
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        for i, frame_dir in enumerate(self.frame_dirs):
            try:
                frame_files = [
                    name for name in os.listdir(frame_dir)
                    if name.startswith("frame_") and name.endswith(".jpg")
                ]
                frame_files.sort()
                if not frame_files:
                    continue
                first_path = os.path.join(frame_dir, frame_files[0])
                first_img = cv2.imread(first_path)
                if first_img is None:
                    continue
                height, width = first_img.shape[:2]
                video_path = (
                    self.video_output_paths[i]
                    if i < len(self.video_output_paths)
                    else os.path.join(self.session_dir["video"], f"camera_{i+1}.mp4")
                )
                writer = cv2.VideoWriter(video_path, fourcc, self.record_fps, (width, height))
                for name in frame_files:
                    frame_path = os.path.join(frame_dir, name)
                    img = cv2.imread(frame_path)
                    if img is None:
                        continue
                    writer.write(img)
                writer.release()
                print(f"生成视频文件: {video_path}")
            except Exception as e:
                print(f"生成视频失败: {frame_dir} ({e})")
    def save_action_data(self):
        """保存动作数据"""
        if not self.action_data_buffer or not self.session_dir:
            return

        action_dir = self.session_dir['action']
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        data_to_save = []
        for frame in self.action_data_buffer:
            if not isinstance(frame, dict):
                data_to_save.append(frame)
                continue
            franka = frame.get("franka_joints")
            if isinstance(franka, dict):
                new_franka, _, changed = self._reorder_franka_block(franka)
                if changed:
                    new_frame = frame.copy()
                    new_frame["franka_joints"] = new_franka
                    data_to_save.append(new_frame)
                    continue
            data_to_save.append(frame)

        # 保存JSON
        json_file = os.path.join(action_dir, f"action_data_{timestamp}.json")
        with open(json_file, 'w') as f:
            json.dump({
                'session': f"record_{self.session_index:03d}",
                'timestamp': timestamp,
                'packet_count': len(data_to_save),
                'data': data_to_save
            }, f, indent=2)

        # 保存HDF5
        hdf5_file = os.path.join(action_dir, f"action_data_{timestamp}.h5")
        with h5py.File(hdf5_file, 'w') as f:
            f.attrs['session'] = f"record_{self.session_index:03d}"
            f.attrs['timestamp'] = timestamp
            f.attrs['packet_count'] = len(data_to_save)

            # 提取数据
            timestamps = []
            gello_pos = []
            franka_pos = []
            franka_vel = []
            franka_eff = []
            gripper_pos = []
            gripper_vel = []
            gripper_eff = []
            ee_pos = []
            ee_ori = []
            gripper_joint_len = None

            for data in data_to_save:
                frame_ts = data.get('frame_timestamp')
                if frame_ts is None:
                    frame_ts = data.get('timestamp', 0)
                timestamps.append(frame_ts)

                if data.get('gello_joints'):
                    gello_pos.append(data['gello_joints'].get('position', [0]*7))
                else:
                    gello_pos.append([0]*7)

                if data.get('franka_joints'):
                    franka_pos.append(data['franka_joints'].get('position', [0]*7))
                    franka_vel.append(data['franka_joints'].get('velocity', [0]*7))
                    franka_eff.append(data['franka_joints'].get('effort', [0]*7))
                else:
                    franka_pos.append([0]*7)
                    franka_vel.append([0]*7)
                    franka_eff.append([0]*7)

                # 提取夹爪数据
                gripper_data = data.get('gripper_joints') or {}
                if gripper_data:
                    g_pos = gripper_data.get('position', [])
                    g_vel = gripper_data.get('velocity', [])
                    g_eff = gripper_data.get('effort', [])
                    if g_pos:
                        if gripper_joint_len is None or len(g_pos) > gripper_joint_len:
                            gripper_joint_len = len(g_pos)
                    # 夹爪关节数可能不同
                    gripper_pos.append(list(g_pos) if g_pos else None)
                    gripper_vel.append(list(g_vel) if g_vel else None)
                    gripper_eff.append(list(g_eff) if g_eff else None)
                elif data.get('gripper_command') is not None:
                    try:
                        width = float(data['gripper_command']) * GRIPPER_MAX_WIDTH
                        half_width = width / 2.0
                        gripper_pos.append([half_width, half_width])
                    except Exception:
                        gripper_pos.append(None)
                    gripper_vel.append(None)
                    gripper_eff.append(None)
                else:
                    gripper_pos.append(None)
                    gripper_vel.append(None)
                    gripper_eff.append(None)

                if data.get('end_effector_pose'):
                    pos = data['end_effector_pose']['position']
                    ori = data['end_effector_pose']['orientation']
                    ee_pos.append([pos['x'], pos['y'], pos['z']])
                    ee_ori.append([ori['x'], ori['y'], ori['z'], ori['w']])
                else:
                    ee_pos.append([0, 0, 0])
                    ee_ori.append([0, 0, 0, 1])

            if gripper_joint_len is None:
                gripper_joint_len = 2

            def _pad_gripper(values):
                if values is None:
                    return [0] * gripper_joint_len
                if len(values) >= gripper_joint_len:
                    return list(values[:gripper_joint_len])
                return list(values) + [0] * (gripper_joint_len - len(values))

            gripper_pos = [_pad_gripper(v) for v in gripper_pos]
            gripper_vel = [_pad_gripper(v) for v in gripper_vel]
            gripper_eff = [_pad_gripper(v) for v in gripper_eff]

            # 保存数据集
            f.create_dataset('timestamps', data=np.array(timestamps))
            f.create_dataset('gello/positions', data=np.array(gello_pos))
            f.create_dataset('franka/positions', data=np.array(franka_pos))
            f.create_dataset('franka/velocities', data=np.array(franka_vel))
            f.create_dataset('franka/efforts', data=np.array(franka_eff))
            f.create_dataset('gripper/positions', data=np.array(gripper_pos))
            f.create_dataset('gripper/velocities', data=np.array(gripper_vel))
            f.create_dataset('gripper/efforts', data=np.array(gripper_eff))
            f.create_dataset('end_effector/positions', data=np.array(ee_pos))
            f.create_dataset('end_effector/orientations', data=np.array(ee_ori))

        print(f"动作数据已保存: {action_dir}")

    def update_duration(self):
        """更新录制时长"""
        if self.is_recording and self.record_start_time:
            elapsed = int(time.time() - self.record_start_time)
            hours = elapsed // 3600
            minutes = (elapsed % 3600) // 60
            seconds = elapsed % 60

            self.time_label.config(text=f"录制时长: {hours:02d}:{minutes:02d}:{seconds:02d}")
            self.root.after(1000, self.update_duration)

    def cleanup_remote_processes(self):
        """清理远程进程"""
        try:
            print("清理远程进程...")
            subprocess.run([
                *self._ssh_base_cmd(),
                f"{REMOTE_USER}@{REMOTE_HOST}",
                "pkill -9 -f 'remote_data_bridge.py'; "
                "pkill -9 -f 'run_fr3_real_ros2_robotiq.sh'; "
                "pkill -9 -f 'start_relative_gello_control.sh'; "
                "pkill -9 -f 'gello_relative_publisher'; "
                "pkill -9 -f 'gello_publisher'; "
                "pkill -9 -f 'franka_fr3'; "
                "pkill -9 -f 'franka_gripper'; "
                "pkill -9 -f 'ros2'; "
                "rm -rf /tmp/robot_data_bridge"
            ], check=False, capture_output=True, timeout=5)
            print("远程进程已清理")
        except Exception as e:
            print(f"清理远程进程错误: {e}")

    def on_window_close(self):
        """窗口关闭事件"""
        if self.is_recording:
            if messagebox.askyesno("确认", "正在录制中，确定要退出吗？\n（数据将会保存）"):
                self.stop_recording()
                self.cleanup_and_exit()
        else:
            if messagebox.askyesno("确认", "确定要退出吗？"):
                self.cleanup_and_exit()

    def on_exit_button(self):
        """退出按钮事件"""
        self.on_window_close()

    def cleanup_and_exit(self):
        """清理并退出"""
        print("正在退出...")
        self.running = False

        if self.is_recording:
            self.stop_recording()

        if self.action_socket:
            try:
                self.action_socket.close()
            except Exception:
                pass
            self.action_socket = None

        # 释放相机
        for pipeline in self.pipelines:
            try:
                pipeline.stop()
            except:
                pass

        # 清理远程进程
        self.cleanup_remote_processes()

        # 关闭窗口
        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass

        sys.exit(0)

    def run(self):
        """运行主循环"""
        self.root.mainloop()


def main():
    try:
        app = VideoActionRecorder()
        app.run()
    except KeyboardInterrupt:
        print("\n程序被中断")
        sys.exit(0)
    except Exception as e:
        print(f"程序错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
