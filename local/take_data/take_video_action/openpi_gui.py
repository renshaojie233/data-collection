#!/usr/bin/env python3
import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pyrealsense2 as rs
from openpi_client import image_tools
from openpi_client import websocket_client_policy
from PyQt5 import QtCore, QtGui, QtWidgets

DEFAULT_SERIALS = [
    "243722070232",
    "243622073040",
    "243722073691",
]

CONFIG_FILE = "/home/ubuntu/take_data/take_video_action/video_action_recorder.py"
DEFAULT_ACTION_JSON = (
    "/home/ubuntu/take_data/data/record_001/action/15HZ/"
    "action_data_20251230_012732.json"
)
MAX_JOINT_DELTA = 0.05
RELATIVE_MAX_JOINT_DELTA = np.array([MAX_JOINT_DELTA] * 7, dtype=np.float32)
RELATIVE_MAX_JOINT_VEL = RELATIVE_MAX_JOINT_DELTA / MAX_JOINT_DELTA

def set_max_joint_delta(value: float) -> None:
    global MAX_JOINT_DELTA, RELATIVE_MAX_JOINT_DELTA, RELATIVE_MAX_JOINT_VEL
    MAX_JOINT_DELTA = max(1e-6, float(value))
    RELATIVE_MAX_JOINT_DELTA = np.array([MAX_JOINT_DELTA] * 7, dtype=np.float32)
    RELATIVE_MAX_JOINT_VEL = RELATIVE_MAX_JOINT_DELTA / MAX_JOINT_DELTA

class ActionPositionReader:
    """读取 JSON 文件中的 franka_joints.position 和 gripper_command 作为 action 输出"""
    
    def __init__(self, json_path: str = DEFAULT_ACTION_JSON):
        self.json_path = json_path
        self.positions: List[List[float]] = []
        self.gripper_commands: List[float] = []
        self.current_index = 0
        self._load_data()
    
    def _load_data(self):
        """加载 JSON 文件并提取所有 position 和 gripper_command"""
        with open(self.json_path, 'r') as f:
            data = json.load(f)
        
        # 从 data 字段中提取每个时间步的 franka_joints.position 和 gripper_command
        for item in data.get('data', []):
            franka_joints = item.get('franka_joints', {})
            position = franka_joints.get('position')
            gripper_command = item.get('gripper_command', 0.0)
            
            if position is not None:
                self.positions.append(position)
                # 如果 gripper_command 是 None，使用默认值 0.0
                if gripper_command is None:
                    gripper_command = 0.0
                self.gripper_commands.append(float(gripper_command))
    
    def get_next(self) -> Optional[Tuple[List[float], float]]:
        """获取下一个时间步的 (position, gripper_command)，如果已经读完则返回 None"""
        if self.current_index >= len(self.positions):
            return None
        position = self.positions[self.current_index]
        gripper_command = self.gripper_commands[self.current_index]
        self.current_index += 1
        return (position, gripper_command)
    
    def reset(self):
        """重置索引，从头开始读取"""
        self.current_index = 0
    
    def has_next(self) -> bool:
        """检查是否还有下一个数据"""
        return self.current_index < len(self.positions)
    
    def get_total_steps(self) -> int:
        """获取总的时间步数"""
        return len(self.positions)


class ActionVelocityReader:
    """读取 JSON 文件中的 franka_joints.velocity 和 gripper_command 作为 action 输出"""
    
    def __init__(self, json_path: str = DEFAULT_ACTION_JSON):
        self.json_path = json_path
        self.velocities: List[List[float]] = []
        self.gripper_commands: List[float] = []
        self.current_index = 0
        self._load_data()
    
    def _load_data(self):
        """加载 JSON 文件并提取所有 velocity 和 gripper_command"""
        with open(self.json_path, 'r') as f:
            data = json.load(f)
        
        # 从 data 字段中提取每个时间步的 franka_joints.velocity 和 gripper_command
        for item in data.get('data', []):
            franka_joints = item.get('franka_joints', {})
            velocity = franka_joints.get('velocity')
            gripper_command = item.get('gripper_command', 0.0)
            
            if velocity is not None:
                self.velocities.append(velocity)
                # 如果 gripper_command 是 None，使用默认值 0.0
                if gripper_command is None:
                    gripper_command = 0.0
                self.gripper_commands.append(float(gripper_command))
    
    def get_next(self) -> Optional[Tuple[List[float], float]]:
        """获取下一个时间步的 (velocity, gripper_command)，如果已经读完则返回 None"""
        if self.current_index >= len(self.velocities):
            return None
        velocity = self.velocities[self.current_index]
        gripper_command = self.gripper_commands[self.current_index]
        self.current_index += 1
        return (velocity, gripper_command)
    
    def reset(self):
        """重置索引，从头开始读取"""
        self.current_index = 0
    
    def has_next(self) -> bool:
        """检查是否还有下一个数据"""
        return self.current_index < len(self.velocities)
    
    def get_total_steps(self) -> int:
        """获取总的时间步数"""
        return len(self.velocities)


@dataclass
class RobotState:
    joint_position: np.ndarray
    gripper_position: float


class StateReceiver:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._latest: Optional[RobotState] = None
        self._latest_time = 0.0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self._host, self._port))
                sock.settimeout(1.0)
                buffer = ""
                while True:
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buffer += data.decode("utf-8", errors="ignore")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        state = self._parse_state(payload)
                        if state is None:
                            continue
                        now = time.time()
                        with self._lock:
                            self._latest = state
                            self._latest_time = now
            except Exception:
                time.sleep(1.0)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def _parse_state(self, payload: Dict) -> Optional[RobotState]:
        positions = payload.get("joint_position")
        if not isinstance(positions, list) or len(positions) < 7:
            franka = payload.get("franka_joints") or {}
            positions = franka.get("position")
        if not isinstance(positions, list) or len(positions) < 7:
            return None
        joint_position = np.asarray(positions[:7], dtype=np.float32)
        gripper_position = payload.get("gripper_position")
        if gripper_position is None:
            gripper_position = payload.get("gripper_command")
        if gripper_position is None:
            gripper_position = 0.0
        return RobotState(joint_position=joint_position, gripper_position=float(gripper_position))

    def get_latest(self) -> Tuple[Optional[RobotState], float]:
        with self._lock:
            return self._latest, self._latest_time


class RealSenseCameras:
    def __init__(self, serials, width, height, fps) -> None:
        self._pipelines = []
        self._serials = serials
        for serial in serials:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            pipeline.start(config)
            self._pipelines.append(pipeline)

    def read(self, index: int) -> np.ndarray:
        frames = self._pipelines[index].wait_for_frames()
        color = frames.get_color_frame()
        if color is None:
            raise RuntimeError("No color frame")
        img = np.asanyarray(color.get_data())
        return img[:, :, ::-1]  # BGR to RGB

    def close(self) -> None:
        for pipeline in self._pipelines:
            try:
                pipeline.stop()
            except Exception:
                pass


def send_actions(sock: socket.socket, actions: np.ndarray) -> None:
    payload = {"actions": actions.tolist(), "timestamp": time.time()}
    message = json.dumps(payload) + "\n"
    sock.sendall(message.encode("utf-8"))


def smooth_step(x: float) -> float:
    s = max(0.0, min(1.0, x))
    return s * s * s * (10.0 + s * (-15.0 + s * 6.0))


def clamp_gripper(value: float, binarize: bool = False) -> float:
    g = float(value)
    if binarize:
        g = 1.0 if g > 0.5 else 0.0
    return max(0.0, min(1.0, g))


class GripperDebouncer:
    def __init__(self, steps: int) -> None:
        self._steps = max(1, int(steps))
        self._stable: Optional[float] = None
        self._candidate: Optional[float] = None
        self._count = 0

    def update(self, value: float) -> float:
        raw = 1.0 if value > 0.5 else 0.0
        if self._stable is None:
            self._stable = raw
            self._candidate = raw
            self._count = 0
            return raw
        if raw == self._stable:
            self._candidate = raw
            self._count = 0
            return self._stable
        if self._candidate != raw:
            self._candidate = raw
            self._count = 1
        else:
            self._count += 1
        if self._count >= self._steps:
            self._stable = raw
            self._count = 0
        return self._stable

def joint_velocity_to_delta(joint_velocity: np.ndarray) -> np.ndarray:
    vel = np.asarray(joint_velocity, dtype=np.float32)
    max_norm = float(np.max(np.abs(vel) / RELATIVE_MAX_JOINT_VEL))
    if max_norm > 1.0:
        vel = vel / max_norm
    return vel * MAX_JOINT_DELTA


def velocity_action_to_position(
    joint_position: np.ndarray, joint_velocity: np.ndarray, gripper: float, binarize: bool
) -> np.ndarray:
    delta = joint_velocity_to_delta(joint_velocity)
    target = np.asarray(joint_position, dtype=np.float32) + delta
    gripper_cmd = clamp_gripper(gripper, binarize=binarize)
    return np.concatenate([target, [gripper_cmd]]).astype(np.float32)


def velocity_chunk_to_positions(
    joint_position: np.ndarray, action_chunk: np.ndarray, binarize: bool
) -> np.ndarray:
    target = np.asarray(joint_position, dtype=np.float32).copy()
    positions = []
    for step in action_chunk:
        delta = joint_velocity_to_delta(step[:7])
        target = target + delta
        gripper_cmd = clamp_gripper(step[7], binarize=binarize)
        positions.append(np.concatenate([target, [gripper_cmd]]))
    return np.asarray(positions, dtype=np.float32)

def parse_remote_config(path: str) -> Dict[str, str]:
    result = {}
    try:
        text = Path(path).read_text(encoding="utf-8")
    except Exception:
        return result
    for key in ("REMOTE_HOST", "REMOTE_USER", "REMOTE_PASSWORD"):
        match = re.search(rf"^{key}\s*=\s*\"([^\"]+)\"", text, re.M)
        if match:
            result[key] = match.group(1)
    return result


def check_port(host: str, port: int, timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


class InferenceThread(QtCore.QThread):
    images_signal = QtCore.pyqtSignal(object, object, object, object)
    state_signal = QtCore.pyqtSignal(object, float)
    actions_signal = QtCore.pyqtSignal(object, int, int)
    infer_time_signal = QtCore.pyqtSignal(float)
    status_signal = QtCore.pyqtSignal(dict)
    log_signal = QtCore.pyqtSignal(str)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(
        self,
        policy_host: str,
        policy_port: int,
        action_host: str,
        action_port: int,
        state_host: str,
        state_port: int,
        prompt: str,
        execute_steps: int,
        rate_hz: float,
        image_size: int,
        camera_width: int,
        camera_height: int,
        camera_fps: int,
        serials,
        external_index: int,
        wrist_index: int,
        action_source: str,
        action_json: str,
        blend_s: float,
        gripper_filter_n: int,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._policy_host = policy_host
        self._policy_port = policy_port
        self._action_host = action_host
        self._action_port = action_port
        self._state_host = state_host
        self._state_port = state_port
        self._prompt = prompt
        self._execute_steps = execute_steps
        self._rate_hz = rate_hz
        self._image_size = image_size
        self._camera_width = camera_width
        self._camera_height = camera_height
        self._camera_fps = camera_fps
        self._serials = serials
        self._external_index = external_index
        self._wrist_index = wrist_index
        self._action_source = action_source
        self._action_json = action_json
        self._blend_s = max(0.0, float(blend_s))
        self._gripper_filter_n = max(1, int(gripper_filter_n))
        self._gripper_filter = (
            GripperDebouncer(self._gripper_filter_n)
            if self._gripper_filter_n > 1
            else None
        )
        self._blend_start_time: Optional[float] = None
        self._blend_start_joint: Optional[np.ndarray] = None
        self._blend_start_gripper: Optional[float] = None
        self._stop_event = threading.Event()
        self._last_send_time = 0.0
        self._action_connected = False

    def _filter_gripper_cmd(self, value: float) -> float:
        if self._gripper_filter is None:
            return value
        if 0.1 < value < 0.9:
            return value
        return self._gripper_filter.update(value)

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        state_receiver = StateReceiver(self._state_host, self._state_port)
        cameras = None
        action_sock = None
        policy = None
        reader = None
        try:
            if self._action_source == "policy":
                cameras = RealSenseCameras(
                    self._serials,
                    self._camera_width,
                    self._camera_height,
                    self._camera_fps,
                )
                policy = websocket_client_policy.WebsocketClientPolicy(
                    host=self._policy_host, port=self._policy_port
                )
            elif self._action_source == "position_json":
                reader = ActionPositionReader(self._action_json)
            elif self._action_source == "velocity_json":
                reader = ActionVelocityReader(self._action_json)
        except Exception as exc:
            self.error_signal.emit(f"Init failed: {exc}")
            return

        action_chunk = None
        steps_in_chunk = 0
        current_chunk_steps = max(1, self._execute_steps)
        dt = 1.0 / max(1e-6, self._rate_hz)
        next_time = time.time()
        last_step_sent = None

        while not self._stop_event.is_set():
            state, state_time = state_receiver.get_latest()
            if state is None:
                time.sleep(0.02)
                continue

            if self._blend_s > 0.0 and self._blend_start_time is None:
                self._blend_start_time = time.time()
                self._blend_start_joint = state.joint_position.copy()
                self._blend_start_gripper = float(state.gripper_position)

            now = time.time()
            self.state_signal.emit(state.joint_position, state.gripper_position)

            if self._action_source != "policy":
                action = None
                if action_sock is None:
                    action_sock = self._connect_action_sock()
                if action_sock is not None:
                    try:
                        result = reader.get_next() if reader is not None else None
                        if result is None:
                            self._action_connected = False
                            self.error_signal.emit("Action JSON exhausted.")
                            self._stop_event.set()
                            continue
                        if self._action_source == "position_json":
                            position, gripper_command = result
                            gripper_cmd = clamp_gripper(gripper_command, binarize=False)
                            gripper_cmd = self._filter_gripper_cmd(gripper_cmd)
                            action = np.array(
                                position + [gripper_cmd], dtype=np.float32
                            ).reshape(1, -1)
                        else:
                            velocity, gripper_command = result
                            action = velocity_action_to_position(
                                state.joint_position,
                                velocity,
                                gripper_command,
                                binarize=False,
                            ).reshape(1, -1)
                            action[0, 7] = self._filter_gripper_cmd(action[0, 7])
                        if (
                            self._blend_start_time is not None
                            and self._blend_start_joint is not None
                            and self._blend_start_gripper is not None
                        ):
                            elapsed = time.time() - self._blend_start_time
                            if elapsed < self._blend_s:
                                alpha = smooth_step(elapsed / self._blend_s)
                                action[0, :7] = (
                                    self._blend_start_joint
                                    + (action[0, :7] - self._blend_start_joint) * alpha
                                )
                                action[0, 7] = (
                                    self._blend_start_gripper
                                    + (action[0, 7] - self._blend_start_gripper) * alpha
                                )
                        send_actions(action_sock, action)
                        self._last_send_time = time.time()
                        self._action_connected = True
                    except Exception as exc:
                        self._action_connected = False
                        self.error_signal.emit(f"Action send failed: {exc}")
                        try:
                            action_sock.close()
                        except Exception:
                            pass
                        action_sock = None

                if action is not None:
                    self.actions_signal.emit(action, 1, 0)

                status = {
                    "action_connected": self._action_connected,
                    "state_age": max(0.0, now - state_time),
                    "last_send_age": max(0.0, now - self._last_send_time)
                    if self._last_send_time > 0
                    else None,
                }
                self.status_signal.emit(status)

                next_time += dt
                time.sleep(max(0.0, next_time - time.time()))
                continue

            need_infer = action_chunk is None or steps_in_chunk >= current_chunk_steps
            if need_infer:
                try:
                    ext_raw = cameras.read(self._external_index)
                    wrist_raw = cameras.read(self._wrist_index)
                except Exception as exc:
                    self.error_signal.emit(f"Camera read failed: {exc}")
                    time.sleep(0.1)
                    continue

                ext_model = image_tools.resize_with_pad(
                    ext_raw, self._image_size, self._image_size
                )
                wrist_model = image_tools.resize_with_pad(
                    wrist_raw, self._image_size, self._image_size
                )

                obs = {
                    "observation/exterior_image_1_left": ext_model,
                    "observation/wrist_image_left": wrist_model,
                    "observation/joint_position": state.joint_position,
                    "observation/gripper_position": np.array(
                        [state.gripper_position], dtype=np.float32
                    ),
                    "prompt": self._prompt,
                }

                try:
                    infer_start = time.perf_counter()
                    result = policy.infer(obs)
                    infer_ms = (time.perf_counter() - infer_start) * 1000.0
                except Exception as exc:
                    self.error_signal.emit(f"Inference failed: {exc}")
                    time.sleep(0.1)
                    continue

                action_chunk = np.asarray(result.get("actions"), dtype=np.float32)
                if action_chunk.ndim != 2 or action_chunk.shape[1] < 8:
                    self.error_signal.emit("Invalid action chunk shape")
                    action_chunk = None
                    time.sleep(0.1)
                    continue

                current_chunk_steps = min(self._execute_steps, action_chunk.shape[0])
                steps_in_chunk = 0
                last_step_sent = None
                action_chunk = velocity_chunk_to_positions(
                    state.joint_position,
                    action_chunk[:current_chunk_steps],
                    binarize=True,
                )
                if self._gripper_filter is not None:
                    for step in range(current_chunk_steps):
                        action_chunk[step, 7] = self._filter_gripper_cmd(
                            action_chunk[step, 7]
                        )
                current_chunk_steps = action_chunk.shape[0]
                if (
                    self._blend_start_time is not None
                    and self._blend_start_joint is not None
                    and self._blend_start_gripper is not None
                    and self._blend_s > 0.0
                ):
                    elapsed = time.time() - self._blend_start_time
                    for step in range(current_chunk_steps):
                        t = elapsed + step * dt
                        if t >= self._blend_s:
                            continue
                        alpha = smooth_step(t / self._blend_s)
                        action_chunk[step, :7] = (
                            self._blend_start_joint
                            + (action_chunk[step, :7] - self._blend_start_joint) * alpha
                        )
                        action_chunk[step, 7] = (
                            self._blend_start_gripper
                            + (action_chunk[step, 7] - self._blend_start_gripper) * alpha
                        )

                if action_sock is None:
                    action_sock = self._connect_action_sock()

                if action_sock is not None:
                    try:
                        send_actions(action_sock, action_chunk[:current_chunk_steps])

                        self._last_send_time = time.time()
                        self._action_connected = True
                    except Exception as exc:
                        self._action_connected = False
                        self.error_signal.emit(f"Action send failed: {exc}")
                        try:
                            action_sock.close()
                        except Exception:
                            pass
                        action_sock = None

                self.images_signal.emit(ext_raw, wrist_raw, ext_model, wrist_model)
                self.actions_signal.emit(action_chunk, current_chunk_steps, 0)
                self.infer_time_signal.emit(infer_ms)
                last_step_sent = 0
            else:
                current_step = min(steps_in_chunk, current_chunk_steps - 1)
                if last_step_sent != current_step:
                    self.actions_signal.emit(action_chunk, current_chunk_steps, current_step)
                    last_step_sent = current_step

            status = {
                "action_connected": self._action_connected,
                "state_age": max(0.0, now - state_time),
                "last_send_age": max(0.0, now - self._last_send_time)
                if self._last_send_time > 0
                else None,
            }
            self.status_signal.emit(status)

            steps_in_chunk += 1
            next_time += dt
            time.sleep(max(0.0, next_time - time.time()))

        try:
            if action_sock is not None:
                action_sock.close()
        except Exception:
            pass
        if cameras is not None:
            cameras.close()

    def _connect_action_sock(self) -> Optional[socket.socket]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self._action_host, self._action_port))
            return sock
        except Exception as exc:
            self.error_signal.emit(f"Action connect failed: {exc}")
            return None


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args) -> None:
        super().__init__()
        self.setWindowTitle("OpenPI Droid GUI")
        self._args = args
        self._worker: Optional[InferenceThread] = None
        self._server_process = None
        self._remote = self._load_remote_config()
        self._policy_ok: Optional[bool] = None
        self._last_status: Dict = {}
        self._auto_start = args.auto_start
        self._auto_start_delay = args.auto_start_delay
        self._auto_stop_after = args.auto_stop_after
        self._action_source = args.action_source
        self._action_json = args.action_json
        self._blend_s = args.blend_s
        self._invert_gripper = bool(args.invert_gripper)
        self._max_joint_delta = float(args.max_joint_delta)
        self._gripper_filter_n = int(args.gripper_filter_n)

        self._init_ui()
        self._init_status_timer()
        if self._auto_start:
            QtCore.QTimer.singleShot(self._auto_start_delay, self._on_start)
        if self._auto_stop_after > 0:
            QtCore.QTimer.singleShot(self._auto_stop_after * 1000, self.close)

    def _load_remote_config(self) -> Dict[str, str]:
        remote = {
            "REMOTE_HOST": os.environ.get("OPENPI_REMOTE_HOST", ""),
            "REMOTE_USER": os.environ.get("OPENPI_REMOTE_USER", ""),
            "REMOTE_PASSWORD": os.environ.get("OPENPI_REMOTE_PASSWORD", ""),
        }
        if not all(remote.values()):
            remote.update(parse_remote_config(CONFIG_FILE))
        return remote

    def _init_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # Image grid
        image_grid = QtWidgets.QGridLayout()
        self.ext_raw_label = self._make_image_panel("External Raw", 320, 240)
        self.wrist_raw_label = self._make_image_panel("Wrist Raw", 320, 240)
        self.ext_model_label = self._make_image_panel("External Model", 224, 224)
        self.wrist_model_label = self._make_image_panel("Wrist Model", 224, 224)

        image_grid.addWidget(self.ext_raw_label, 0, 0)
        image_grid.addWidget(self.wrist_raw_label, 0, 1)
        image_grid.addWidget(self.ext_model_label, 1, 0)
        image_grid.addWidget(self.wrist_model_label, 1, 1)

        main_layout.addLayout(image_grid, stretch=2)

        # Right panel
        right_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=1)

        controls = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        controls.addWidget(self.start_button)
        controls.addWidget(self.stop_button)
        right_panel.addLayout(controls)

        prompt_layout = QtWidgets.QHBoxLayout()
        prompt_layout.addWidget(QtWidgets.QLabel("Prompt:"))
        self.prompt_edit = QtWidgets.QLineEdit(self._args.prompt)
        prompt_layout.addWidget(self.prompt_edit)
        right_panel.addLayout(prompt_layout)

        source_layout = QtWidgets.QHBoxLayout()
        source_layout.addWidget(QtWidgets.QLabel("Action source:"))
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItem("Policy", "policy")
        self.source_combo.addItem("Position JSON", "position_json")
        self.source_combo.addItem("Velocity JSON", "velocity_json")
        current_index = self.source_combo.findData(self._action_source)
        if current_index >= 0:
            self.source_combo.setCurrentIndex(current_index)
        else:
            self._action_source = self.source_combo.currentData()
        self.source_combo.currentIndexChanged.connect(self._on_action_source_changed)
        source_layout.addWidget(self.source_combo)
        right_panel.addLayout(source_layout)

        json_layout = QtWidgets.QHBoxLayout()
        json_layout.addWidget(QtWidgets.QLabel("Action JSON:"))
        self.action_json_edit = QtWidgets.QLineEdit(self._action_json)
        json_layout.addWidget(self.action_json_edit)
        self.action_json_button = QtWidgets.QPushButton("Browse")
        self.action_json_button.clicked.connect(self._browse_action_json)
        json_layout.addWidget(self.action_json_button)
        right_panel.addLayout(json_layout)

        invert_layout = QtWidgets.QHBoxLayout()
        self.invert_check = QtWidgets.QCheckBox("Invert gripper")
        self.invert_check.setChecked(self._invert_gripper)
        invert_layout.addWidget(self.invert_check)
        right_panel.addLayout(invert_layout)

        gripper_filter_layout = QtWidgets.QHBoxLayout()
        gripper_filter_layout.addWidget(QtWidgets.QLabel("Gripper filter N:"))
        self.gripper_filter_spin = QtWidgets.QSpinBox()
        self.gripper_filter_spin.setMinimum(1)
        self.gripper_filter_spin.setMaximum(10)
        self.gripper_filter_spin.setValue(self._gripper_filter_n)
        gripper_filter_layout.addWidget(self.gripper_filter_spin)
        right_panel.addLayout(gripper_filter_layout)

        delta_layout = QtWidgets.QHBoxLayout()
        delta_layout.addWidget(QtWidgets.QLabel("Max joint delta:"))
        self.delta_spin = QtWidgets.QDoubleSpinBox()
        self.delta_spin.setMinimum(0.01)
        self.delta_spin.setMaximum(0.3)
        self.delta_spin.setSingleStep(0.01)
        self.delta_spin.setDecimals(3)
        self.delta_spin.setValue(self._max_joint_delta)
        delta_layout.addWidget(self.delta_spin)
        right_panel.addLayout(delta_layout)

        exec_layout = QtWidgets.QHBoxLayout()
        exec_layout.addWidget(QtWidgets.QLabel("Execute steps:"))
        self.exec_spin = QtWidgets.QSpinBox()
        self.exec_spin.setMinimum(1)
        self.exec_spin.setMaximum(15)
        self.exec_spin.setValue(self._args.execute_steps)
        exec_layout.addWidget(self.exec_spin)
        right_panel.addLayout(exec_layout)

        self.infer_label = QtWidgets.QLabel("Inference: -- ms")
        right_panel.addWidget(self.infer_label)

        self.state_label = QtWidgets.QLabel("Joint: --")
        self.gripper_label = QtWidgets.QLabel("Gripper: --")
        right_panel.addWidget(self.state_label)
        right_panel.addWidget(self.gripper_label)

        self.status_label = QtWidgets.QLabel("Status: --")
        right_panel.addWidget(self.status_label)

        self.actions_table = QtWidgets.QTableWidget(15, 8)
        self.actions_table.setHorizontalHeaderLabels([f"a{i}" for i in range(8)])
        self.actions_table.setVerticalHeaderLabels([str(i) for i in range(15)])
        self.actions_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.actions_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        right_panel.addWidget(self.actions_table, stretch=1)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        right_panel.addWidget(self.log_box, stretch=1)

        self.start_button.clicked.connect(self._on_start)
        self.stop_button.clicked.connect(self._on_stop)
        self._update_action_source_controls()

    def _make_image_panel(self, title: str, width: int, height: int) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel()
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setFixedSize(width, height)
        label.setStyleSheet("border: 1px solid #444;")
        label.setText(title)
        return label

    def _init_status_timer(self) -> None:
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(1500)
        self._status_timer.timeout.connect(self._update_policy_status)
        self._status_timer.start()

    def _update_policy_status(self) -> None:
        if self._action_source != "policy":
            self._policy_ok = None
            self._render_status()
            return
        self._policy_ok = check_port(self._args.policy_host, self._args.policy_port)
        self._render_status()

    def _render_status(self) -> None:
        parts = []
        if self._policy_ok is not None:
            parts.append("Policy: OK" if self._policy_ok else "Policy: DOWN")
        action_ok = self._last_status.get("action_connected")
        state_age = self._last_status.get("state_age")
        last_send_age = self._last_status.get("last_send_age")
        if action_ok is not None:
            parts.append("Action: OK" if action_ok else "Action: DOWN")
        if state_age is not None:
            parts.append(f"State age: {state_age * 1000.0:.0f} ms")
        if last_send_age is not None:
            parts.append(f"Last send: {last_send_age * 1000.0:.0f} ms")
        if not parts:
            parts.append("--")
        self.status_label.setText("Status: " + ", ".join(parts))

    def _log(self, msg: str) -> None:
        self.log_box.appendPlainText(msg)

    def _log_threadsafe(self, msg: str) -> None:
        QtCore.QTimer.singleShot(0, lambda m=msg: self._log(m))

    def _browse_action_json(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select action JSON",
            self.action_json_edit.text().strip(),
            "JSON Files (*.json);;All Files (*)",
        )
        if path:
            self.action_json_edit.setText(path)

    def _update_action_source_controls(self) -> None:
        source = self.source_combo.currentData()
        use_json = source in ("position_json", "velocity_json")
        self.action_json_edit.setEnabled(use_json)
        self.action_json_button.setEnabled(use_json)
        self.delta_spin.setEnabled(source in ("policy", "velocity_json"))

    def _on_action_source_changed(self, _index: int) -> None:
        source = self.source_combo.currentData()
        if source:
            self._action_source = source
            self._update_action_source_controls()
            self._update_policy_status()

    def _ensure_policy_server(self) -> None:
        if check_port(self._args.policy_host, self._args.policy_port):
            self._log_threadsafe("Policy server already running.")
            return

        self._log_threadsafe("Starting policy server...")
        env = os.environ.copy()
        env["OPENPI_CONDA_ENV"] = os.environ.get("OPENPI_SERVER_ENV", "openpi")
        env["OPENPI_PORT"] = str(self._args.policy_port)
        script = "/home/ubuntu/openpi/scripts/start_pi05_droid_server.sh"
        self._server_process = subprocess.Popen(
            [script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env
        )

    def _build_ssh(self) -> Optional[list]:
        host = self._remote.get("REMOTE_HOST")
        user = self._remote.get("REMOTE_USER")
        password = self._remote.get("REMOTE_PASSWORD")
        if not host or not user:
            self._log_threadsafe("Remote host/user not configured.")
            return None
        base = ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{host}"]
        if password:
            if not shutil.which("sshpass"):
                self._log_threadsafe("sshpass not available for password auth.")
                return None
            base = ["sshpass", "-p", password] + base
        return base

    def _start_remote(self) -> None:
        ssh_base = self._build_ssh()
        if ssh_base is None:
            return
        cmd = [
            f"ROBOTIQ_PORT={self._args.robotiq_port}",
            f"INVERT_GRIPPER={int(self._invert_gripper)}",
            f"ACTION_PORT={self._args.action_port}",
            f"STATE_PORT={self._args.state_port}",
            f"VEL_SCALE={self._args.vel_scale}",
            f"MAX_ACC={self._args.max_acc}",
            f"VEL_ALPHA={self._args.vel_alpha}",
            f"TIMEOUT_S={self._args.timeout_s}",
            #"/home/rsj/franka_cpp_control/start_openpi_stream.sh",
            #"/home/rsj/franka_cpp_control/start_openpi_stream_trackjson.sh",
            "/home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh",
            "start",
        ]
        subprocess.run(ssh_base + [" ".join(cmd)], check=False)

    def _stop_remote(self) -> None:
        ssh_base = self._build_ssh()
        if ssh_base is None:
            return
        cmd = ["/home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh", "stop"]
        subprocess.run(ssh_base + [" ".join(cmd)], check=False)

    def _on_start(self) -> None:
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.source_combo.setEnabled(False)
        self.action_json_edit.setEnabled(False)
        self.action_json_button.setEnabled(False)
        self.invert_check.setEnabled(False)
        self.delta_spin.setEnabled(False)
        self.gripper_filter_spin.setEnabled(False)
        self._action_source = self.source_combo.currentData()
        self._action_json = self.action_json_edit.text().strip()
        self._invert_gripper = bool(self.invert_check.isChecked())
        self._max_joint_delta = float(self.delta_spin.value())
        self._gripper_filter_n = int(self.gripper_filter_spin.value())
        set_max_joint_delta(self._max_joint_delta)
        if self._action_source in ("position_json", "velocity_json"):
            if not self._action_json or not Path(self._action_json).is_file():
                self._log("Action JSON file not found.")
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                self.source_combo.setEnabled(True)
                self.invert_check.setEnabled(True)
                self._update_action_source_controls()
                return
        self._log("Starting system...")

        def _start():
            try:
                if self._action_source == "policy":
                    self._ensure_policy_server()
                self._start_remote()
                self._start_worker()
            except Exception as exc:
                self._log_threadsafe(f"Start failed: {exc}")

        threading.Thread(target=_start, daemon=True).start()

    def _start_worker(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            return
        self._worker = InferenceThread(
            policy_host=self._args.policy_host,
            policy_port=self._args.policy_port,
            action_host=self._args.action_host,
            action_port=self._args.action_port,
            state_host=self._args.state_host,
            state_port=self._args.state_port,
            prompt=self.prompt_edit.text().strip() or self._args.prompt,
            execute_steps=self.exec_spin.value(),
            rate_hz=self._args.rate,
            image_size=self._args.image_size,
            camera_width=self._args.camera_width,
            camera_height=self._args.camera_height,
            camera_fps=self._args.camera_fps,
            serials=self._args.serials,
            external_index=self._args.external_index,
            wrist_index=self._args.wrist_index,
            action_source=self._action_source,
            action_json=self._action_json,
            blend_s=self._blend_s,
            gripper_filter_n=self._gripper_filter_n,
        )
        self._worker.images_signal.connect(self._update_images)
        self._worker.state_signal.connect(self._update_state)
        self._worker.actions_signal.connect(self._update_actions)
        self._worker.infer_time_signal.connect(self._update_infer_time)
        self._worker.status_signal.connect(self._update_status)
        self._worker.error_signal.connect(self._log)
        self._worker.log_signal.connect(self._log)
        self._worker.start()

    def _on_stop(self) -> None:
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.source_combo.setEnabled(True)
        self.invert_check.setEnabled(True)
        self.gripper_filter_spin.setEnabled(True)
        self._update_action_source_controls()
        self._log("Stopping...")
        self._stop_worker()
        self._stop_remote()

    def _stop_worker(self) -> None:
        if self._worker is None:
            return
        self._worker.stop()
        self._worker.wait(3000)
        self._worker = None

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._stop_worker()
        self._stop_remote()
        super().closeEvent(event)

    def _update_images(self, ext_raw, wrist_raw, ext_model, wrist_model) -> None:
        self._set_image(self.ext_raw_label, ext_raw)
        self._set_image(self.wrist_raw_label, wrist_raw)
        self._set_image(self.ext_model_label, ext_model)
        self._set_image(self.wrist_model_label, wrist_model)

    def _set_image(self, label: QtWidgets.QLabel, image: np.ndarray) -> None:
        if image is None:
            return
        image = np.ascontiguousarray(image)
        h, w, ch = image.shape
        qimg = QtGui.QImage(image.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg.copy())
        label.setPixmap(pix.scaled(label.size(), QtCore.Qt.KeepAspectRatio))

    def _update_state(self, joint_position, gripper) -> None:
        joint_str = ", ".join(f"{v:.3f}" for v in joint_position)
        self.state_label.setText(f"Joint: [{joint_str}]")
        self.gripper_label.setText(f"Gripper: {gripper:.3f}")

    def _update_actions(self, actions, execute_steps: int, current_step: int) -> None:
        actions = np.asarray(actions)
        rows = min(15, actions.shape[0])
        cols = min(8, actions.shape[1])
        for r in range(15):
            for c in range(8):
                item = self.actions_table.item(r, c)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.actions_table.setItem(r, c, item)
                if r < rows and c < cols:
                    item.setText(f"{actions[r, c]:.3f}")
                else:
                    item.setText("")

                color = QtGui.QColor(255, 255, 255)
                if r < execute_steps:
                    color = QtGui.QColor(210, 255, 210)
                if r == current_step:
                    color = QtGui.QColor(255, 245, 180)
                item.setBackground(color)

    def _update_infer_time(self, infer_ms: float) -> None:
        self.infer_label.setText(f"Inference: {infer_ms:.2f} ms")

    def _update_status(self, status: Dict) -> None:
        self._last_status = status
        self._render_status()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy-host", default="127.0.0.1")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--state-host", default="172.16.1.2")
    parser.add_argument("--state-port", type=int, default=15124)
    parser.add_argument("--action-host", default="172.16.1.2")
    parser.add_argument("--action-port", type=int, default=15123)
    parser.add_argument("--prompt", default="Pick up the yellow object and put it on the white box.")
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--execute-steps", type=int, default=1)
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--serials", nargs="*", default=DEFAULT_SERIALS)
    parser.add_argument("--external-index", type=int, default=2)
    parser.add_argument("--wrist-index", type=int, default=1)
    parser.add_argument(
        "--action-source",
        choices=["policy", "position_json", "velocity_json"],
        default="policy",
        help="Choose policy inference or JSON replay for actions.",
    )
    parser.add_argument(
        "--action-json",
        default=DEFAULT_ACTION_JSON,
        help="Path to action JSON used for position/velocity replay.",
    )
    parser.add_argument(
        "--blend-s",
        type=float,
        default=0.0,
        help="Blend duration in seconds from current state to targets.",
    )

    parser.add_argument("--robotiq-port", default="/dev/robotiq")
    parser.add_argument("--invert-gripper", type=int, default=1)
    parser.add_argument("--max-joint-delta", type=float, default=0.05)
    parser.add_argument("--gripper-filter-n", type=int, default=3)
    parser.add_argument("--vel-scale", type=float, default=0.1)
    parser.add_argument("--max-acc", type=float, default=2.0)
    parser.add_argument("--vel-alpha", type=float, default=0.95)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--auto-start", action="store_true", help="Auto start after GUI loads.")
    parser.add_argument("--auto-start-delay", type=int, default=1000)
    parser.add_argument("--auto-stop-after", type=int, default=0)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = QtWidgets.QApplication([])
    window = MainWindow(args)
    window.resize(1200, 800)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
