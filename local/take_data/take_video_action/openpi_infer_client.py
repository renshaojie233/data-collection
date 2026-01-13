#!/usr/bin/env python3
import argparse
import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import pyrealsense2 as rs
from openpi_client import image_tools
from openpi_client import websocket_client_policy


DEFAULT_SERIALS = [
    "243722070232",
    "243622073040",
    "243722073691",
]


@dataclass
class RobotState:
    joint_position: np.ndarray
    gripper_position: float


class StateReceiver:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._latest: Optional[RobotState] = None
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
                        with self._lock:
                            self._latest = state
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

    def get_latest(self) -> Optional[RobotState]:
        with self._lock:
            return self._latest


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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy-host", default="127.0.0.1")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--state-host", default="172.16.1.2")
    parser.add_argument("--state-port", type=int, default=15124)
    parser.add_argument("--action-host", default="172.16.1.2")
    parser.add_argument("--action-port", type=int, default=15123)
    parser.add_argument("--prompt", default="pick up the apple")
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--open-loop-horizon", type=int, default=15)
    parser.add_argument(
        "--execute-steps",
        type=int,
        default=1,
        help="Only execute the first N actions from each chunk (default: 1).",
    )
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--serials", nargs="*", default=DEFAULT_SERIALS)
    parser.add_argument("--external-index", type=int, default=2)
    parser.add_argument("--wrist-index", type=int, default=1)
    args = parser.parse_args()
    execute_steps = args.execute_steps
    if execute_steps <= 0:
        raise ValueError("execute-steps must be >= 1")

    state_receiver = StateReceiver(args.state_host, args.state_port)
    cameras = RealSenseCameras(args.serials, args.camera_width, args.camera_height, args.camera_fps)

    policy = websocket_client_policy.WebsocketClientPolicy(host=args.policy_host, port=args.policy_port)

    action_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    action_sock.connect((args.action_host, args.action_port))

    action_chunk = None
    steps_in_chunk = 0
    current_chunk_steps = execute_steps
    dt = 1.0 / args.rate
    next_time = time.time()

    try:
        while True:
            state = state_receiver.get_latest()
            if state is None:
                time.sleep(0.05)
                continue

            if action_chunk is None or steps_in_chunk >= current_chunk_steps:
                ext_img = cameras.read(args.external_index)
                wrist_img = cameras.read(args.wrist_index)

                obs = {
                    "observation/exterior_image_1_left": image_tools.resize_with_pad(
                        ext_img, args.image_size, args.image_size
                    ),
                    "observation/wrist_image_left": image_tools.resize_with_pad(
                        wrist_img, args.image_size, args.image_size
                    ),
                    "observation/joint_position": state.joint_position,
                    "observation/gripper_position": np.array([state.gripper_position], dtype=np.float32),
                    "prompt": args.prompt,
                }

                infer_start = time.perf_counter()
                result = policy.infer(obs)
                infer_ms = (time.perf_counter() - infer_start) * 1000.0
                action_chunk = np.asarray(result["actions"], dtype=np.float32)
                current_chunk_steps = min(execute_steps, action_chunk.shape[0])
                steps_in_chunk = 0
                #send_actions(action_sock, action_chunk.sum(axis=0, keepdims=True))
                send_actions(action_sock, action_chunk[:current_chunk_steps])
                print(f"infer_time_ms={infer_ms:.2f}, steps={current_chunk_steps}")

            steps_in_chunk += 1
            next_time += dt
            time.sleep(max(0.0, next_time - time.time()))
    finally:
        cameras.close()
        action_sock.close()


if __name__ == "__main__":
    main()
