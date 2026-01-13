#!/usr/bin/env python3
import argparse
import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


@dataclass
class ActionChunk:
    actions: np.ndarray
    index: int
    timestamp: float


class ActionServer(Node):
    def __init__(
        self,
        host: str,
        port: int,
        rate_hz: float,
        vel_scale: float,
        timeout_s: float,
        invert_gripper: bool,
    ) -> None:
        super().__init__(openpi_action_server)
        self._rate_hz = rate_hz
        self._dt = 1.0 / rate_hz
        self._vel_scale = vel_scale
        self._timeout_s = timeout_s
        self._invert_gripper = invert_gripper

        self._joint_state: Optional[JointState] = None
        self._joint_state_lock = threading.Lock()

        self._chunk: Optional[ActionChunk] = None
        self._chunk_lock = threading.Lock()

        self._joint_pub = self.create_publisher(JointState, /gello/joint_states, 10)
        self._gripper_pub = self.create_publisher(Float32, /gripper_client/target_gripper_width_percent, 10)
        self.create_subscription(JointState, /franka/joint_states, self._joint_state_cb, 10)

        self._server_thread = threading.Thread(target=self._run_server, args=(host, port), daemon=True)
        self._server_thread.start()

        self.create_timer(self._dt, self._control_step)
        self.get_logger().info(
            fAction server listening on {host}:{port}, rate={rate_hz}Hz, vel_scale={vel_scale}
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._joint_state = msg

    def _set_chunk(self, actions: List[List[float]]) -> None:
        arr = np.asarray(actions, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] < 8:
            self.get_logger().warn(fInvalid action chunk shape: {arr.shape})
            return
        with self._chunk_lock:
            self._chunk = ActionChunk(actions=arr, index=0, timestamp=time.time())

    def _pop_action(self) -> Optional[np.ndarray]:
        with self._chunk_lock:
            if self._chunk is None:
                return None
            if time.time() - self._chunk.timestamp > self._timeout_s:
                self._chunk = None
                return None
            if self._chunk.index >= len(self._chunk.actions):
                return None
            action = self._chunk.actions[self._chunk.index]
            self._chunk.index += 1
            return action

    def _control_step(self) -> None:
        with self._joint_state_lock:
            joint_state = self._joint_state
        if joint_state is None or not joint_state.position:
            return

        action = self._pop_action()
        if action is None:
            return

        q_current = np.asarray(joint_state.position[:7], dtype=np.float32)
        v = np.asarray(action[:7], dtype=np.float32)
        v = np.clip(v, -1.0, 1.0) * self._vel_scale
        q_target = q_current + v * self._dt

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(joint_state.name[:7]) if joint_state.name else [fjoint{i+1} for i in range(7)]
        cmd.position = q_target.tolist()
        self._joint_pub.publish(cmd)

        grip = float(action[7])
        grip = max(0.0, min(1.0, grip))
        if self._invert_gripper:
            grip = 1.0 - grip
        self._gripper_pub.publish(Float32(data=grip))

    def _run_server(self, host: str, port: int) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)

        while rclpy.ok():
            conn, addr = server.accept()
            self.get_logger().info(fClient connected from {addr})
            try:
                conn.settimeout(1.0)
                buffer = 
                while rclpy.ok():
                    try:
                        data = conn.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buffer += data.decode(utf-8, errors=ignore)
                    while n in buffer:
                        line, buffer = buffer.split(n, 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line)
                        except json.JSONDecodeError:
                            self.get_logger().warn(Failed to parse JSON payload)
                            continue
                        actions = payload.get(actions)
                        if actions is None:
                            continue
                        self._set_chunk(actions)
            finally:
                conn.close()
                self.get_logger().info(Client disconnected)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(--host, default=0.0.0.0)
    parser.add_argument(--port, type=int, default=15123)
    parser.add_argument(--rate, type=float, default=15.0)
    parser.add_argument(--vel-scale, type=float, default=0.2)
    parser.add_argument(--timeout, type=float, default=2.0)
    parser.add_argument(--invert-gripper, action=store_true)
    args = parser.parse_args()

    rclpy.init()
    node = ActionServer(
        host=args.host,
        port=args.port,
        rate_hz=args.rate,
        vel_scale=args.vel_scale,
        timeout_s=args.timeout,
        invert_gripper=args.invert_gripper,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == __main__:
    main()
