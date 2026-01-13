import argparse
import json
import re
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


CONFIG_FILE = "/home/ubuntu/take_data/take_video_action/video_action_recorder.py"
STATE_PROXY_SCRIPT = "/home/ubuntu/take_data/take_video_action/state_proxy.py"


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


def parse_state(payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    positions = payload.get("joint_position")
    if not isinstance(positions, list) or len(positions) < 7:
        franka = payload.get("franka_joints") or {}
        positions = franka.get("position")
    if not isinstance(positions, list) or len(positions) < 7:
        return None
    gripper = payload.get("gripper_position")
    if gripper is None:
        gripper = payload.get("gripper_command")
    if gripper is None:
        gripper = 0.0
    return {
        "joint_position": np.asarray(positions[:7], dtype=np.float32),
        "gripper_position": float(gripper),
        "raw": payload,
    }


class RemoteStateClient:
    def __init__(
        self,
        remote_host: Optional[str] = None,
        remote_user: Optional[str] = None,
        remote_password: Optional[str] = None,
        state_host: str = "172.16.1.2",
        state_port: int = 15124,
        action_host: str = "172.16.1.2",
        action_port: int = 15123,
        proxy_host: str = "127.0.0.1",
        proxy_port: int = 15125,
        use_proxy: bool = True,
        auto_start: bool = True,
        auto_start_remote: bool = True,
        auto_start_proxy: bool = True,
        rate_hz: int = 15,
        k_gain: int = 20,
        d_gain: int = 4,
        action_interp: int = 1,
        invert_gripper: int = 1,
        remote_script: str = "/home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh",
    ) -> None:
        cfg = parse_remote_config(CONFIG_FILE)
        self._remote_host = remote_host or cfg.get("REMOTE_HOST") or state_host
        self._remote_user = remote_user or cfg.get("REMOTE_USER") or "rsj"
        self._remote_password = remote_password or cfg.get("REMOTE_PASSWORD")
        self._state_host = state_host
        self._state_port = state_port
        self._action_host = action_host
        self._action_port = action_port
        self._proxy_host = proxy_host
        self._proxy_port = proxy_port
        self._use_proxy = use_proxy
        self._auto_start_remote = auto_start_remote
        self._auto_start_proxy = auto_start_proxy
        self._rate_hz = rate_hz
        self._k_gain = k_gain
        self._d_gain = d_gain
        self._action_interp = action_interp
        self._invert_gripper = invert_gripper
        self._remote_script = remote_script
        self._proxy_process: Optional[subprocess.Popen] = None
        if auto_start:
            self.start()

    def _build_ssh(self) -> Optional[list]:
        if not self._remote_host or not self._remote_user:
            return None
        base = ["ssh", "-o", "StrictHostKeyChecking=no", f"{self._remote_user}@{self._remote_host}"]
        if self._remote_password:
            if not shutil.which("sshpass"):
                return None
            base = ["sshpass", "-p", self._remote_password] + base
        return base

    def start(self) -> None:
        if self._auto_start_remote:
            self.start_remote()
        if self._auto_start_proxy and self._use_proxy:
            self.start_proxy()

    def start_remote(self) -> None:
        ssh_base = self._build_ssh()
        if ssh_base is None:
            return
        cmd = (
            f"RATE_HZ={self._rate_hz} K_GAIN={self._k_gain} "
            f"D_GAIN={self._d_gain} ACTION_INTERP={self._action_interp} "
            f"INVERT_GRIPPER={self._invert_gripper} {self._remote_script} start"
        )
        subprocess.run(ssh_base + [cmd], check=False)

    def start_proxy(self) -> None:
        if check_port(self._proxy_host, self._proxy_port):
            return
        cmd = [
            sys.executable,
            STATE_PROXY_SCRIPT,
            "--remote-host",
            self._state_host,
            "--remote-port",
            str(self._state_port),
            "--listen-host",
            self._proxy_host,
            "--listen-port",
            str(self._proxy_port),
        ]
        self._proxy_process = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def get_state(self, timeout_s: float = 2.0) -> Optional[Dict[str, Any]]:
        host = self._proxy_host if self._use_proxy else self._state_host
        port = self._proxy_port if self._use_proxy else self._state_port
        sock = socket.create_connection((host, port), timeout=2.0)
        sock.settimeout(0.2)
        buffer = ""
        deadline = time.time() + timeout_s
        try:
            while time.time() < deadline:
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
                    state = parse_state(payload)
                    if state is not None:
                        return state
        finally:
            sock.close()
        return None

    def send_action(self, actions: np.ndarray) -> None:
        payload = {"actions": actions.tolist(), "timestamp": time.time()}
        message = json.dumps(payload) + "\n"
        sock = socket.create_connection((self._action_host, self._action_port), timeout=2.0)
        try:
            sock.sendall(message.encode("utf-8"))
        finally:
            sock.close()

    def close(self) -> None:
        if self._proxy_process is not None:
            self._proxy_process.terminate()
            self._proxy_process = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=2.0)
    args = parser.parse_args()
    with RemoteStateClient() as client:
        state = client.get_state(timeout_s=args.timeout_s)
    if state is None:
        print("No state received.")
    else:
        print("joint_position:", state["joint_position"])
        print("gripper_position:", state["gripper_position"])


if __name__ == "__main__":
    main()
