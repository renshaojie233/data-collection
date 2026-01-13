import json
import math
import re
import shutil
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional


CONFIG_FILE = "/home/ubuntu/take_data/take_video_action/video_action_recorder.py"
REMOTE_CONDA_SH = "/home/rsj/anaconda3/etc/profile.d/conda.sh"
REMOTE_SERVER_PATH = "/home/rsj/franky_control/franky_remote_server.py"


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


class FrankyRemoteClient:
    def __init__(
        self,
        remote_host: Optional[str] = None,
        remote_user: Optional[str] = None,
        remote_password: Optional[str] = None,
        conda_env: str = "franky",
        server_host: str = "0.0.0.0",
        server_port: int = 17001,
        robot_ip: str = "172.16.0.2",
        realtime_config: str = "Enforce",
        dynamics_factor: object = 0.05,
        connect_timeout_s: float = 5.0,
        start_timeout_s: float = 60.0,
        auto_start: bool = True,
    ) -> None:
        cfg = parse_remote_config(CONFIG_FILE)
        self._remote_host = remote_host or cfg.get("REMOTE_HOST") or "172.16.1.2"
        self._remote_user = remote_user or cfg.get("REMOTE_USER") or "rsj"
        self._remote_password = remote_password or cfg.get("REMOTE_PASSWORD")
        self._conda_env = conda_env
        self._server_host = server_host
        self._server_port = int(server_port)
        self._robot_ip = robot_ip
        self._realtime_config = realtime_config
        self._dynamics_factor = dynamics_factor
        self._connect_timeout_s = float(connect_timeout_s)
        self._start_timeout_s = float(start_timeout_s)
        if auto_start:
            self.start()

    def start(self) -> None:
        if check_port(self._remote_host, self._server_port, timeout=1.0):
            return
        self._kill_remote_conflicts()
        if check_port(self._remote_host, self._server_port, timeout=1.0):
            return
        self._ensure_remote_server()
        self._start_remote_server()
        for _ in range(100):
            if check_port(self._remote_host, self._server_port, timeout=1.0):
                return
            time.sleep(0.2)
        raise RuntimeError("Remote server did not start.")

    def close(self, kill_conflicts: bool = True) -> None:
        try:
            self._send_cmd({"cmd": "shutdown"})
        except Exception:
            pass
        self._stop_remote_server()
        if kill_conflicts:
            self._kill_remote_conflicts()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return None

    def get_state(self, include_gripper: bool = True) -> list[float]:
        payload = self._send_cmd(
            {"cmd": "get_state", "include_gripper": bool(include_gripper)}
        )
        if payload.get("error"):
            self._restart_server()
            payload = self._send_cmd(
                {"cmd": "get_state", "include_gripper": bool(include_gripper)}
            )
            if payload.get("error"):
                raise RuntimeError(payload["error"])
        state = payload.get("state")
        if not isinstance(state, list) or len(state) < 8:
            raise RuntimeError("Invalid state response.")
        return state

    def start_stream(self, rate_hz: float = 15.0) -> None:
        payload = self._send_cmd({"cmd": "start_stream", "rate_hz": rate_hz})
        if not payload.get("ok"):
            raise RuntimeError(payload.get("error", "start_stream failed."))

    def set_target(self, target: list[float]) -> None:
        if len(target) < 8:
            raise ValueError("target must have 8 elements (7 joints + gripper)")
        payload = self._send_cmd({"cmd": "set_target", "target": target[:8]})
        if not payload.get("ok"):
            raise RuntimeError(payload.get("error", "set_target failed."))

    def stop_stream(self) -> None:
        payload = self._send_cmd({"cmd": "stop_stream"})
        if not payload.get("ok"):
            raise RuntimeError(payload.get("error", "stop_stream failed."))

    def send_action(self, target: list[float], wait: bool = True) -> None:
        if len(target) < 8:
            raise ValueError("target must have 8 elements (7 joints + gripper)")
        payload = {"cmd": "send_action", "target": target[:8], "wait": bool(wait)}
        result = self._send_cmd(payload)
        if not result.get("ok"):
            err = result.get("error", "send_action failed.")
            raise RuntimeError(err)

    def _build_ssh(self) -> list[str]:
        base = [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            f"ConnectTimeout={int(self._connect_timeout_s)}",
            f"{self._remote_user}@{self._remote_host}",
        ]
        if self._remote_password:
            if not shutil.which("sshpass"):
                raise RuntimeError("sshpass not available for password auth.")
            base = ["sshpass", "-p", self._remote_password] + base
        return base

    def _ensure_remote_server(self) -> None:
        code = _REMOTE_SERVER_CODE.strip()
        cmd = f"cat > {REMOTE_SERVER_PATH} <<'PY'\n{code}\nPY"
        subprocess.run(
            self._build_ssh() + [cmd],
            check=False,
            timeout=self._start_timeout_s,
        )

    def _start_remote_server(self) -> None:
        dyn = self._format_dynamics_factor(self._dynamics_factor)
        cmd = (
            f"source {REMOTE_CONDA_SH} && conda activate {self._conda_env} "
            f"&& nohup python {REMOTE_SERVER_PATH} "
            f"--host {self._server_host} --port {self._server_port} "
            f"--robot-ip {self._robot_ip} --realtime-config {self._realtime_config} "
            f"--dynamics-factor {dyn} "
            f"> /tmp/franky_remote_server.log 2>&1 & echo $! > /tmp/franky_remote_server.pid"
        )
        subprocess.run(
            self._build_ssh() + [cmd],
            check=False,
            timeout=self._start_timeout_s,
        )

    def _stop_remote_server(self) -> None:
        cmd = (
            "if [ -f /tmp/franky_remote_server.pid ]; then "
            "kill $(cat /tmp/franky_remote_server.pid) >/dev/null 2>&1 || true; "
            "rm -f /tmp/franky_remote_server.pid; fi"
        )
        subprocess.run(self._build_ssh() + [cmd], check=False)

    def _restart_server(self) -> None:
        self._kill_remote_conflicts()
        self._start_remote_server()
        for _ in range(100):
            if check_port(self._remote_host, self._server_port, timeout=1.0):
                return
            time.sleep(0.2)
        raise RuntimeError("Remote server did not restart.")

    def _kill_remote_conflicts(self) -> None:
        cmd = (
            "/home/rsj/franka_cpp_control/start_openpi_stream_impedance_pos.sh stop "
            ">/dev/null 2>&1 || true; "
            "pkill -f 'franka_openpi_stream_' >/dev/null 2>&1 || true; "
            "pkill -f 'franky_remote_server.py' >/dev/null 2>&1 || true; "
            "pkill -f 'plot_joint7_curve.py' >/dev/null 2>&1 || true; "
            "if [ -f /tmp/franky_remote_server.pid ]; then "
            "kill $(cat /tmp/franky_remote_server.pid) >/dev/null 2>&1 || true; "
            "rm -f /tmp/franky_remote_server.pid; fi; "
            "pids=$(ss -ltnp 2>/dev/null | awk -v port=':17001' '$4 ~ port {print $0}' "
            "| sed -n 's/.*pid=\\([0-9]\\+\\).*/\\1/p' | sort -u); "
            "for pid in $pids; do kill $pid >/dev/null 2>&1 || true; done"
        )
        subprocess.run(self._build_ssh() + [cmd], check=False)

    def _send_cmd(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        message = json.dumps(payload) + "\n"
        sock = socket.create_connection((self._remote_host, self._server_port), timeout=2.0)
        sock.settimeout(2.0)
        buffer = ""
        try:
            sock.sendall(message.encode("utf-8"))
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="ignore")
                if "\n" in buffer:
                    line, _ = buffer.split("\n", 1)
                    return json.loads(line)
        finally:
            sock.close()
        raise RuntimeError("No response from remote server.")

    @staticmethod
    def _format_dynamics_factor(value: object) -> str:
        if isinstance(value, (list, tuple)):
            if len(value) != 3:
                raise ValueError("dynamics_factor tuple must have 3 elements")
            return ",".join("{0}".format(float(v)) for v in value)
        return "{0}".format(float(value))


_REMOTE_SERVER_CODE = r"""
import argparse
import json
import socket
import sys
import threading
import time

sys.path.insert(0, "/home/rsj/franky_control/franky")

from franky import RealtimeConfig, RelativeDynamicsFactor, Robot
from robotiq_gripper import RobotiqGripper
from robot_actions import execute, get_state


class Streamer:
    def __init__(self, robot, gripper, robot_lock, last_gripper):
        self._robot = robot
        self._gripper = gripper
        self._robot_lock = robot_lock
        self._last_gripper = last_gripper
        self._rate_hz = 15.0
        self._target = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._last_error = None

    def start(self, rate_hz: float) -> None:
        self._rate_hz = max(0.1, float(rate_hz))
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_target(self, target):
        with self._lock:
            self._target = target

    def get_last_error(self):
        return self._last_error

    def _run(self):
        period = 1.0 / self._rate_hz
        next_t = time.time()
        while not self._stop.is_set():
            with self._lock:
                target = self._target
            if target is not None:
                try:
                    with self._robot_lock:
                        execute(self._robot, self._gripper, target)
                        self._last_gripper["value"] = float(target[7])
                    self._last_error = None
                except Exception as exc:
                    self._last_error = str(exc)
            next_t += period
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)


def handle_client(conn, robot, gripper, streamer, robot_lock, last_gripper):
    buffer = ""
    while True:
        data = conn.recv(4096)
        if not data:
            return
        buffer += data.decode("utf-8", errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            payload = json.loads(line)
            cmd = payload.get("cmd")
            if cmd == "get_state":
                include_gripper = payload.get("include_gripper", True)
                with robot_lock:
                    joints = list(robot.state.q)
                    if include_gripper:
                        last_gripper["value"] = gripper.get_position()
                    state = joints + [last_gripper["value"]]
                conn.sendall(json.dumps({"state": state}) .encode("utf-8") + b"\n")
                return
            if cmd == "start_stream":
                rate_hz = payload.get("rate_hz", 15.0)
                streamer.start(rate_hz)
                conn.sendall(json.dumps({"ok": True}) .encode("utf-8") + b"\n")
                return
            if cmd == "set_target":
                target = payload.get("target")
                if not isinstance(target, list) or len(target) < 8:
                    conn.sendall(json.dumps({"ok": False, "error": "invalid target"}) .encode("utf-8") + b"\n")
                    return
                streamer.set_target(target)
                last_gripper["value"] = float(target[7])
                err = streamer.get_last_error()
                if err:
                    conn.sendall(json.dumps({"ok": False, "error": err}) .encode("utf-8") + b"\n")
                    return
                conn.sendall(json.dumps({"ok": True}) .encode("utf-8") + b"\n")
                return
            if cmd == "stop_stream":
                streamer.stop()
                conn.sendall(json.dumps({"ok": True}) .encode("utf-8") + b"\n")
                return
            if cmd == "send_action":
                target = payload.get("target")
                if not isinstance(target, list) or len(target) < 8:
                    conn.sendall(json.dumps({"ok": False, "error": "invalid target"}) .encode("utf-8") + b"\n")
                    return
                with robot_lock:
                    execute(robot, gripper, target)
                    last_gripper["value"] = float(target[7])
                    if payload.get("wait", True):
                        robot.join_motion()
                conn.sendall(json.dumps({"ok": True}) .encode("utf-8") + b"\n")
                return
            if cmd == "shutdown":
                conn.sendall(json.dumps({"ok": True}) .encode("utf-8") + b"\n")
                raise SystemExit(0)
            conn.sendall(json.dumps({"ok": False, "error": "unknown cmd"}) .encode("utf-8") + b"\n")
            return


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=17001)
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument("--realtime-config", default="Enforce")
    parser.add_argument("--dynamics-factor", default="0.05")
    args = parser.parse_args()

    rt_cfg = getattr(RealtimeConfig, args.realtime_config)
    robot = Robot(args.robot_ip, realtime_config=rt_cfg)
    robot.recover_from_errors()
    if isinstance(args.dynamics_factor, str) and "," in args.dynamics_factor:
        parts = [float(v) for v in args.dynamics_factor.split(",")]
        if len(parts) != 3:
            raise SystemExit("dynamics-factor must be float or three comma values")
        robot.relative_dynamics_factor = RelativeDynamicsFactor(*parts)
    else:
        robot.relative_dynamics_factor = float(args.dynamics_factor)

    robot_lock = threading.Lock()
    last_gripper = {"value": 0.0}
    with RobotiqGripper() as gripper:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(5)
        last_gripper["value"] = gripper.get_position()
        streamer = Streamer(robot, gripper, robot_lock, last_gripper)
        while True:
            conn, _addr = server.accept()
            try:
                handle_client(conn, robot, gripper, streamer, robot_lock, last_gripper)
            except SystemExit:
                return
            except Exception as exc:
                try:
                    conn.sendall(json.dumps({"ok": False, "error": str(exc)}) .encode("utf-8") + b"\n")
                except Exception:
                    pass
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
            time.sleep(0.01)


if __name__ == "__main__":
    main()
"""


def main() -> None:
    client_ref = {"client": None}  # 使用字典来绕过作用域限制
    
    def signal_handler(sig, frame):
        """处理 Ctrl+C 信号"""
        print("\n收到中断信号，正在清理资源...")
        if client_ref["client"] is not None:
            try:
                client_ref["client"].close()
            except Exception as e:
                print(f"清理资源时出错: {e}")
        exit(0)
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        with FrankyRemoteClient() as client:
            client_ref["client"] = client
            state = client.get_state()
            print("state:", state)
            target = state[:]
            target[6] = target[6] + math.radians(5.0)
            target[7] = 0.0
            client.send_action(target)
            print("sent action")
    except KeyboardInterrupt:
        # 这个应该被信号处理器捕获，但作为备用
        print("\n收到键盘中断，正在清理资源...")
        if client_ref["client"] is not None:
            client_ref["client"].close()


if __name__ == "__main__":
    main()
