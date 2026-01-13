#!/usr/bin/env python3
"""Replay franka_joints.position trajectories with Robotiq gripper via Python."""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

sys.path.insert(0, '/home/rsj/franky_control/franky')

from franky import (
    Duration,
    JointMotion,
    JointState,
    JointWaypoint,
    JointWaypointMotion,
    RealtimeConfig,
    ReferenceType,
    Robot,
)
from franky._franky import RelativeDynamicsFactor
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType

CANONICAL_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

FINGER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint2")


@dataclass
class Trajectory:
    times: List[float]
    positions: List[List[float]]
    velocities: List[List[float]]
    gripper_times: List[float]
    gripper_widths: List[float]


def _to_float_list(values: List[float], idx: List[int]) -> List[float]:
    return [float(values[i]) for i in idx]


def load_trajectory(path: str) -> Trajectory:
    with open(path, "r") as f:
        root = json.load(f)

    data = root.get("data", [])
    if not data:
        raise RuntimeError("JSON data is empty")

    times: List[float] = []
    positions: List[List[float]] = []
    velocities: List[Optional[List[float]]] = []
    gripper_times: List[float] = []
    gripper_widths: List[float] = []

    joint_index: Optional[List[int]] = None
    finger_index: Optional[Tuple[int, int]] = None

    for entry in data:
        ts = entry.get("timestamp")
        franka = entry.get("franka_joints", {})
        names = franka.get("names", [])
        pos = franka.get("position", [])
        vel = franka.get("velocity", None)
        if ts is None or not names or not pos:
            continue

        if joint_index is None:
            joint_index = []
            for name in CANONICAL_JOINTS:
                if name not in names:
                    raise RuntimeError("Missing joint name in franka_joints.names")
                joint_index.append(names.index(name))

        if len(pos) < 7:
            raise RuntimeError("Invalid franka_joints.position length")

        times.append(float(ts))
        positions.append(_to_float_list(pos, joint_index))

        if isinstance(vel, list) and len(vel) >= 7:
            velocities.append(_to_float_list(vel, joint_index))
        else:
            velocities.append(None)

        gripper = entry.get("gripper_joints")
        if isinstance(gripper, dict):
            g_names = gripper.get("names", [])
            g_pos = gripper.get("position", [])
            if g_names and g_pos:
                if finger_index is None:
                    if all(name in g_names for name in FINGER_JOINTS):
                        finger_index = (
                            g_names.index(FINGER_JOINTS[0]),
                            g_names.index(FINGER_JOINTS[1]),
                        )
                if finger_index is not None:
                    f1, f2 = finger_index
                    if len(g_pos) > max(f1, f2):
                        width = float(g_pos[f1]) + float(g_pos[f2])
                        gripper_times.append(float(ts))
                        gripper_widths.append(width)

    if len(times) < 2:
        raise RuntimeError("Not enough trajectory points")

    t0 = times[0]
    times = [t - t0 for t in times]
    if gripper_times:
        gripper_times = [t - t0 for t in gripper_times]

    have_vel = all(v is not None for v in velocities)
    if have_vel:
        vel_list: List[List[float]] = [v for v in velocities if v is not None]
    else:
        vel_list = compute_velocities(times, positions)

    return Trajectory(times, positions, vel_list, gripper_times, gripper_widths)


def compute_velocities(times: List[float], positions: List[List[float]]) -> List[List[float]]:
    n = len(times)
    vel = [[0.0] * 7 for _ in range(n)]
    for i in range(n):
        if i == 0:
            dt = max(1e-6, times[1] - times[0])
            for j in range(7):
                vel[i][j] = (positions[1][j] - positions[0][j]) / dt
        elif i == n - 1:
            dt = max(1e-6, times[-1] - times[-2])
            for j in range(7):
                vel[i][j] = (positions[-1][j] - positions[-2][j]) / dt
        else:
            dt = max(1e-6, times[i + 1] - times[i - 1])
            for j in range(7):
                vel[i][j] = (positions[i + 1][j] - positions[i - 1][j]) / dt
    return vel


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def resample_positions(
    times: List[float],
    positions: List[List[float]],
    hz: float,
    ease_in_out: bool,
) -> Tuple[List[float], List[List[float]]]:
    if hz <= 0.0:
        return times, positions
    duration = times[-1]
    if duration <= 0.0:
        return times, positions
    dt = 1.0 / hz
    count = int(duration / dt) + 1
    out_times: List[float] = []
    out_positions: List[List[float]] = []
    idx = 0
    n = len(times)
    for k in range(count):
        t = k * dt
        t_query = smoothstep(t / duration) * duration if ease_in_out else t
        while idx + 1 < n and times[idx + 1] < t_query:
            idx += 1
        idx_next = min(idx + 1, n - 1)
        t0 = times[idx]
        t1 = times[idx_next]
        alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
        q = [
            positions[idx][j] + (positions[idx_next][j] - positions[idx][j]) * alpha
            for j in range(7)
        ]
        out_times.append(t)
        out_positions.append(q)
    return out_times, out_positions


def smooth_positions(positions: List[List[float]], window: int) -> List[List[float]]:
    if window <= 1:
        return positions
    half = window // 2
    out: List[List[float]] = []
    n = len(positions)
    for i in range(n):
        start = max(0, i - half)
        end = min(n - 1, i + half)
        count = end - start + 1
        acc = [0.0] * 7
        for k in range(start, end + 1):
            for j in range(7):
                acc[j] += positions[k][j]
        out.append([v / count for v in acc])
    return out


def process_trajectory(
    traj: Trajectory,
    resample_hz: float,
    smooth_window: int,
    ease_in_out: bool,
) -> Trajectory:
    times, positions = resample_positions(traj.times, traj.positions, resample_hz, ease_in_out)
    positions = smooth_positions(positions, smooth_window)
    velocities = compute_velocities(times, positions)
    return Trajectory(times, positions, velocities, traj.gripper_times, traj.gripper_widths)


def select_indices(times: List[float], sample_period: float, max_points: int) -> List[int]:
    n = len(times)
    if n == 0:
        return []
    indices = [0]
    last_t = times[0]
    if sample_period <= 0.0:
        indices = list(range(n))
    else:
        for i in range(1, n - 1):
            if times[i] - last_t >= sample_period:
                indices.append(i)
                last_t = times[i]
        if indices[-1] != n - 1:
            indices.append(n - 1)

    if max_points > 0 and len(indices) > max_points:
        stride = int(math.ceil(len(indices) / max_points))
        indices = indices[::stride]
        if indices[-1] != n - 1:
            indices.append(n - 1)
    return indices


def interpolate_scalar(times: List[float], values: List[float], t: float) -> float:
    if not times or not values:
        return 0.0
    if t <= 0.0:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    idx1 = 0
    for i, tv in enumerate(times):
        if tv >= t:
            idx1 = i
            break
    if idx1 <= 0:
        return values[0]
    idx0 = idx1 - 1
    t0 = times[idx0]
    t1 = times[idx1]
    alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return values[idx0] + (values[idx1] - values[idx0]) * alpha


@dataclass
class RobotiqConfig:
    port: str
    baud: int
    slave_id: int
    out_start: int
    in_start: int
    timeout_ms: int
    min_command_interval_ms: int
    speed: int
    force: int
    source_max_width: float
    position_eps: float
    codex_layout: bool
    hold_initial: bool
    activation_mode: int


def to_robotiq_byte(value: float, fallback: int) -> int:
    if math.isnan(value) or math.isinf(value):
        return fallback
    if value <= 1.0:
        return max(0, min(255, int(round(value * 255.0))))
    return max(0, min(255, int(round(value))))


def parse_activation_mode(value: str) -> int:
    lowered = value.strip().lower()
    if lowered in {"skip", "0", "false"}:
        return 2
    if lowered in {"force", "1", "true"}:
        return 1
    return 0


def parse_float_list(value: str, expected: int, label: str) -> Optional[List[float]]:
    if not value:
        return None
    parts = [p.strip() for p in value.split(",") if p.strip()]
    if len(parts) != expected:
        raise SystemExit(f"{label} expects {expected} values, got {len(parts)}")
    try:
        return [float(p) for p in parts]
    except ValueError as exc:
        raise SystemExit(f"{label} contains non-numeric values") from exc


def detect_robotiq_port() -> Optional[str]:
    if os.path.exists("/dev/robotiq"):
        return "/dev/robotiq"
    patterns = [
        "/dev/serial/by-id/usb-FTDI_USB_TO_RS-485*",
        "/dev/serial/by-id/usb-FTDI_FT232*",
        "/dev/serial/by-id/usb-ROBOTIQ*",
    ]
    matches: List[str] = []
    for pattern in patterns:
        matches.extend([p for p in glob_glob(pattern) if os.path.exists(p)])
    if len(matches) == 1:
        return matches[0]
    matches = [p for p in glob_glob("/dev/ttyUSB*") if os.path.exists(p)]
    if len(matches) == 1:
        return matches[0]
    return None


def glob_glob(pattern: str) -> List[str]:
    import glob

    return glob.glob(pattern)


class RobotiqModbus:
    def __init__(self, config: RobotiqConfig) -> None:
        self._config = config
        self._client = ModbusSerialClient(
            port=config.port,
            framer=FramerType.RTU,
            baudrate=config.baud,
            timeout=config.timeout_ms / 1000.0,
        )

    def connect(self) -> None:
        if not self._client.connect():
            raise RuntimeError(f"Failed to connect to {self._config.port}")

    def close(self) -> None:
        self._client.close()

    def _write_command(self, rACT: int, rGTO: int, rPR: int, rSP: int, rFR: int) -> None:
        b0 = (rACT & 1) | ((rGTO & 1) << 3)
        if self._config.codex_layout:
            reg0 = (b0 << 8) | 0
            reg1 = rPR & 0xFF
            reg2 = ((rSP & 0xFF) << 8) | (rFR & 0xFF)
        else:
            reg0 = ((b0 & 0xFF) << 8) | (rPR & 0xFF)
            reg1 = ((rSP & 0xFF) << 8) | (rFR & 0xFF)
            reg2 = 0
        result = self._client.write_registers(
            self._config.out_start,
            [reg0, reg1, reg2],
            device_id=self._config.slave_id,
        )
        if result.isError():
            raise RuntimeError(f"Robotiq write failed: {result}")

    def _read_status(self) -> Optional[Tuple[int, int]]:
        result = self._client.read_holding_registers(
            self._config.in_start,
            count=3,
            device_id=self._config.slave_id,
        )
        if result.isError():
            return None
        reg0 = result.registers[0]
        gACT = (reg0 >> 15) & 0x01
        gSTA = (reg0 >> 8) & 0x03
        return gACT, gSTA

    def activate(self) -> None:
        if self._config.activation_mode == 2:
            return
        if self._config.activation_mode == 0:
            status = self._read_status()
            if status is not None:
                gACT, gSTA = status
                if gACT == 1 and gSTA >= 2:
                    return
        speed = max(0, min(255, self._config.speed))
        force = max(0, min(255, self._config.force))
        self._write_command(0, 0, 0, speed, force)
        time.sleep(0.4)
        self._write_command(1, 0, 0, speed, force)

    def move_normalized(self, position: float) -> None:
        position = max(0.0, min(1.0, position))
        rPR = max(0, min(255, int(round(position * 255.0))))
        speed = max(0, min(255, self._config.speed))
        force = max(0, min(255, self._config.force))
        self._write_command(1, 1, rPR, speed, force)


def stop_conflicts() -> None:
    patterns = [
        "/home/rsj/gello_software/run_fr3_real_ros2_robotiq.sh",
        "franka_fr3_arm_controllers",
        "franka_gripper_manager",
        "robotiq_modbus_gripper",
        "robotiq_gripper_client",
        "franka_gello_state_publisher",
    ]
    pids = set()
    for pat in patterns:
        try:
            out = subprocess.check_output(["pgrep", "-af", pat], text=True)
        except subprocess.CalledProcessError:
            continue
        for line in out.strip().splitlines():
            parts = line.split(None, 1)
            if not parts:
                continue
            pid = int(parts[0])
            if pid in (os.getpid(), os.getppid()):
                continue
            pids.add(pid)
    if pids:
        print(f"Stopping conflicting processes: {' '.join(str(p) for p in sorted(pids))}")
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
        time.sleep(2.0)


def build_waypoints(
    traj: Trajectory,
    time_scale: float,
    sample_period: float,
    max_points: int,
    use_velocity: bool,
    use_min_time: bool,
) -> List[JointWaypoint]:
    indices = select_indices(traj.times, sample_period, max_points)
    waypoints: List[JointWaypoint] = []
    prev_t = traj.times[indices[0]]
    for i, idx in enumerate(indices):
        pos = traj.positions[idx]
        vel = traj.velocities[idx]
        if use_velocity and vel is not None:
            scale = 1.0 / max(1e-6, time_scale)
            vel = [v * scale for v in vel]
            target = JointState(pos, vel)
        else:
            target = pos
        min_time = None
        if use_min_time and i > 0:
            dt = (traj.times[idx] - prev_t) * time_scale
            if dt > 0.0:
                min_time = Duration(int(round(dt * 1000.0)))
        waypoint = JointWaypoint(
            target,
            reference_type=ReferenceType.Absolute,
            relative_dynamics_factor=1.0,
            minimum_time=min_time,
        )
        waypoints.append(waypoint)
        prev_t = traj.times[idx]
    return waypoints


def gripper_thread(
    traj: Trajectory,
    cfg: RobotiqConfig,
    time_scale: float,
    start_delay: float,
    ease_in_out: bool,
    frame_hz: float,
    start_event: Optional[threading.Event],
    ready_event: Optional[threading.Event],
    start_time_ref: Optional[List[Optional[float]]],
    error_holder: List[BaseException],
) -> None:
    if not traj.gripper_times or not traj.gripper_widths:
        return
    source_max_width = cfg.source_max_width
    if source_max_width <= 0.0:
        source_max_width = max(traj.gripper_widths)
    if source_max_width <= 0.0:
        return

    try:
        gripper = RobotiqModbus(cfg)
        gripper.connect()
        gripper.activate()

        if ready_event is not None:
            ready_event.set()
        if start_event is not None:
            start_event.wait()
        start = time.monotonic()
        if start_time_ref and start_time_ref[0] is not None:
            start = start_time_ref[0]
        if start_delay > 0.0:
            start += start_delay
        now = time.monotonic()
        if start > now:
            time.sleep(start - now)
        last_cmd = start - cfg.min_command_interval_ms / 1000.0
        last_position = None
        have_state = False
        initial_skipped = False

        duration = traj.gripper_times[-1]
        frame_dt = 1.0 / frame_hz if frame_hz and frame_hz > 0.0 else 0.0
        frame_times = traj.times if frame_dt > 0.0 else None
        while True:
            now = time.monotonic()
            elapsed = now - start
            t_traj = elapsed / max(1e-6, time_scale)
            if ease_in_out and duration > 0.0:
                t_traj = smoothstep(min(t_traj / duration, 1.0)) * duration
            t_clamped = min(t_traj, duration)
            if frame_times is not None:
                frame_idx = min(int(t_clamped / frame_dt), len(frame_times) - 1)
                t_query = frame_times[frame_idx]
            else:
                t_query = t_clamped
            width = interpolate_scalar(traj.gripper_times, traj.gripper_widths, t_query)
            width_ratio = max(0.0, min(1.0, width / max(1e-6, source_max_width)))
            position = 1.0 - width_ratio

            if not have_state and cfg.hold_initial and not initial_skipped:
                last_position = position
                have_state = True
                initial_skipped = True

            if (not have_state) or (last_position is not None and abs(position - last_position) >= cfg.position_eps):
                if now - last_cmd >= cfg.min_command_interval_ms / 1000.0:
                    gripper.move_normalized(position)
                    last_cmd = now
                    last_position = position
                    have_state = True

            if t_traj >= duration:
                break
            time.sleep(0.01)
        gripper.close()
    except BaseException as exc:
        error_holder.append(exc)


def pick_latest_json_with_gripper(json_dir: str) -> Optional[str]:
    paths = []
    for name in os.listdir(json_dir):
        if not name.endswith(".json"):
            continue
        path = os.path.join(json_dir, name)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        paths.append((mtime, path))
    paths.sort(reverse=True)
    for _, path in paths:
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except Exception:
            continue
        entries = data.get("data", [])
        for entry in entries[:200]:
            gripper = entry.get("gripper_joints")
            if not isinstance(gripper, dict):
                continue
            names = gripper.get("names")
            pos = gripper.get("position")
            if isinstance(names, list) and isinstance(pos, list) and names and pos:
                return path
    return None


def pick_latest_json(json_dir: str) -> Optional[str]:
    paths = []
    for name in os.listdir(json_dir):
        if not name.endswith(".json"):
            continue
        path = os.path.join(json_dir, name)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        paths.append((mtime, path))
    if not paths:
        return None
    paths.sort(reverse=True)
    return paths[0][1]


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded joint trajectory with Robotiq gripper")
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    parser.add_argument("--json", default="", help="JSON path (defaults to latest)")
    parser.add_argument(
        "--json-dir",
        default="/home/rsj/franka_cpp_control/replay_data/action",
        help="Directory to search for JSON",
    )
    parser.add_argument("--time-scale", type=float, default=1.0, help="Replay speed factor")
    parser.add_argument("--sample-period", type=float, default=0.01, help="Downsample period (s), 0=all")
    parser.add_argument("--max-points", type=int, default=5000, help="Max waypoints (0=unlimited)")
    parser.add_argument("--relative-dynamics", type=float, default=0.1, help="Robot dynamics factor")
    parser.add_argument("--dyn-vel", type=float, default=0.08, help="Relative velocity factor override")
    parser.add_argument("--dyn-acc", type=float, default=0.05, help="Relative acceleration factor override")
    parser.add_argument("--dyn-jerk", type=float, default=0.03, help="Relative jerk factor override")
    parser.add_argument("--joint-impedance", default="150,150,150,150,100,80,60", help="7 stiffness values (comma-separated)")
    parser.add_argument("--cartesian-impedance", default="", help="6 stiffness values (comma-separated)")
    parser.add_argument("--start-delay", type=float, default=0.5, help="Gripper start delay (s)")
    parser.add_argument("--sync-delay", type=float, default=0.2, help="Delay before synced start (s)")
    parser.add_argument("--no-sync-gripper", action="store_true", help="Disable gripper/arm start sync")
    parser.add_argument("--skip-move-to-start", action="store_true", help="Skip move to first waypoint")
    parser.add_argument("--no-gripper", action="store_true", help="Disable gripper tracking")
    parser.add_argument("--use-velocity", action="store_true", help="Use recorded joint velocities")
    parser.add_argument("--no-min-time", action="store_true", help="Ignore per-waypoint minimum time")
    parser.add_argument("--resample-hz", type=float, default=200.0, help="Resample arm trajectory (Hz), 0=disable")
    parser.add_argument("--smooth-window", type=int, default=None, help="Moving average window (<=1=off)")
    parser.add_argument("--ease-in-out", action="store_true", help="Smooth time warp at start/end")
    parser.add_argument("--frame-hz", type=float, default=15.0, help="Align arm trajectory to video frame rate (Hz)")
    parser.add_argument("--stop-conflicts", action="store_true", help="Stop known conflicting processes")
    parser.add_argument(
        "--realtime-config",
        default="enforce",
        choices=["enforce", "ignore"],
        help="Realtime config for libfranka",
    )
    args = parser.parse_args()

    json_path = args.json
    if not json_path:
        json_path = pick_latest_json_with_gripper(args.json_dir) or pick_latest_json(args.json_dir)
    if not json_path:
        raise SystemExit(f"No JSON files found in {args.json_dir}")

    if args.stop_conflicts:
        stop_conflicts()

    resample_hz = args.resample_hz
    ease_in_out = args.ease_in_out
    sample_period = args.sample_period
    max_points = args.max_points
    frame_hz = args.frame_hz if args.frame_hz and args.frame_hz > 0.0 else 0.0
    start_delay = args.start_delay
    if args.smooth_window is None:
        smooth_window = 11
    else:
        smooth_window = args.smooth_window

    if frame_hz > 0.0:
        resample_hz = frame_hz
        ease_in_out = False
        sample_period = 0.0
        max_points = 0
        if args.smooth_window is None:
            smooth_window = 5
        if args.start_delay == 0.5:
            start_delay = 0.0
        print(f"Frame alignment: {frame_hz} Hz")

    traj = load_trajectory(json_path)
    traj = process_trajectory(traj, resample_hz, smooth_window, ease_in_out)
    print(f"JSON: {json_path}")
    print(f"Trajectory points: {len(traj.times)}")

    if args.no_gripper or not traj.gripper_times:
        print("Gripper tracking: disabled")
        gripper_cfg = None
    else:
        port = os.getenv("ROBOTIQ_PORT", "") or detect_robotiq_port() or ""
        if not port:
            print("Warning: ROBOTIQ_PORT not found, disabling gripper")
            gripper_cfg = None
        else:
            layout = os.getenv("ROBOTIQ_LAYOUT", "codex").strip().lower()
            start_mode = os.getenv("ROBOTIQ_START_MODE", "hold").strip().lower()
            activate_mode = os.getenv("ROBOTIQ_ACTIVATE", "auto")
            min_cmd_env = os.getenv("ROBOTIQ_MIN_CMD_MS")
            pos_eps_env = os.getenv("ROBOTIQ_POSITION_EPS")
            min_cmd_ms = int(min_cmd_env) if min_cmd_env is not None else 200
            if frame_hz > 0.0 and min_cmd_env is None:
                min_cmd_ms = max(20, int(round(1000.0 / frame_hz)))
            pos_eps = float(pos_eps_env) if pos_eps_env is not None else 0.02
            if frame_hz > 0.0 and pos_eps_env is None:
                pos_eps = min(pos_eps, 0.005)
            gripper_cfg = RobotiqConfig(
                port=port,
                baud=int(os.getenv("ROBOTIQ_BAUD", "115200")),
                slave_id=int(os.getenv("ROBOTIQ_SLAVE_ID", "9"), 0),
                out_start=int(os.getenv("ROBOTIQ_OUT_START", "0x03E8"), 0),
                in_start=int(os.getenv("ROBOTIQ_IN_START", "0x07D0"), 0),
                timeout_ms=int(os.getenv("ROBOTIQ_TIMEOUT_MS", "200")),
                min_command_interval_ms=min_cmd_ms,
                speed=to_robotiq_byte(float(os.getenv("ROBOTIQ_SPEED", "0.1")), 128),
                force=to_robotiq_byte(float(os.getenv("ROBOTIQ_FORCE", "20")), 128),
                source_max_width=float(os.getenv("ROBOTIQ_SOURCE_MAX_WIDTH", "0.08")),
                position_eps=pos_eps,
                codex_layout=layout not in {"gello", "legacy", "original"},
                hold_initial=start_mode == "hold",
                activation_mode=parse_activation_mode(activate_mode),
            )
            print(f"Robotiq port: {port}")

    rt_cfg = RealtimeConfig.Ignore if args.realtime_config == "ignore" else RealtimeConfig.Enforce
    robot = Robot(args.host, realtime_config=rt_cfg)
    robot.recover_from_errors()
    dyn_vel = args.dyn_vel if args.dyn_vel is not None else args.relative_dynamics
    dyn_acc = args.dyn_acc if args.dyn_acc is not None else args.relative_dynamics
    dyn_jerk = args.dyn_jerk if args.dyn_jerk is not None else args.relative_dynamics
    robot.relative_dynamics_factor = RelativeDynamicsFactor(dyn_vel, dyn_acc, dyn_jerk)
    joint_imp = parse_float_list(args.joint_impedance, 7, "joint-impedance")
    if joint_imp is not None:
        robot.set_joint_impedance(joint_imp)
    cart_imp = parse_float_list(args.cartesian_impedance, 6, "cartesian-impedance")
    if cart_imp is not None:
        robot.set_cartesian_impedance(cart_imp)

    gripper_errors: List[BaseException] = []
    sync_gripper = gripper_cfg is not None and not args.no_sync_gripper
    start_event = threading.Event() if sync_gripper else None
    ready_event = threading.Event() if sync_gripper else None
    start_time_ref: Optional[List[Optional[float]]] = [None] if sync_gripper else None
    if gripper_cfg is not None:
        thread = threading.Thread(
            target=gripper_thread,
            args=(
                traj,
                gripper_cfg,
                args.time_scale,
                start_delay,
                ease_in_out,
                frame_hz,
                start_event,
                ready_event,
                start_time_ref,
                gripper_errors,
            ),
            daemon=True,
        )
        thread.start()
    else:
        thread = None

    if not args.skip_move_to_start:
        robot.move(JointMotion(traj.positions[0]))

    if sync_gripper and ready_event is not None:
        ready_event.wait(timeout=10.0)
        start_time = time.monotonic() + max(0.0, args.sync_delay)
        if start_time_ref is not None:
            start_time_ref[0] = start_time
        if start_event is not None:
            start_event.set()
        now = time.monotonic()
        if start_time > now:
            time.sleep(start_time - now)

    waypoints = build_waypoints(
        traj,
        args.time_scale,
        sample_period,
        max_points,
        args.use_velocity,
        not args.no_min_time,
    )
    motion = JointWaypointMotion(waypoints)
    robot.move(motion)

    if thread is not None:
        thread.join()
    if gripper_errors:
        raise gripper_errors[0]


if __name__ == "__main__":
    main()