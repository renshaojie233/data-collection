#!/usr/bin/env python3
import argparse
import bisect
import json
import os
import socket
import sys
import time


CANONICAL_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


def smooth_step(x):
    s = max(0.0, min(1.0, x))
    return s * s * s * (10.0 + s * (-15.0 + s * 6.0))


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_action_order(order_csv):
    if not order_csv:
        return None
    order = [token.strip() for token in order_csv.split(",") if token.strip()]
    if len(order) != len(CANONICAL_JOINTS):
        print("ACTION_ORDER size mismatch; using canonical order.", file=sys.stderr)
        return None
    mapping = []
    for name in order:
        if name not in CANONICAL_JOINTS:
            print(f"ACTION_ORDER missing {name}; using canonical order.", file=sys.stderr)
            return None
        mapping.append(CANONICAL_JOINTS.index(name))
    return mapping


def load_trajectory(path):
    with open(path, "r") as f:
        root = json.load(f)
    data = root.get("data", [])
    if not data:
        raise RuntimeError("JSON data is empty")

    times = []
    positions = []
    gripper_times = []
    widths = []
    joint_index = None
    finger_indices = None

    for entry in data:
        ts = entry.get("timestamp")
        franka = entry.get("franka_joints", {})
        names = franka.get("names", [])
        pos = franka.get("position", [])
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
        q = [pos[idx] for idx in joint_index]
        times.append(float(ts))
        positions.append(q)

        gripper = entry.get("gripper_joints")
        if isinstance(gripper, dict):
            g_names = gripper.get("names", [])
            g_pos = gripper.get("position", [])
            if g_names and g_pos:
                if finger_indices is None:
                    if "fr3_finger_joint1" in g_names and "fr3_finger_joint2" in g_names:
                        finger_indices = (
                            g_names.index("fr3_finger_joint1"),
                            g_names.index("fr3_finger_joint2"),
                        )
                    else:
                        finger_indices = (-1, -1)
                f1, f2 = finger_indices
                if f1 >= 0 and f2 >= 0 and len(g_pos) > max(f1, f2):
                    width = g_pos[f1] + g_pos[f2]
                    gripper_times.append(float(ts))
                    widths.append(width)

    if len(times) < 2 or len(positions) < 2:
        raise RuntimeError("Not enough trajectory points")

    t0 = times[0]
    times = [t - t0 for t in times]
    if gripper_times:
        gripper_times = [t - t0 for t in gripper_times]
    return times, positions, gripper_times, widths


def interpolate_scalar(times, values, t):
    if not times or not values:
        return 0.0
    if t <= 0.0:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    idx1 = bisect.bisect_left(times, t)
    if idx1 <= 0:
        return values[0]
    idx0 = idx1 - 1
    t0 = times[idx0]
    t1 = times[idx1]
    alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return values[idx0] + (values[idx1] - values[idx0]) * alpha


def interpolate_positions(times, positions, t):
    if t <= 0.0:
        return positions[0]
    if t >= times[-1]:
        return positions[-1]
    idx1 = bisect.bisect_left(times, t)
    if idx1 <= 0:
        return positions[0]
    idx0 = idx1 - 1
    t0 = times[idx0]
    t1 = times[idx1]
    alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    q0 = positions[idx0]
    q1 = positions[idx1]
    return [q0[i] + (q1[i] - q0[i]) * alpha for i in range(7)]


def read_state_sample(host, port, timeout_s):
    try:
        with socket.create_connection((host, port), timeout=timeout_s) as sock:
            sock.settimeout(timeout_s)
            buffer = b""
            while b"\n" not in buffer:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk
            if b"\n" in buffer:
                line = buffer.split(b"\n", 1)[0].decode("utf-8", errors="ignore").strip()
            else:
                line = buffer.decode("utf-8", errors="ignore").strip()
            if not line:
                return None
            payload = json.loads(line)
            joints = payload.get("joint_position", [])
            gripper = payload.get("gripper_position", 0.0)
            if len(joints) != 7:
                return None
            return joints, float(gripper)
    except Exception:
        return None


def build_gripper_positions(gripper_times, widths, max_width, invert_send):
    if not widths or not gripper_times:
        return None
    max_width = max(max_width, max(widths), 1e-6)
    positions = []
    for width in widths:
        ratio = clamp(width / max_width, 0.0, 1.0)
        pos = 1.0 - ratio
        if invert_send:
            pos = 1.0 - pos
        positions.append(pos)
    return gripper_times, positions


def main():
    parser = argparse.ArgumentParser(
        description="Stream joint position actions from JSON over TCP."
    )
    parser.add_argument("json_path")
    parser.add_argument("--host", default=os.getenv("ACTION_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("ACTION_PORT", "15123")))
    parser.add_argument("--rate", type=float, default=float(os.getenv("RATE_HZ", "30")))
    parser.add_argument("--time-scale", type=float, default=float(os.getenv("TIME_SCALE", "1.0")))
    parser.add_argument("--blend", type=float, default=float(os.getenv("BLEND_S", "0")))
    parser.add_argument("--loop", action="store_true", default=os.getenv("LOOP", "0") == "1")
    parser.add_argument("--state-host", default=os.getenv("STATE_HOST", "127.0.0.1"))
    parser.add_argument("--state-port", type=int, default=int(os.getenv("STATE_PORT", "15124")))
    parser.add_argument("--state-timeout", type=float, default=float(os.getenv("STATE_TIMEOUT_S", "1.0")))
    args = parser.parse_args()

    action_order = parse_action_order(os.getenv("ACTION_ORDER", ""))
    invert_send = os.getenv("GRIPPER_INVERT_SEND", "0") == "1"
    max_width_env = os.getenv("GRIPPER_MAX_WIDTH", "")
    max_width = float(max_width_env) if max_width_env else 0.0

    times, positions, gripper_times, widths = load_trajectory(args.json_path)
    gripper_data = build_gripper_positions(gripper_times, widths, max_width, invert_send)

    blend_s = max(0.0, args.blend)
    q_start = positions[0]
    g_start = gripper_data[1][0] if gripper_data else 0.0
    if blend_s > 0.0:
        state = read_state_sample(args.state_host, args.state_port, args.state_timeout)
        if state is not None:
            q_start, g_start = state
        else:
            print("Warning: no state sample; blend starts from trajectory start.",
                  file=sys.stderr)
            blend_s = 0.0

    duration = times[-1]
    dt = 1.0 / max(1e-6, args.rate)
    time_scale = max(1e-6, args.time_scale)

    try:
        sock = socket.create_connection((args.host, args.port), timeout=2.0)
    except Exception as exc:
        print(f"Failed to connect to action port: {exc}", file=sys.stderr)
        return 2

    print(f"Streaming {args.json_path} -> {args.host}:{args.port}")
    print(f"duration={duration:.3f}s rate={args.rate:.2f}Hz time_scale={time_scale:.3f}")
    if blend_s > 0.0:
        print(f"blend_s={blend_s:.2f}s from current state")

    start_time = time.time()
    next_send = start_time
    try:
        while True:
            now = time.time()
            if now < next_send:
                time.sleep(next_send - now)
                continue

            elapsed = now - start_time
            t_traj = elapsed / time_scale
            if t_traj > duration:
                if args.loop:
                    start_time = time.time()
                    next_send = start_time
                    continue
                break

            q_traj = interpolate_positions(times, positions, t_traj)
            if gripper_data:
                g_traj = interpolate_scalar(gripper_data[0], gripper_data[1], t_traj)
            else:
                g_traj = 0.0

            if blend_s > 0.0 and elapsed < blend_s:
                s = smooth_step(elapsed / blend_s)
                q_target = [q_start[i] + (q_traj[i] - q_start[i]) * s for i in range(7)]
                g_target = g_start + (g_traj - g_start) * s
            else:
                q_target = q_traj
                g_target = g_traj

            if action_order:
                q_out = [q_target[idx] for idx in action_order]
            else:
                q_out = q_target

            action = q_out + [float(g_target)]
            payload = json.dumps({"actions": [action]}, separators=(",", ":")) + "\n"
            sock.sendall(payload.encode("utf-8"))

            next_send += dt
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Streaming error: {exc}", file=sys.stderr)
        return 3
    finally:
        try:
            sock.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
