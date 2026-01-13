#!/usr/bin/env python3

import argparse
import json
import os
import subprocess
import re
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


def run_cmd(cmd, timeout=15):
    try:
        return subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", f"Timeout after {timeout}s")


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(text):
    return ANSI_RE.sub("", text)


def stop_gello_publisher():
    run_cmd(["pkill", "-9", "-f", "gello_publisher"], timeout=5)


def start_gello_publisher():
    cmd = (
        "nohup ros2 launch franka_gello_state_publisher main.launch.py "
        "config_file:=fr3_rsjt.yaml > /tmp/gello_publisher_replay.log 2>&1 &"
    )
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def unload_controller(controller_name):
    return run_cmd(["ros2", "control", "unload_controller", controller_name], timeout=20)


def spawn_controller(controller_name):
    result = run_cmd(
        [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            controller_name,
            "--controller-manager-timeout",
            "30",
        ],
        timeout=45,
    )
    if result.returncode != 0:
        output = (result.stderr or result.stdout).strip()
        raise RuntimeError(output or "Failed to spawn controller")
    return result


def list_controllers():
    result = run_cmd(["ros2", "control", "list_controllers"], timeout=20)
    controllers = {}
    if result.returncode != 0:
        return controllers
    for line in strip_ansi(result.stdout).splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        name = parts[0]
        state = parts[-1]
        controllers[name] = state
    return controllers


def controller_manager_ready(timeout=3):
    result = run_cmd(["ros2", "control", "list_controllers"], timeout=timeout)
    return result.returncode == 0


def wait_for_controller_manager(timeout=30.0, poll_interval=1.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if controller_manager_ready(timeout=min(3.0, timeout)):
            return True
        time.sleep(poll_interval)
    return False


def set_controller_state(controller_name, state):
    return run_cmd(
        ["ros2", "control", "set_controller_state", controller_name, state],
        timeout=20,
    )


def reset_controller(controller_name, wait_timeout=30.0):
    if wait_timeout and wait_timeout > 0:
        if not wait_for_controller_manager(timeout=wait_timeout):
            raise RuntimeError("controller_manager not available")
    set_controller_state(controller_name, "inactive")
    for _ in range(3):
        unload_controller(controller_name)
        time.sleep(0.5)
        controllers = list_controllers()
        if controllers and controller_name not in controllers:
            break
        if not controllers:
            break
    spawn_controller(controller_name)


class ReplayNode(Node):
    def __init__(self, publish_rate):
        super().__init__("gello_replay_node")
        self.publish_rate = publish_rate
        self.gello_pub = self.create_publisher(JointState, "/gello/joint_states", 10)
        self.gripper_pub = self.create_publisher(
            Float32, "/gripper/gripper_client/target_gripper_width_percent", 10
        )
        self.franka_sub = self.create_subscription(
            JointState, "/joint_states", self._franka_cb, 10
        )
        self._lock = threading.Lock()
        self._current_q = None

    def _franka_cb(self, msg):
        if not msg.position:
            return
        q = None
        if msg.name and len(msg.name) == len(msg.position):
            name_map = dict(zip(msg.name, msg.position))
            if all(name in name_map for name in JOINT_NAMES):
                q = [name_map[name] for name in JOINT_NAMES]
        if q is None:
            if len(msg.position) < 7:
                return
            q = list(msg.position[:7])
        with self._lock:
            self._current_q = q

    def get_current_q(self):
        with self._lock:
            if self._current_q is None:
                return None
            return list(self._current_q)

    def publish_gello(self, q, gripper):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "fr3_link0"
        msg.name = JOINT_NAMES
        msg.position = list(q)
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        self.gello_pub.publish(msg)
        if gripper is not None:
            gmsg = Float32()
            gmsg.data = float(gripper)
            self.gripper_pub.publish(gmsg)


def load_trajectory(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    trajectory = data.get("trajectory") or []
    if not trajectory:
        raise ValueError("trajectory is empty")
    return trajectory


def wait_for_franka(node, timeout=15.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if node.get_current_q() is not None:
            return True
        time.sleep(0.05)
    return False




def align_to_start(node, target_q, gripper, tolerance, stable_count, timeout):
    start = time.monotonic()
    stable = 0
    period = 1.0 / node.publish_rate
    last_log = 0.0
    while time.monotonic() - start < timeout:
        node.publish_gello(target_q, gripper)
        current = node.get_current_q()
        if current is not None:
            diff = max(abs(a - b) for a, b in zip(current, target_q))
            now = time.monotonic()
            if now - last_log > 1.0:
                print(f"Align diff: {diff:.4f} rad", flush=True)
                last_log = now
            if diff <= tolerance:
                stable += 1
            else:
                stable = 0
            if stable >= stable_count:
                return True
        time.sleep(period)
    return False


def play_trajectory(node, trajectory):
    period = 1.0 / node.publish_rate
    t_end = float(trajectory[-1]["t"])
    start = time.monotonic()
    idx = 0
    while True:
        loop_start = time.monotonic()
        t_now = loop_start - start
        while idx + 1 < len(trajectory) and trajectory[idx + 1]["t"] <= t_now:
            idx += 1
        sample = trajectory[idx]
        node.publish_gello(sample["q"], sample.get("gripper"))
        if t_now >= t_end and idx == len(trajectory) - 1:
            break
        sleep_time = period - (time.monotonic() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)


def hold_pose(node, q, gripper, duration):
    period = 1.0 / node.publish_rate
    start = time.monotonic()
    while time.monotonic() - start < duration:
        node.publish_gello(q, gripper)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description="Replay gello trajectory")
    parser.add_argument("--input", required=True, help="Path to replay JSON")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate")
    parser.add_argument("--align-tolerance", type=float, default=0.02, help="Align tolerance")
    parser.add_argument("--align-stable", type=int, default=10, help="Stable samples")
    parser.add_argument("--align-timeout", type=float, default=60.0, help="Align timeout")
    parser.add_argument("--controller", default="joint_impedance_controller", help="Controller name")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Input not found: {args.input}")
        sys.exit(1)

    trajectory = load_trajectory(args.input)
    start_q = trajectory[0]["q"]
    start_gripper = trajectory[0].get("gripper")

    rclpy.init()
    node = ReplayNode(publish_rate=args.rate)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print("Stopping gello_publisher...", flush=True)
        stop_gello_publisher()

        if not wait_for_franka(node, timeout=15.0):
            raise RuntimeError("No franka joint states received")

        print("Waiting for controller manager...", flush=True)
        if not wait_for_controller_manager(timeout=60.0):
            raise RuntimeError("controller_manager not available")

        print("Resetting controller for alignment...", flush=True)
        reset_controller(args.controller, wait_timeout=0.0)

        print("Aligning to start pose...", flush=True)
        aligned = align_to_start(
            node,
            start_q,
            start_gripper,
            tolerance=args.align_tolerance,
            stable_count=args.align_stable,
            timeout=args.align_timeout,
        )
        if not aligned:
            raise RuntimeError("Align to start pose timed out")

        print("Replaying trajectory...", flush=True)
        play_trajectory(node, trajectory)
        hold_pose(node, trajectory[-1]["q"], trajectory[-1].get("gripper"), duration=1.0)

        print("Stopping controller and restoring gello...", flush=True)
        unload_controller(args.controller)

        start_gello_publisher()
        time.sleep(2.0)
        try:
            reset_controller(args.controller)
        except RuntimeError as exc:
            print(f"Warning: failed to reset controller after replay: {exc}", flush=True)

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
