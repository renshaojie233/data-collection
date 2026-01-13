#!/usr/bin/env python3
import argparse
import os
import sys

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

from franka_gello_state_publisher.gello_hardware import GelloHardware
from franka_gello_state_publisher.gello_parameter_config import GelloParameterConfig


JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

DEFAULT_GELLO_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_ROBOT_ZERO = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
DEFAULT_RATE_HZ = 25.0
DEFAULT_ROBOT_JOINT_TOPIC = "franka/joint_states"
DEFAULT_CAPTURE_ROBOT_ZERO = False
DEFAULT_RETURN_SPEED_RAD_S = 0.5


def _load_yaml(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def _normalize(values, length, fill_value=0.0):
    items = list(values or [])
    if len(items) < length:
        items.extend([fill_value] * (length - len(items)))
    elif len(items) > length:
        items = items[:length]
    return items


def _load_gello_params(path):
    cfg = _load_yaml(path)
    if "SINGLE" in cfg:
        cfg = cfg["SINGLE"]
    defaults = {param.descriptor.name: param.default for param in GelloParameterConfig()}
    defaults.update(cfg)
    if not defaults.get("gello_name"):
        defaults["gello_name"] = defaults.get("com_port", "gello")
    return defaults


def _load_relative_config(path):
    cfg = _load_yaml(path)
    gello_zero = cfg.get("gello_zero", DEFAULT_GELLO_ZERO)
    robot_zero = cfg.get("robot_zero", DEFAULT_ROBOT_ZERO)
    rate_hz = float(cfg.get("publish_rate_hz", DEFAULT_RATE_HZ))
    robot_joint_topic = cfg.get("robot_joint_topic", DEFAULT_ROBOT_JOINT_TOPIC)
    capture_robot_zero = bool(cfg.get("capture_robot_zero", DEFAULT_CAPTURE_ROBOT_ZERO))
    return_speed_rad_s = float(cfg.get("return_speed_rad_s", DEFAULT_RETURN_SPEED_RAD_S))
    return {
        "gello_zero": gello_zero,
        "robot_zero": robot_zero,
        "publish_rate_hz": rate_hz,
        "robot_joint_topic": robot_joint_topic,
        "capture_robot_zero": capture_robot_zero,
        "return_speed_rad_s": return_speed_rad_s,
    }


class RelativeGelloPublisher(Node):
    def __init__(self, gello_params, rel_cfg, auto_enable):
        super().__init__("gello_relative_publisher")
        self._num_joints = int(gello_params["num_arm_joints"])
        self._gello_zero = np.array(
            _normalize(rel_cfg["gello_zero"], self._num_joints, 0.0), dtype=float
        )
        self._robot_zero = np.array(
            _normalize(rel_cfg["robot_zero"], self._num_joints, 0.0), dtype=float
        )
        self._publish_rate_hz = float(rel_cfg["publish_rate_hz"])
        self._robot_joint_topic = rel_cfg["robot_joint_topic"]
        self._capture_robot_zero = bool(rel_cfg["capture_robot_zero"])
        self._return_speed_rad_s = float(rel_cfg["return_speed_rad_s"])
        self._last_robot_q = None
        self._last_target = None
        self._control_enabled = bool(auto_enable)

        self._gello_hardware = GelloHardware(gello_params, self.get_logger())
        self._arm_pub = self.create_publisher(JointState, "gello/joint_states", 10)
        self._gripper_pub = self.create_publisher(
            Float32, "gripper/gripper_client/target_gripper_width_percent", 10
        )
        self._robot_joint_sub = None
        if self._capture_robot_zero:
            self._robot_joint_sub = self.create_subscription(
                JointState, self._robot_joint_topic, self._robot_joint_callback, 10
            )
        self._enable_srv = self.create_service(SetBool, "gello_relative/enable", self._handle_enable)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._tick)

        if self._control_enabled:
            if self._capture_relative_zero():
                self.get_logger().info("Relative control enabled (auto).")
            else:
                self._control_enabled = False
                self.get_logger().warning(
                    "Auto-enable requested, but failed to capture zero poses. Control disabled."
                )
        else:
            self.get_logger().info(
                "Relative control disabled. Call /gello_relative/enable to start."
            )

    def _handle_enable(self, request, response):
        if request.data:
            if self._capture_relative_zero():
                self._control_enabled = True
                response.success = True
                response.message = "Relative control enabled"
            else:
                self._control_enabled = False
                response.success = False
                response.message = "Failed to capture zero poses; control not enabled"
        else:
            self._control_enabled = False
            response.success = True
            response.message = "Relative control disabled"
        self.get_logger().info(response.message)
        return response

    def _robot_joint_callback(self, msg):
        positions = self._extract_joint_positions(msg)
        if positions is None:
            return
        self._last_robot_q = positions

    def _extract_joint_positions(self, msg):
        if not msg.position:
            return None
        if msg.name:
            name_to_pos = dict(zip(msg.name, msg.position))
            try:
                return [float(name_to_pos[name]) for name in JOINT_NAMES[: self._num_joints]]
            except KeyError:
                return None
        if len(msg.position) >= self._num_joints:
            return [float(v) for v in msg.position[: self._num_joints]]
        return None

    def _capture_robot_zero(self):
        if self._last_robot_q is None:
            self.get_logger().warning(
                f"No robot joint states on {self._robot_joint_topic}; cannot set robot zero."
            )
            return False
        if len(self._last_robot_q) != self._num_joints:
            self.get_logger().warning(
                f"Unexpected robot joint count {len(self._last_robot_q)} "
                f"(expected {self._num_joints})"
            )
            return False
        self._robot_zero = np.array(self._last_robot_q, dtype=float)
        self.get_logger().info(
            f"Captured robot zero: {np.array2string(self._robot_zero, precision=3)}"
        )
        return True

    def _capture_gello_zero(self):
        try:
            gello_q, _ = self._gello_hardware.read_joint_states()
        except Exception as exc:
            self.get_logger().warning(f"Failed to capture GELLO zero: {exc}")
            return False
        if len(gello_q) != self._num_joints:
            self.get_logger().warning(
                f"Unexpected GELLO joint count {len(gello_q)} (expected {self._num_joints})"
            )
            return False
        self._gello_zero = np.array(gello_q, dtype=float)
        self.get_logger().info(
            f"Captured GELLO zero: {np.array2string(self._gello_zero, precision=3)}"
        )
        return True

    def _capture_relative_zero(self):
        if self._capture_robot_zero:
            if not self._capture_robot_zero():
                return False
        else:
            self.get_logger().info(
                f"Using fixed robot zero: {np.array2string(self._robot_zero, precision=3)}"
            )
        return self._capture_gello_zero()

    def _step_toward_zero(self):
        if self._last_target is None:
            return self._robot_zero
        if self._return_speed_rad_s <= 0.0:
            return self._robot_zero
        step = self._return_speed_rad_s / self._publish_rate_hz
        diff = self._robot_zero - self._last_target
        if np.all(np.abs(diff) <= step):
            return self._robot_zero
        return self._last_target + np.clip(diff, -step, step)

    def _tick(self):
        try:
            gello_q, gripper = self._gello_hardware.read_joint_states()
        except Exception as exc:
            self.get_logger().warning(f"Failed to read GELLO joints: {exc}")
            return
        gello_q = np.array(gello_q, dtype=float)

        if self._control_enabled:
            delta = gello_q - self._gello_zero
            q_target = self._robot_zero + delta
        else:
            q_target = self._step_toward_zero()
        self._last_target = np.array(q_target, dtype=float)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "fr3_link0"
        msg.name = JOINT_NAMES[: self._num_joints]
        msg.position = [float(v) for v in q_target.tolist()]
        self._arm_pub.publish(msg)

        gripper_msg = Float32()
        gripper_msg.data = float(gripper)
        self._gripper_pub.publish(gripper_msg)


def main():
    parser = argparse.ArgumentParser(description="Relative GELLO publisher")
    parser.add_argument(
        "--gello-config",
        default="/home/rsj/gello_software/ros2/src/franka_gello_state_publisher/config/fr3_rsjt.yaml",
        help="GELLO config YAML path",
    )
    parser.add_argument(
        "--relative-config",
        default="/home/rsj/gello_software/configs/relative_gello_control.yaml",
        help="Relative control config YAML path",
    )
    parser.add_argument(
        "--auto-enable",
        action="store_true",
        help="Enable relative control immediately",
    )
    args = parser.parse_args()

    gello_params = _load_gello_params(args.gello_config)
    rel_cfg = _load_relative_config(args.relative_config)

    rclpy.init()
    node = RelativeGelloPublisher(gello_params, rel_cfg, args.auto_enable)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if "context is not valid" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
