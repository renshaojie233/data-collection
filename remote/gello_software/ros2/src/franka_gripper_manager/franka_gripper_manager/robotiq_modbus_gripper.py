import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer
from rclpy.node import Node
import serial


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read(slave_id: int, func: int, start: int, qty: int) -> bytes:
    payload = bytes(
        [
            slave_id & 0xFF,
            func & 0xFF,
            (start >> 8) & 0xFF,
            start & 0xFF,
            (qty >> 8) & 0xFF,
            qty & 0xFF,
        ]
    )
    c = crc16(payload)
    return payload + bytes([c & 0xFF, (c >> 8) & 0xFF])


def build_write_multi(slave_id: int, start: int, regs: bytes) -> bytes:
    if len(regs) % 2 != 0:
        raise ValueError("regs length must be even")
    qty = len(regs) // 2
    payload = bytes(
        [
            slave_id & 0xFF,
            0x10,
            (start >> 8) & 0xFF,
            start & 0xFF,
            (qty >> 8) & 0xFF,
            qty & 0xFF,
            len(regs) & 0xFF,
        ]
    ) + regs
    c = crc16(payload)
    return payload + bytes([c & 0xFF, (c >> 8) & 0xFF])


class ModbusClient:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    def open(self, port: str, baud: int, timeout: float):
        self.ser = serial.Serial(
            port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
        )

    def close(self):
        if self.ser:
            self.ser.close()
        self.ser = None

    def read_exact(self, count: int, timeout: float) -> bytes:
        end = time.time() + timeout
        buf = bytearray()
        while len(buf) < count and time.time() < end:
            chunk = self.ser.read(count - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def write_command(
        self,
        slave_id: int,
        out_start: int,
        rACT: int,
        rGTO: int,
        rPR: int,
        rSP: int,
        rFR: int,
        timeout: float,
    ):
        b0 = (rACT & 1) | ((rGTO & 1) << 3)
        regs = bytes([b0, rPR & 0xFF, rSP & 0xFF, rFR & 0xFF, 0, 0])
        req = build_write_multi(slave_id, out_start, regs)
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(req)
            self.ser.flush()
            time.sleep(0.05)
            resp = self.read_exact(8, timeout)
        if len(resp) < 8:
            return False, f"short write response ({len(resp)} bytes)"
        data, crc_bytes = resp[:-2], resp[-2:]
        crc_calc = crc16(data)
        crc_recv = crc_bytes[0] | (crc_bytes[1] << 8)
        if crc_calc != crc_recv:
            return False, "bad CRC"
        if data[1] & 0x80:
            return False, f"exception func=0x{data[1]:02x} code=0x{data[2]:02x}"
        if data[1] != 0x10:
            return False, f"unexpected func=0x{data[1]:02x}"
        return True, None


class RobotiqModbusGripper(Node):
    def __init__(self):
        super().__init__("robotiq_modbus_gripper")
        self.declare_parameter("com_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("slave_id", 9)
        self.declare_parameter("out_start", 0x03E9)
        self.declare_parameter("in_start", 0x07D0)
        self.declare_parameter("timeout", 0.2)
        self.declare_parameter("speed", 128)
        self.declare_parameter("force", 128)

        self._client = ModbusClient()
        self._connected = False
        self._connect_and_activate()

        self._action_server = ActionServer(
            self,
            GripperCommand,
            "robotiq_gripper_controller/gripper_cmd",
            self._execute_callback,
        )

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def _get_int(self, name: str) -> int:
        value = self._get_param(name)
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def _get_float(self, name: str) -> float:
        value = self._get_param(name)
        if isinstance(value, str):
            return float(value)
        return float(value)

    def _connect_and_activate(self):
        port = str(self._get_param("com_port"))
        baud = self._get_int("baud")
        timeout = self._get_float("timeout")
        try:
            self._client.open(port, baud, timeout)
        except Exception as exc:
            self.get_logger().error(f"Failed to open {port}: {exc}")
            return

        slave = self._get_int("slave_id")
        out_start = self._get_int("out_start")
        speed = self._get_int("speed")
        force = self._get_int("force")

        ok, err = self._client.write_command(
            slave, out_start, 0, 0, 0, speed, force, timeout
        )
        if not ok:
            self.get_logger().warn(f"Reset failed: {err}")
        time.sleep(0.4)
        ok, err = self._client.write_command(
            slave, out_start, 1, 0, 0, speed, force, timeout
        )
        if not ok:
            self.get_logger().warn(f"Activate failed: {err}")
        else:
            self._connected = True
            self.get_logger().info(f"Robotiq connected on {port}")

    def _execute_callback(self, goal_handle):
        if not self._connected or self._client.ser is None:
            self.get_logger().error("Gripper not connected")
            goal_handle.abort()
            result = GripperCommand.Result()
            result.reached_goal = False
            return result

        position = goal_handle.request.command.position
        position = max(0.0, min(1.0, float(position)))
        rPR = int(round(position * 255.0))

        slave = self._get_int("slave_id")
        out_start = self._get_int("out_start")
        timeout = self._get_float("timeout")
        speed = self._get_int("speed")
        force = self._get_int("force")

        ok, err = self._client.write_command(
            slave, out_start, 1, 1, rPR, speed, force, timeout
        )
        result = GripperCommand.Result()
        result.position = position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = bool(ok)

        if ok:
            goal_handle.succeed()
        else:
            self.get_logger().error(f"Write failed: {err}")
            goal_handle.abort()
        return result

    def destroy_node(self):
        try:
            self._action_server.destroy()
        except Exception:
            pass
        self._client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqModbusGripper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
