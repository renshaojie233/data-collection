#!/usr/bin/env python3
"""
Remote ROS2 Data Bridge
Subscribes to ROS2 topics and streams data via TCP to local machine
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import socket
import json
import threading
import time
from datetime import datetime


class ROS2DataBridge(Node):
    def __init__(self, port=9999):
        super().__init__('ros2_data_bridge')

        # TCP server setup
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = True

        # Data buffers
        self.latest_data = {
            'timestamp': None,
            'gello_joints': None,
            'franka_joints': None,
            'gripper_joints': None,
            'gripper_command': None,
            'end_effector_pose': None
        }

        # Create subscribers
        self.gello_sub = self.create_subscription(
            JointState,
            '/gello/joint_states',
            self.gello_callback,
            10
        )

        self.franka_sub = self.create_subscription(
            JointState,
            '/franka/joint_states',
            self.franka_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            JointState,
            '/gripper_joint_states',
            self.gripper_callback,
            10
        )
        self.gripper_ns_sub = self.create_subscription(
            JointState,
            '/gripper/gripper_joint_states',
            self.gripper_callback,
            10
        )
        self.gripper_ns_joint_sub = self.create_subscription(
            JointState,
            '/gripper/joint_states',
            self.gripper_callback,
            10
        )
        self.gripper_franka_sub = self.create_subscription(
            JointState,
            '/franka_gripper/joint_states',
            self.gripper_callback,
            10
        )
        self.gripper_fallback_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.gripper_callback,
            10
        )
        self.gripper_command_sub = self.create_subscription(
            Float32,
            '/gripper/gripper_client/target_gripper_width_percent',
            self.gripper_command_callback,
            10
        )
        self.gripper_command_fallback_sub = self.create_subscription(
            Float32,
            '/gripper_client/target_gripper_width_percent',
            self.gripper_command_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('ROS2 Data Bridge initialized')

        # Start TCP server in separate thread
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

    def gello_callback(self, msg):
        """Callback for GELLO joint states"""
        self.latest_data['gello_joints'] = {
            'position': list(msg.position),
            'velocity': list(msg.velocity) if msg.velocity else None,
            'effort': list(msg.effort) if msg.effort else None,
            'names': list(msg.name) if msg.name else None
        }
        self.latest_data['timestamp'] = time.time()
        self.send_data()

    def franka_callback(self, msg):
        """Callback for Franka joint states"""
        self.latest_data['franka_joints'] = {
            'position': list(msg.position),
            'velocity': list(msg.velocity) if msg.velocity else None,
            'effort': list(msg.effort) if msg.effort else None,
            'names': list(msg.name) if msg.name else None
        }
        self.latest_data['timestamp'] = time.time()
        self.send_data()

    def gripper_callback(self, msg):
        """Callback for Franka gripper joint states"""
        if not self._looks_like_gripper_joint_state(msg):
            return
        self.latest_data['gripper_joints'] = {
            'position': list(msg.position),
            'velocity': list(msg.velocity) if msg.velocity else None,
            'effort': list(msg.effort) if msg.effort else None,
            'names': list(msg.name) if msg.name else None
        }
        self.latest_data['timestamp'] = time.time()
        self.send_data()

    def gripper_command_callback(self, msg):
        """Callback for gripper command (percent open)."""
        try:
            self.latest_data['gripper_command'] = float(msg.data)
        except Exception:
            return
        self.latest_data['timestamp'] = time.time()
        self.send_data()

    @staticmethod
    def _looks_like_gripper_joint_state(msg):
        """Filter non-gripper joint state updates."""
        if not msg.name:
            return False
        keywords = ("gripper", "finger", "knuckle", "robotiq")
        for name in msg.name:
            lower = name.lower()
            if any(keyword in lower for keyword in keywords):
                return True
        return False

    def pose_callback(self, msg):
        """Callback for end effector pose"""
        self.latest_data['end_effector_pose'] = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.latest_data['timestamp'] = time.time()

    def send_data(self):
        """Send data to connected client"""
        if self.client_socket is None:
            return

        try:
            # Prepare data packet
            data_packet = {
                'timestamp': self.latest_data['timestamp'],
                'datetime': datetime.fromtimestamp(self.latest_data['timestamp']).isoformat(),
                'gello_joints': self.latest_data['gello_joints'],
                'franka_joints': self.latest_data['franka_joints'],
                'gripper_joints': self.latest_data['gripper_joints'],
                'gripper_command': self.latest_data['gripper_command'],
                'end_effector_pose': self.latest_data['end_effector_pose']
            }

            # Serialize and send
            json_data = json.dumps(data_packet)
            message = json_data + '\n'  # Use newline as delimiter
            self.client_socket.sendall(message.encode('utf-8'))

        except (BrokenPipeError, ConnectionResetError) as e:
            self.get_logger().warn(f'Client disconnected: {e}')
            self.client_socket = None
            self.client_address = None

    def run_server(self):
        """Run TCP server to accept connections"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(1)

        self.get_logger().info(f'TCP server listening on port {self.port}')

        while self.running:
            try:
                self.get_logger().info('Waiting for client connection...')
                client_socket, client_address = self.server_socket.accept()
                self.client_socket = client_socket
                self.client_address = client_address
                self.get_logger().info(f'Client connected from {client_address}')

                # Keep connection alive
                while self.running and self.client_socket is not None:
                    time.sleep(0.1)

            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Server error: {e}')
                    time.sleep(1)

    def shutdown(self):
        """Cleanup on shutdown"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)

    # Create bridge node
    bridge = ROS2DataBridge(port=9999)

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
