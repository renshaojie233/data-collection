#!/usr/bin/env python3
"""
ROS2 node to record real Franka robot arm joint states during operation.
Subscribes to /franka/joint_states and /franka_gripper/joint_states
and saves the data to a pickle file when recording is enabled.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import pickle
import os
from datetime import datetime
import threading


class FrankaRecorderNode(Node):
    def __init__(self):
        super().__init__('franka_recorder_node')

        # Recording state
        self.is_recording = False
        self.recorded_data = []
        self.start_time = None
        self.lock = threading.Lock()

        # Data directory
        self.data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        os.makedirs(self.data_dir, exist_ok=True)

        # Subscribers for real robot data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/franka/joint_states',
            self.joint_state_callback,
            10
        )

        self.gripper_state_sub = self.create_subscription(
            JointState,
            '/franka_gripper/joint_states',
            self.gripper_state_callback,
            10
        )

        # Service to start/stop recording
        self.record_srv = self.create_service(
            SetBool,
            'franka_recorder/start_stop',
            self.handle_recording
        )

        self.get_logger().info('Franka Recorder Node initialized')
        self.get_logger().info(f'Recording directory: {self.data_dir}')

    def joint_state_callback(self, msg):
        """Callback for robot joint states"""
        if self.is_recording:
            with self.lock:
                timestamp = self.get_clock().now().to_msg()
                elapsed = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
                self.recorded_data.append({
                    'type': 'joint_state',
                    'timestamp': timestamp,
                    'elapsed': elapsed,
                    'data': {
                        'header': msg.header,
                        'name': msg.name,
                        'position': list(msg.position),
                        'velocity': list(msg.velocity),
                        'effort': list(msg.effort)
                    }
                })

    def gripper_state_callback(self, msg):
        """Callback for gripper joint states"""
        if self.is_recording:
            with self.lock:
                timestamp = self.get_clock().now().to_msg()
                elapsed = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
                self.recorded_data.append({
                    'type': 'gripper_state',
                    'timestamp': timestamp,
                    'elapsed': elapsed,
                    'data': {
                        'header': msg.header,
                        'name': msg.name,
                        'position': list(msg.position),
                        'velocity': list(msg.velocity),
                        'effort': list(msg.effort)
                    }
                })

    def handle_recording(self, request, response):
        """Service handler to start/stop recording"""
        with self.lock:
            if request.data:  # Start recording
                if self.is_recording:
                    response.success = False
                    response.message = 'Already recording'
                else:
                    self.is_recording = True
                    self.recorded_data = []
                    self.start_time = self.get_clock().now().nanoseconds
                    response.success = True
                    response.message = 'Recording started'
                    self.get_logger().info('Recording started')
            else:  # Stop recording
                if not self.is_recording:
                    response.success = False
                    response.message = 'Not currently recording'
                else:
                    self.is_recording = False
                    if self.recorded_data:
                        filename = self.save_recording()
                        response.success = True
                        response.message = f'Recording stopped and saved to {filename}'
                        self.get_logger().info(f'Recording stopped. Saved {len(self.recorded_data)} samples to {filename}')
                    else:
                        response.success = True
                        response.message = 'Recording stopped (no data recorded)'
                        self.get_logger().warn('Recording stopped but no data was recorded')

        return response

    def save_recording(self):
        """Save recorded data to a pickle file"""
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'franka_recording_{timestamp_str}.pkl'
        filepath = os.path.join(self.data_dir, filename)

        recording = {
            'metadata': {
                'timestamp': timestamp_str,
                'sample_count': len(self.recorded_data),
                'duration': self.recorded_data[-1]['elapsed'] if self.recorded_data else 0
            },
            'data': self.recorded_data
        }

        with open(filepath, 'wb') as f:
            pickle.dump(recording, f)

        return filename


def main(args=None):
    rclpy.init(args=args)
    node = FrankaRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
