#!/usr/bin/env python3
"""
ROS2 node to replay recorded Franka robot arm trajectories.
Features:
- Automatically moves robot to recording start position
- Pauses GELLO control during replay
- Replays recorded trajectory
- Returns control to GELLO after completion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
import pickle
import os
import glob
import threading
import time
import numpy as np
from motion_generator import MotionGenerator


class FrankaReplayNode(Node):
    def __init__(self):
        super().__init__('franka_replay_node')

        # Replay state
        self.is_replaying = False
        self.replay_data = None
        self.replay_thread = None
        self.lock = threading.Lock()
        self.stop_replay_flag = False

        # Current states
        self.current_gello_state = None
        self.current_franka_state = None
        self.gello_lock = threading.Lock()
        self.franka_lock = threading.Lock()

        # Data directory
        self.data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        os.makedirs(self.data_dir, exist_ok=True)

        # Publishers for controlling the robot
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/gello/joint_states',
            10
        )

        # Subscribers to monitor current states
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

        # Service to start replay
        self.replay_srv = self.create_service(
            SetBool,
            'franka_replay/start_stop',
            self.handle_replay
        )

        # Service to list available recordings
        self.list_srv = self.create_service(
            Trigger,
            'franka_replay/list_recordings',
            self.handle_list_recordings
        )

        # Parameter to specify which recording to load
        self.declare_parameter('recording_file', '')

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Motion parameters (same as FR3 controller)
        self.speed_factor = 0.2  # Speed factor for motion generator (0.2 = 20% max speed)
        self.control_rate = 30  # Hz for control loop

        self.get_logger().info('Franka Replay Node initialized')
        self.get_logger().info(f'Recording directory: {self.data_dir}')
        self.get_logger().info(f'Using MotionGenerator with speed_factor={self.speed_factor}')

    def gello_callback(self, msg):
        """Store current GELLO state"""
        with self.gello_lock:
            self.current_gello_state = msg

    def franka_callback(self, msg):
        """Store current Franka state"""
        with self.franka_lock:
            self.current_franka_state = msg


    def calculate_position_difference(self, pos1, pos2):
        """Calculate maximum position difference"""
        arr1 = np.array(pos1)
        arr2 = np.array(pos2)
        diff = np.abs(arr1 - arr2)
        return np.max(diff)

    def handle_list_recordings(self, request, response):
        """List all available recordings"""
        recordings = sorted(glob.glob(os.path.join(self.data_dir, '*.pkl')))
        if recordings:
            filenames = [os.path.basename(f) for f in recordings]
            response.success = True
            response.message = '\n'.join(filenames)
        else:
            response.success = False
            response.message = 'No recordings found'
        return response

    def parameter_callback(self, params):
        """Callback when parameters are changed"""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'recording_file' and param.value:
                filename = param.value
                if self.load_recording_by_name(filename):
                    return SetParametersResult(successful=True)
                else:
                    return SetParametersResult(successful=False, reason=f'Failed to load {filename}')

        return SetParametersResult(successful=True)

    def load_recording_by_name(self, filename):
        """Load a specific recording file by name"""
        filepath = os.path.join(self.data_dir, filename)

        if not os.path.exists(filepath):
            self.get_logger().error(f'File not found: {filename}')
            return False

        try:
            with open(filepath, 'rb') as f:
                self.replay_data = pickle.load(f)

            metadata = self.replay_data.get('metadata', {})
            self.get_logger().info(f'Loaded {filename}: {metadata.get("sample_count", 0)} samples, {metadata.get("duration", 0):.2f}s')
            return True
        except Exception as e:
            self.get_logger().error(f'Error loading file: {str(e)}')
            return False

    def handle_replay(self, request, response):
        """Service handler to start/stop replay"""
        with self.lock:
            if request.data:  # Start replay
                if self.is_replaying:
                    response.success = False
                    response.message = 'Already replaying'
                elif self.replay_data is None:
                    response.success = False
                    response.message = 'No recording loaded'
                else:
                    # Check if we have current states
                    with self.franka_lock:
                        if self.current_franka_state is None:
                            response.success = False
                            response.message = 'Waiting for Franka state'
                            return response

                    with self.gello_lock:
                        if self.current_gello_state is None:
                            response.success = False
                            response.message = 'Waiting for GELLO state'
                            return response

                    # Check if recording has data
                    data = self.replay_data['data']
                    joint_states = [d for d in data if d['type'] == 'joint_state']
                    if not joint_states:
                        response.success = False
                        response.message = 'No joint state data to replay'
                        return response

                    # Calculate position difference for info
                    with self.franka_lock:
                        current_pos = list(self.current_franka_state.position)
                    start_pos = joint_states[0]['data']['position']
                    max_diff = self.calculate_position_difference(current_pos, start_pos)

                    self.is_replaying = True
                    self.stop_replay_flag = False
                    self.replay_thread = threading.Thread(target=self.replay_loop)
                    self.replay_thread.start()
                    response.success = True
                    response.message = f'Replay started (will auto-align from {max_diff:.3f} rad difference)'
                    self.get_logger().info(f'Replay started, auto-alignment needed: {max_diff:.3f} rad')
            else:  # Stop replay
                if not self.is_replaying:
                    response.success = False
                    response.message = 'Not currently replaying'
                else:
                    self.stop_replay_flag = True
                    if self.replay_thread:
                        self.replay_thread.join(timeout=5.0)
                    self.is_replaying = False
                    response.success = True
                    response.message = 'Replay stopped'
                    self.get_logger().info('Replay stopped')

        return response

    def replay_loop(self):
        """
        Main replay loop with automatic alignment
        Phase 1: Move to recording start position
        Phase 2: Replay recorded trajectory
        """
        try:
            data = self.replay_data['data']
            joint_states = [d for d in data if d['type'] == 'joint_state']

            if not joint_states:
                self.get_logger().warn('No joint state data to replay')
                return

            self.get_logger().info(f'Starting replay of {len(joint_states)} samples')

            # Get recording start state
            recording_start_state = joint_states[0]['data']

            # Get current Franka state
            with self.franka_lock:
                current_franka = {
                    'name': list(self.current_franka_state.name),
                    'position': list(self.current_franka_state.position),
                    'velocity': list(self.current_franka_state.velocity) if self.current_franka_state.velocity else [0.0] * len(self.current_franka_state.position),
                    'effort': list(self.current_franka_state.effort) if self.current_franka_state.effort else [0.0] * len(self.current_franka_state.position)
                }

            # Calculate position difference
            max_diff = self.calculate_position_difference(
                current_franka['position'],
                recording_start_state['position']
            )

            # Phase 1: Auto-align to recording start position using MotionGenerator
            # (Same algorithm as FR3 controller startup)
            if max_diff > 0.01:  # Only align if difference > 0.01 rad
                self.get_logger().info(f'Phase 1: Auto-aligning to start position (diff: {max_diff:.3f} rad)...')

                # Create MotionGenerator (same as FR3 controller)
                q_start = np.array(current_franka['position'])
                q_goal = np.array(recording_start_state['position'])
                motion_gen = MotionGenerator(self.speed_factor, q_start, q_goal)

                duration = motion_gen.get_trajectory_duration()
                self.get_logger().info(f'Motion duration: {duration:.2f}s (speed_factor={self.speed_factor})')

                # Execute alignment trajectory
                dt = 1.0 / self.control_rate
                start_time = time.time()
                iteration = 0

                while True:
                    if self.stop_replay_flag:
                        self.get_logger().info('Replay interrupted during alignment')
                        return

                    # Calculate elapsed time
                    elapsed = time.time() - start_time

                    # Get desired position from motion generator
                    q_desired, motion_finished = motion_gen.get_desired_joint_positions(elapsed)

                    # Create and publish joint state message
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = recording_start_state['name']
                    msg.position = list(q_desired)
                    msg.velocity = [0.0] * len(q_desired)
                    msg.effort = [0.0] * len(q_desired)

                    self.joint_state_pub.publish(msg)

                    # Log progress every second
                    if iteration % self.control_rate == 0:
                        progress = min(100.0, (elapsed / duration) * 100)
                        self.get_logger().info(f'Alignment progress: {progress:.1f}%')

                    iteration += 1

                    # Check if motion is finished
                    if motion_finished:
                        self.get_logger().info('✓ Alignment complete, ready to replay')
                        break

                    time.sleep(dt)
            else:
                self.get_logger().info('Already at start position, skipping alignment')

            # Small pause to stabilize
            time.sleep(0.5)

            # Phase 2: Replay the recorded trajectory
            self.get_logger().info('Phase 2: Replaying recorded trajectory...')

            start_time = time.time()
            for i, sample in enumerate(joint_states):
                if self.stop_replay_flag:
                    self.get_logger().info('Replay interrupted during playback')
                    return

                # Calculate timing
                target_elapsed = sample['elapsed']
                current_elapsed = time.time() - start_time

                # Sleep to match original timing
                sleep_time = target_elapsed - current_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                # Publish joint state
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = sample['data']['name']
                msg.position = sample['data']['position']
                msg.velocity = sample['data']['velocity']
                msg.effort = sample['data']['effort']

                self.joint_state_pub.publish(msg)

                if i % 100 == 0:
                    self.get_logger().info(f'Replay progress: {i}/{len(joint_states)} samples')

            self.get_logger().info('✓ Replay completed successfully')
            self.get_logger().info('Robot remains at end position. GELLO control is now active.')

        except Exception as e:
            self.get_logger().error(f'Error during replay: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            with self.lock:
                self.is_replaying = False

    def load_latest_recording(self):
        """Automatically load the most recent recording"""
        recordings = sorted(glob.glob(os.path.join(self.data_dir, '*.pkl')))
        if recordings:
            latest = recordings[-1]
            try:
                with open(latest, 'rb') as f:
                    self.replay_data = pickle.load(f)
                self.get_logger().info(f'Auto-loaded latest recording: {os.path.basename(latest)}')
                return True
            except Exception as e:
                self.get_logger().error(f'Error auto-loading recording: {str(e)}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FrankaReplayNode()

    # Try to auto-load the latest recording
    node.load_latest_recording()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
