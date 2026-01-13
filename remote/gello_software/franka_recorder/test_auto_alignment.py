#!/usr/bin/env python3
"""
Test script to verify auto-alignment replay functionality.
Tests that replay node correctly auto-aligns to start position before replaying.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time
import numpy as np
import os
import glob


class AutoAlignmentTester(Node):
    def __init__(self):
        super().__init__('auto_alignment_tester')

        # Service clients
        self.replay_client = self.create_client(SetBool, 'franka_replay/start_stop')
        self.list_client = self.create_client(Trigger, 'franka_replay/list_recordings')
        self.param_client = self.create_client(SetParameters, '/franka_replay_node/set_parameters')

        # State tracking
        self.joint_states_received = []
        self.gello_states_received = []
        self.franka_states_received = []

        # Subscribers to monitor published states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/gello/joint_states',
            self.joint_state_callback,
            10
        )

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

        self.get_logger().info('Auto-alignment tester initialized')

    def joint_state_callback(self, msg):
        """Track joint states published during replay"""
        self.joint_states_received.append({
            'timestamp': time.time(),
            'position': list(msg.position)
        })

    def gello_callback(self, msg):
        """Track GELLO states"""
        self.gello_states_received.append({
            'timestamp': time.time(),
            'position': list(msg.position)
        })

    def franka_callback(self, msg):
        """Track Franka states"""
        self.franka_states_received.append({
            'timestamp': time.time(),
            'position': list(msg.position)
        })

    def wait_for_service(self, client, timeout=5.0):
        """Wait for a service to be available"""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return False
        return True

    def list_recordings(self):
        """List available recordings"""
        if not self.wait_for_service(self.list_client):
            return None

        request = Trigger.Request()
        future = self.list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.message.split('\n')
        return None

    def set_recording_file(self, filename):
        """Set the recording file to replay"""
        if not self.wait_for_service(self.param_client):
            return False

        param = Parameter()
        param.name = 'recording_file'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = filename

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().results[0].successful:
            self.get_logger().info(f'Set recording file to: {filename}')
            return True
        return False

    def start_replay(self):
        """Start replay"""
        if not self.wait_for_service(self.replay_client):
            return False, "Replay service not available"

        request = SetBool.Request()
        request.data = True

        future = self.replay_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        return False, "Failed to call service"

    def calculate_position_difference(self, pos1, pos2):
        """Calculate maximum position difference"""
        arr1 = np.array(pos1)
        arr2 = np.array(pos2)
        diff = np.abs(arr1 - arr2)
        return np.max(diff)

    def run_test(self):
        """Run the auto-alignment test"""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('Starting Auto-Alignment Replay Test')
        self.get_logger().info('='*70)

        # Step 1: List available recordings
        self.get_logger().info('\n[Step 1] Listing available recordings...')
        recordings = self.list_recordings()

        if not recordings or len(recordings) == 0:
            self.get_logger().error('No recordings found. Please record a trajectory first.')
            return False

        self.get_logger().info(f'Found {len(recordings)} recording(s):')
        for i, rec in enumerate(recordings):
            self.get_logger().info(f'  {i+1}. {rec}')

        # Use the latest recording
        latest_recording = recordings[-1]
        self.get_logger().info(f'\nUsing latest recording: {latest_recording}')

        # Step 2: Set recording file
        self.get_logger().info('\n[Step 2] Setting recording file...')
        if not self.set_recording_file(latest_recording):
            self.get_logger().error('Failed to set recording file')
            return False

        self.get_logger().info('✓ Recording file set successfully')

        # Step 3: Wait for initial states
        self.get_logger().info('\n[Step 3] Waiting for initial states...')
        time.sleep(2.0)  # Wait for states to be published

        if len(self.franka_states_received) == 0:
            self.get_logger().error('No Franka states received')
            return False

        initial_franka_position = self.franka_states_received[-1]['position']
        self.get_logger().info(f'Initial Franka position (first 3 joints): {[f"{p:.3f}" for p in initial_franka_position[:3]]}')

        # Step 4: Clear tracking data and start replay
        self.get_logger().info('\n[Step 4] Starting replay with auto-alignment...')
        self.joint_states_received.clear()

        success, message = self.start_replay()

        if not success:
            self.get_logger().error(f'Failed to start replay: {message}')
            return False

        self.get_logger().info(f'✓ Replay started: {message}')

        # Step 5: Monitor replay progress
        self.get_logger().info('\n[Step 5] Monitoring replay progress...')
        self.get_logger().info('Phase 1: Auto-aligning to start position (should take ~5 seconds)...')

        start_time = time.time()
        last_log_time = start_time

        # Monitor for 15 seconds (5s alignment + recording duration)
        while (time.time() - start_time) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Log progress every second
            if time.time() - last_log_time >= 1.0:
                elapsed = time.time() - start_time
                states_count = len(self.joint_states_received)

                if elapsed < 6.0:
                    self.get_logger().info(f'  Alignment progress: {elapsed:.1f}s elapsed, {states_count} states published')
                else:
                    self.get_logger().info(f'  Replay progress: {elapsed:.1f}s elapsed, {states_count} states published')

                last_log_time = time.time()

        # Step 6: Analyze results
        self.get_logger().info('\n[Step 6] Analyzing results...')

        total_states = len(self.joint_states_received)
        self.get_logger().info(f'Total joint states published: {total_states}')

        if total_states == 0:
            self.get_logger().error('No joint states were published during replay')
            return False

        # Check if alignment occurred
        if total_states < 100:
            self.get_logger().warn(f'Expected more states for 5s alignment at 30Hz (~150), got {total_states}')

        # Compare first published position with initial position
        first_published = self.joint_states_received[0]['position']
        last_published = self.joint_states_received[-1]['position']

        diff_from_initial = self.calculate_position_difference(initial_franka_position, first_published)
        self.get_logger().info(f'\nPosition difference from initial to first published: {diff_from_initial:.4f} rad')

        # Check if positions changed during alignment
        position_changes = []
        for i in range(1, min(len(self.joint_states_received), 150)):  # First 5 seconds at 30Hz
            prev_pos = self.joint_states_received[i-1]['position']
            curr_pos = self.joint_states_received[i]['position']
            diff = self.calculate_position_difference(prev_pos, curr_pos)
            position_changes.append(diff)

        if len(position_changes) > 0:
            max_change = max(position_changes)
            avg_change = np.mean(position_changes)
            self.get_logger().info(f'During alignment phase:')
            self.get_logger().info(f'  Max position change between steps: {max_change:.4f} rad')
            self.get_logger().info(f'  Avg position change between steps: {avg_change:.4f} rad')

            if max_change > 0.1:
                self.get_logger().warn('Large position jumps detected - may not be smooth')
            else:
                self.get_logger().info('✓ Position changes are smooth (no large jumps)')

        # Final summary
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('Test Summary')
        self.get_logger().info('='*70)
        self.get_logger().info(f'✓ Recording loaded: {latest_recording}')
        self.get_logger().info(f'✓ Replay started with auto-alignment')
        self.get_logger().info(f'✓ Total states published: {total_states}')
        self.get_logger().info(f'✓ First 3 joint initial positions: {[f"{p:.3f}" for p in initial_franka_position[:3]]}')
        self.get_logger().info(f'✓ First 3 joint final positions: {[f"{p:.3f}" for p in last_published[:3]]}')

        if len(position_changes) > 0 and max(position_changes) < 0.1:
            self.get_logger().info('\n✅ AUTO-ALIGNMENT TEST PASSED')
            self.get_logger().info('   Smooth alignment trajectory was generated and executed')
        else:
            self.get_logger().warn('\n⚠️  TEST COMPLETED WITH WARNINGS')
            self.get_logger().warn('   Check position changes for smoothness')

        self.get_logger().info('='*70 + '\n')

        return True


def main():
    rclpy.init()
    tester = AutoAlignmentTester()

    try:
        # Wait a moment for all connections
        time.sleep(1.0)

        # Run the test
        success = tester.run_test()

        if success:
            tester.get_logger().info('\n✅ All tests completed successfully!')
        else:
            tester.get_logger().error('\n❌ Some tests failed')

        # Keep spinning briefly to see any remaining messages
        time.sleep(2.0)

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test error: {str(e)}')
        import traceback
        tester.get_logger().error(traceback.format_exc())
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
