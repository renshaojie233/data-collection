#!/usr/bin/env python3
"""
Test replay functionality
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time
import os
import glob


class ReplayTester(Node):
    def __init__(self):
        super().__init__('replay_tester')

        # Subscribe to gello topic to verify replay
        self.gello_sub = self.create_subscription(
            JointState,
            '/gello/joint_states',
            self.gello_callback,
            10
        )

        self.replay_client = self.create_client(SetBool, 'franka_replay/start_stop')
        self.param_client = self.create_client(SetParameters, '/franka_replay_node/set_parameters')

        self.received_count = 0

    def gello_callback(self, msg):
        """Callback for gello joint states"""
        self.received_count += 1
        if self.received_count % 30 == 0:
            print(f"  Received {self.received_count} replay samples")

    def test_replay(self):
        """Test replay functionality"""
        print("\n" + "="*60)
        print("Testing Replay")
        print("="*60)

        # Find latest recording
        data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        recordings = sorted(glob.glob(os.path.join(data_dir, '*.pkl')))

        if not recordings:
            print("✗ No recordings found")
            return False

        latest = os.path.basename(recordings[-1])
        print(f"Using recording: {latest}")

        # Set parameter to load the recording
        print("Loading recording...")
        param = Parameter()
        param.name = 'recording_file'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = latest

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.result() or not future.result().results[0].successful:
            print("✗ Failed to load recording")
            return False

        print("✓ Recording loaded")

        # Start replay
        print("Starting replay...")
        req = SetBool.Request()
        req.data = True
        future = self.replay_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.result() or not future.result().success:
            print("✗ Failed to start replay")
            return False

        print("✓ Replay started")
        print("Listening for replay data...")

        # Listen for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"\n✓ Received {self.received_count} total samples")

        if self.received_count > 0:
            return True
        else:
            print("✗ No replay data received")
            return False


def main():
    rclpy.init()
    tester = ReplayTester()

    try:
        # Wait for services
        print("Waiting for replay services...")
        if not tester.replay_client.wait_for_service(timeout_sec=10.0):
            print("✗ Replay service not available")
            return 1

        if not tester.param_client.wait_for_service(timeout_sec=10.0):
            print("✗ Parameter service not available")
            return 1

        print("✓ Services ready")

        # Test replay
        if tester.test_replay():
            print("\n" + "="*60)
            print("✓ REPLAY TEST PASSED!")
            print("="*60)
            return 0
        else:
            print("\n✗ Replay test failed")
            return 1

    except KeyboardInterrupt:
        print("\nTest interrupted")
        return 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
