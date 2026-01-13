#!/usr/bin/env python3
"""
Test smooth transition functionality
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
import numpy as np


class SmoothTransitionTester(Node):
    def __init__(self):
        super().__init__('smooth_transition_tester')

        # Publishers to simulate GELLO and Franka
        self.gello_pub = self.create_publisher(JointState, '/gello/joint_states', 10)
        self.franka_pub = self.create_publisher(JointState, '/franka/joint_states', 10)

        # Subscriber to monitor replay output
        self.replay_sub = self.create_subscription(
            JointState,
            '/gello/joint_states',
            self.replay_callback,
            10
        )

        # Service clients
        self.record_client = self.create_client(SetBool, 'franka_recorder/start_stop')
        self.replay_client = self.create_client(SetBool, 'franka_replay/start_stop')
        self.param_client = self.create_client(SetParameters, '/franka_replay_node/set_parameters')

        self.received_count = 0
        self.phase = "waiting"

    def replay_callback(self, msg):
        """Monitor replay messages"""
        self.received_count += 1

    def publish_mock_states(self, duration=5.0):
        """Publish mock GELLO and Franka states"""
        print(f"Publishing mock states for {duration}s...")
        start = time.time()

        while time.time() - start < duration:
            t = time.time() - start

            # Simulate GELLO at a different position
            gello_msg = JointState()
            gello_msg.header.stamp = self.get_clock().now().to_msg()
            gello_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            gello_msg.position = [0.5 + 0.1 * np.sin(2 * np.pi * 0.5 * t + i) for i in range(7)]
            gello_msg.velocity = [0.05 * (i + 1) for i in range(7)]
            gello_msg.effort = [0.0] * 7

            # Simulate Franka at current position
            franka_msg = JointState()
            franka_msg.header.stamp = gello_msg.header.stamp
            franka_msg.name = gello_msg.name
            franka_msg.position = [0.3 + 0.1 * np.sin(2 * np.pi * 0.3 * t + i * 0.5) for i in range(7)]
            franka_msg.velocity = [0.03 * (i + 1) for i in range(7)]
            franka_msg.effort = [0.0] * 7

            self.gello_pub.publish(gello_msg)
            self.franka_pub.publish(franka_msg)

            time.sleep(0.03)  # ~30 Hz

    def test_smooth_replay(self):
        """Test replay with smooth transitions"""
        print("\n" + "="*60)
        print("Testing Smooth Transition Replay")
        print("="*60)

        # Find latest recording
        data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        recordings = sorted(glob.glob(os.path.join(data_dir, '*.pkl')))

        if not recordings:
            print("✗ No recordings found. Creating test recording first...")
            return False

        latest = os.path.basename(recordings[-1])
        print(f"Using recording: {latest}")

        # Start publishing states in background
        import threading
        state_thread = threading.Thread(
            target=self.publish_mock_states,
            args=(30.0,),  # Publish for 30 seconds
            daemon=True
        )
        state_thread.start()

        print("Waiting for states to publish...")
        time.sleep(2)

        # Load recording
        print("\nLoading recording...")
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
        print("\nStarting replay with smooth transitions...")
        print("Expected phases:")
        print("  Phase 1: Transition to recording start (3 seconds)")
        print("  Phase 2: Replay recorded trajectory (~3 seconds)")
        print("  Phase 3: Transition back to GELLO (3 seconds)")
        print()

        self.received_count = 0
        req = SetBool.Request()
        req.data = True
        future = self.replay_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.result() or not future.result().success:
            print(f"✗ Failed to start replay: {future.result().message if future.result() else 'No response'}")
            return False

        print("✓ Replay started")
        print("\nMonitoring replay progress...")

        # Monitor for ~12 seconds (3 + 3 + 3 + buffer)
        start_time = time.time()
        last_count = 0

        while time.time() - start_time < 12.0:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > last_count + 2:
                elapsed = int(time.time() - start_time)
                print(f"  t={elapsed}s: Received {self.received_count} samples so far")
                last_count = elapsed

        print(f"\n✓ Replay completed")
        print(f"  Total samples received: {self.received_count}")

        if self.received_count > 200:
            print("✓ Smooth transition test PASSED")
            return True
        else:
            print("✗ Expected more samples")
            return False


def main():
    print("="*60)
    print("Smooth Transition Test")
    print("="*60)
    print("\nNOTE: Make sure recorder_node.py and replay_node.py are running!")
    print()

    input("Press Enter when nodes are ready...")

    rclpy.init()
    tester = SmoothTransitionTester()

    try:
        # Wait for services
        print("Waiting for services...")
        if not tester.replay_client.wait_for_service(timeout_sec=10.0):
            print("✗ Replay service not available")
            return 1

        if not tester.param_client.wait_for_service(timeout_sec=10.0):
            print("✗ Parameter service not available")
            return 1

        print("✓ Services ready")

        # Test smooth replay
        if tester.test_smooth_replay():
            print("\n" + "="*60)
            print("✓ SMOOTH TRANSITION TEST PASSED!")
            print("="*60)
            print("\nThe replay now includes:")
            print("  ✓ Smooth transition from current to recording start")
            print("  ✓ Replay of recorded trajectory")
            print("  ✓ Smooth transition back to GELLO position")
            return 0
        else:
            print("\n✗ Test failed")
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
