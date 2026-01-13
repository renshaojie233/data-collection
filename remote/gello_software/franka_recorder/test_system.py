#!/usr/bin/env python3
"""
Test script to verify the complete recorder/replay system
Creates mock data and tests all components
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
import time
import os
import pickle
from datetime import datetime


class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')

        # Publisher to simulate robot data
        self.joint_pub = self.create_publisher(JointState, '/franka/joint_states', 10)
        self.gripper_pub = self.create_publisher(JointState, '/franka_gripper/joint_states', 10)

        # Service clients
        self.record_client = self.create_client(SetBool, 'franka_recorder/start_stop')
        self.replay_client = self.create_client(SetBool, 'franka_replay/start_stop')
        self.list_client = self.create_client(Trigger, 'franka_replay/list_recordings')

    def wait_for_services(self, timeout=5.0):
        """Wait for all services to be available"""
        print("Waiting for services...")
        services = [
            (self.record_client, 'recorder'),
            (self.replay_client, 'replay'),
            (self.list_client, 'list')
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout):
                print(f"  ✗ {name} service not available")
                return False
            print(f"  ✓ {name} service ready")
        return True

    def publish_mock_data(self, duration=3.0):
        """Publish mock robot data"""
        print(f"\nPublishing mock robot data for {duration}s...")
        start = time.time()
        count = 0

        while time.time() - start < duration:
            # Create joint state message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

            # Simulate simple sinusoidal motion
            t = time.time() - start
            msg.position = [0.1 * (i + 1) * t for i in range(7)]
            msg.velocity = [0.05 * (i + 1) for i in range(7)]
            msg.effort = [0.0] * 7

            self.joint_pub.publish(msg)

            # Gripper state
            gripper_msg = JointState()
            gripper_msg.header.stamp = msg.header.stamp
            gripper_msg.name = ['finger_joint1', 'finger_joint2']
            gripper_msg.position = [0.02, 0.02]
            gripper_msg.velocity = [0.0, 0.0]
            gripper_msg.effort = [0.0, 0.0]

            self.gripper_pub.publish(gripper_msg)

            count += 1
            time.sleep(0.03)  # ~30 Hz

        print(f"  Published {count} samples")
        return count

    def test_recording(self):
        """Test the recording functionality"""
        print("\n" + "="*60)
        print("Testing Recording")
        print("="*60)

        # Start recording
        req = SetBool.Request()
        req.data = True
        future = self.record_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            print("✓ Recording started")
        else:
            print("✗ Failed to start recording")
            return False

        # Publish data
        self.publish_mock_data(duration=3.0)

        # Stop recording
        req.data = False
        future = self.record_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            print(f"✓ Recording stopped: {future.result().message}")
            return True
        else:
            print("✗ Failed to stop recording")
            return False

    def test_list_recordings(self):
        """Test listing recordings"""
        print("\n" + "="*60)
        print("Testing List Recordings")
        print("="*60)

        req = Trigger.Request()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            print("✓ Available recordings:")
            for filename in future.result().message.split('\n'):
                print(f"    {filename}")
            return True
        else:
            print("✗ No recordings found")
            return False

    def verify_recording_file(self):
        """Verify that the recording file exists and is valid"""
        print("\n" + "="*60)
        print("Verifying Recording File")
        print("="*60)

        data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        import glob
        recordings = sorted(glob.glob(os.path.join(data_dir, '*.pkl')))

        if not recordings:
            print("✗ No recording files found")
            return False

        latest = recordings[-1]
        print(f"  Latest recording: {os.path.basename(latest)}")

        try:
            with open(latest, 'rb') as f:
                data = pickle.load(f)

            metadata = data.get('metadata', {})
            samples = data.get('data', [])

            print(f"  ✓ File loaded successfully")
            print(f"    Sample count: {metadata.get('sample_count', 0)}")
            print(f"    Duration: {metadata.get('duration', 0):.2f}s")
            print(f"    Actual samples: {len(samples)}")

            if len(samples) > 0:
                print(f"    First sample type: {samples[0]['type']}")
                print(f"    Joint names: {samples[0]['data']['name']}")
                return True
            else:
                print("  ✗ No samples in recording")
                return False

        except Exception as e:
            print(f"  ✗ Error loading file: {e}")
            return False


def main():
    print("="*60)
    print("Franka Recorder/Replay System Test")
    print("="*60)
    print("\nNOTE: Make sure recorder_node.py and replay_node.py are running!")
    print("      In separate terminals, run:")
    print("        Terminal 1: python3 recorder_node.py")
    print("        Terminal 2: python3 replay_node.py")
    print()

    input("Press Enter when nodes are ready...")

    rclpy.init()
    tester = SystemTester()

    try:
        # Wait for services
        if not tester.wait_for_services(timeout=10.0):
            print("\n✗ Services not available. Make sure nodes are running.")
            return 1

        # Test recording
        if not tester.test_recording():
            print("\n✗ Recording test failed")
            return 1

        # Wait a bit for file to be written
        time.sleep(1)

        # Verify file
        if not tester.verify_recording_file():
            print("\n✗ Recording file verification failed")
            return 1

        # Test listing
        if not tester.test_list_recordings():
            print("\n✗ List recordings test failed")
            return 1

        print("\n" + "="*60)
        print("✓ ALL TESTS PASSED!")
        print("="*60)
        print("\nThe system is working correctly!")
        print("You can now use the GUI: python3 control_gui.py")
        return 0

    except KeyboardInterrupt:
        print("\nTest interrupted")
        return 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
