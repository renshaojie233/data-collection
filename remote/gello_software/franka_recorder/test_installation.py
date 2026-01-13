#!/usr/bin/env python3
"""
Test script to verify Franka Recorder installation
"""

import sys
import os

def test_imports():
    """Test if all required modules can be imported"""
    print("Testing Python imports...")

    try:
        import rclpy
        print("  ✓ rclpy imported successfully")
    except ImportError as e:
        print(f"  ✗ Failed to import rclpy: {e}")
        print("    Install with: pip3 install rclpy")
        return False

    try:
        import tkinter as tk
        print("  ✓ tkinter imported successfully")
    except ImportError as e:
        print(f"  ✗ Failed to import tkinter: {e}")
        print("    Install with: sudo apt-get install python3-tk")
        return False

    try:
        import pickle
        print("  ✓ pickle imported successfully")
    except ImportError as e:
        print(f"  ✗ Failed to import pickle: {e}")
        return False

    try:
        from sensor_msgs.msg import JointState
        print("  ✓ sensor_msgs imported successfully")
    except ImportError as e:
        print(f"  ✗ Failed to import sensor_msgs: {e}")
        print("    Make sure ROS2 is sourced")
        return False

    try:
        from std_srvs.srv import SetBool, Trigger
        print("  ✓ std_srvs imported successfully")
    except ImportError as e:
        print(f"  ✗ Failed to import std_srvs: {e}")
        return False

    return True

def test_files():
    """Test if all required files exist"""
    print("\nTesting file structure...")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    required_files = [
        'recorder_node.py',
        'replay_node.py',
        'control_gui.py',
        'launch_recorder.sh',
        'environment.yaml',
        'README.md'
    ]

    all_exist = True
    for filename in required_files:
        filepath = os.path.join(script_dir, filename)
        if os.path.exists(filepath):
            print(f"  ✓ {filename} exists")
        else:
            print(f"  ✗ {filename} not found")
            all_exist = False

    return all_exist

def test_executability():
    """Test if scripts are executable"""
    print("\nTesting script permissions...")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    scripts = [
        'recorder_node.py',
        'replay_node.py',
        'control_gui.py',
        'launch_recorder.sh'
    ]

    all_executable = True
    for script in scripts:
        filepath = os.path.join(script_dir, script)
        if os.access(filepath, os.X_OK):
            print(f"  ✓ {script} is executable")
        else:
            print(f"  ✗ {script} is not executable")
            print(f"    Run: chmod +x {filepath}")
            all_executable = False

    return all_executable

def test_directories():
    """Test if required directories exist or can be created"""
    print("\nTesting directories...")

    data_dir = os.path.expanduser('~/gello_software/franka_recordings')

    try:
        os.makedirs(data_dir, exist_ok=True)
        print(f"  ✓ Recording directory: {data_dir}")
    except Exception as e:
        print(f"  ✗ Failed to create directory: {e}")
        return False

    return True

def main():
    print("=" * 60)
    print("Franka Recorder Installation Test")
    print("=" * 60)
    print()

    results = []

    # Run tests
    results.append(("Imports", test_imports()))
    results.append(("Files", test_files()))
    results.append(("Permissions", test_executability()))
    results.append(("Directories", test_directories()))

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name:20s} {status}")
        if not passed:
            all_passed = False

    print()
    if all_passed:
        print("🎉 All tests passed! Installation is complete.")
        print("\nYou can now run:")
        print("  cd ~/gello_software")
        print("  ./run_fr3_with_recorder.sh")
        return 0
    else:
        print("⚠️  Some tests failed. Please fix the issues above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())
