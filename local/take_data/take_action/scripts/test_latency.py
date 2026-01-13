#!/usr/bin/env python3
"""
Test data transmission latency
Measures end-to-end delay from ROS2 topic to local reception
"""

import socket
import json
import time
import argparse
import numpy as np
from datetime import datetime


def test_latency(host='172.16.1.2', port=9999, duration=30):
    """Test latency by comparing remote timestamp with local reception time"""

    print(f"Connecting to {host}:{port}...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        print(f"✓ Connected\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return

    print(f"Measuring latency for {duration} seconds...\n")
    print(f"{'Time':<12} {'Latency (ms)':<15} {'Rate (Hz)':<12} {'Data Size (B)':<15}")
    print("=" * 65)

    buffer = ''
    latencies = []
    data_sizes = []
    packet_count = 0
    start_time = time.time()
    last_print_time = start_time

    try:
        while time.time() - start_time < duration:
            # Receive data
            chunk = sock.recv(4096).decode('utf-8')
            if not chunk:
                break

            buffer += chunk
            reception_time = time.time()  # Record when we received it

            # Process complete messages
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)

                if line.strip():
                    try:
                        data = json.loads(line)
                        packet_count += 1

                        # Calculate latency
                        remote_timestamp = data.get('timestamp', 0)
                        if remote_timestamp > 0:
                            latency_ms = (reception_time - remote_timestamp) * 1000
                            latencies.append(latency_ms)
                            data_sizes.append(len(line))

                            # Print every second
                            if reception_time - last_print_time >= 1.0:
                                elapsed = reception_time - start_time
                                rate = packet_count / elapsed if elapsed > 0 else 0

                                print(f"{elapsed:>6.1f}s      "
                                      f"{latency_ms:>8.2f}          "
                                      f"{rate:>8.1f}      "
                                      f"{len(line):>10d}")

                                last_print_time = reception_time

                    except json.JSONDecodeError:
                        pass

    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        sock.close()

    # Statistics
    if latencies:
        latencies = np.array(latencies)
        data_sizes = np.array(data_sizes)
        total_time = time.time() - start_time

        print("\n" + "=" * 65)
        print("LATENCY STATISTICS")
        print("=" * 65)
        print(f"Total packets:     {packet_count}")
        print(f"Total duration:    {total_time:.2f} seconds")
        print(f"Average rate:      {packet_count/total_time:.1f} Hz")
        print(f"\nLatency (ms):")
        print(f"  Minimum:         {np.min(latencies):.2f}")
        print(f"  Maximum:         {np.max(latencies):.2f}")
        print(f"  Mean:            {np.mean(latencies):.2f}")
        print(f"  Median:          {np.median(latencies):.2f}")
        print(f"  Std Dev:         {np.std(latencies):.2f}")
        print(f"  95th percentile: {np.percentile(latencies, 95):.2f}")
        print(f"  99th percentile: {np.percentile(latencies, 99):.2f}")

        print(f"\nData Size (bytes):")
        print(f"  Minimum:         {np.min(data_sizes)}")
        print(f"  Maximum:         {np.max(data_sizes)}")
        print(f"  Mean:            {np.mean(data_sizes):.0f}")
        print(f"  Total:           {np.sum(data_sizes)/1024:.2f} KB")

        print(f"\nNetwork:")
        print(f"  Bandwidth:       {np.sum(data_sizes)/1024/total_time:.2f} KB/s")
        print("=" * 65)

        # Latency distribution
        print(f"\nLatency Distribution:")
        bins = [0, 5, 10, 20, 50, 100, float('inf')]
        labels = ['<5ms', '5-10ms', '10-20ms', '20-50ms', '50-100ms', '>100ms']

        for i, (low, high, label) in enumerate(zip(bins[:-1], bins[1:], labels)):
            count = np.sum((latencies >= low) & (latencies < high))
            percent = count / len(latencies) * 100
            bar = '█' * int(percent / 2)
            print(f"  {label:<10} {count:>6} ({percent:>5.1f}%)  {bar}")


def main():
    parser = argparse.ArgumentParser(description='Test data transmission latency')
    parser.add_argument('--host', type=str, default='172.16.1.2',
                        help='Remote host (default: 172.16.1.2)')
    parser.add_argument('--port', type=int, default=9999,
                        help='Remote port (default: 9999)')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds (default: 30)')

    args = parser.parse_args()

    test_latency(args.host, args.port, args.duration)


if __name__ == '__main__':
    main()
