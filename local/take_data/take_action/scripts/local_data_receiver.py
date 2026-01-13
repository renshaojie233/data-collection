#!/usr/bin/env python3
"""
Local Data Receiver
Receives robot data from remote bridge and saves to files
"""

import socket
import json
import h5py
import numpy as np
from datetime import datetime
from pathlib import Path
import argparse
import sys
from threading import Thread, Lock
import time


class DataReceiver:
    def __init__(self, host='172.16.1.2', port=9999, save_dir='../data'):
        self.host = host
        self.port = port
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # Data storage
        self.data_buffer = []
        self.buffer_lock = Lock()
        self.running = True

        # Statistics
        self.packet_count = 0
        self.start_time = None

        # Files
        self.session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.json_file = self.save_dir / f'session_{self.session_name}.json'
        self.hdf5_file = self.save_dir / f'session_{self.session_name}.h5'

        print(f'Data will be saved to:')
        print(f'  JSON: {self.json_file}')
        print(f'  HDF5: {self.hdf5_file}')

    def connect(self):
        """Connect to remote TCP server"""
        print(f'Connecting to {self.host}:{self.port}...')

        retry_count = 0
        max_retries = 10

        while retry_count < max_retries:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print(f'✓ Connected to remote bridge at {self.host}:{self.port}')
                return True
            except ConnectionRefusedError:
                retry_count += 1
                print(f'Connection refused. Retrying {retry_count}/{max_retries}...')
                time.sleep(2)
            except Exception as e:
                print(f'Connection error: {e}')
                retry_count += 1
                time.sleep(2)

        print('Failed to connect after maximum retries')
        return False

    def receive_data(self):
        """Receive data stream from remote"""
        buffer = ''
        self.start_time = time.time()

        print('\n--- Streaming Data (Press Ctrl+C to stop) ---\n')

        try:
            while self.running:
                try:
                    # Receive data
                    chunk = self.socket.recv(4096).decode('utf-8')

                    if not chunk:
                        print('Connection closed by remote')
                        break

                    buffer += chunk

                    # Process complete messages (delimited by newline)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)

                        if line.strip():
                            try:
                                data = json.loads(line)
                                self.process_data(data)
                            except json.JSONDecodeError as e:
                                print(f'JSON decode error: {e}')

                except socket.timeout:
                    continue
                except Exception as e:
                    print(f'Receive error: {e}')
                    break

        except KeyboardInterrupt:
            print('\nStopping data collection...')
        finally:
            self.running = False

    def process_data(self, data):
        """Process received data packet"""
        with self.buffer_lock:
            self.data_buffer.append(data)
            self.packet_count += 1

        # Display real-time info
        if self.packet_count % 10 == 0:  # Update every 10 packets
            self.display_status(data)

    def display_status(self, data):
        """Display real-time data status"""
        elapsed = time.time() - self.start_time
        rate = self.packet_count / elapsed if elapsed > 0 else 0

        print(f'\r[{self.packet_count:6d} packets | {rate:6.1f} Hz] ', end='')

        # Display joint positions
        if data.get('franka_joints') and data['franka_joints'].get('position'):
            positions = data['franka_joints']['position']
            pos_str = ' '.join([f'{p:6.3f}' for p in positions[:7]])
            print(f'Franka: [{pos_str}]', end='')

        sys.stdout.flush()

    def save_data(self):
        """Save collected data to files"""
        print(f'\n\nSaving {len(self.data_buffer)} data points...')

        with self.buffer_lock:
            # Save to JSON
            print(f'Saving to JSON: {self.json_file}')
            with open(self.json_file, 'w') as f:
                json.dump({
                    'session_name': self.session_name,
                    'start_time': self.start_time,
                    'packet_count': self.packet_count,
                    'data': self.data_buffer
                }, f, indent=2)

            # Save to HDF5
            print(f'Saving to HDF5: {self.hdf5_file}')
            self.save_hdf5()

        print(f'✓ Data saved successfully!')
        print(f'  Total packets: {self.packet_count}')
        print(f'  Duration: {time.time() - self.start_time:.2f} seconds')

    def save_hdf5(self):
        """Save data to HDF5 format for efficient storage"""
        with h5py.File(self.hdf5_file, 'w') as f:
            # Metadata
            f.attrs['session_name'] = self.session_name
            f.attrs['start_time'] = self.start_time
            f.attrs['packet_count'] = self.packet_count

            # Extract arrays
            timestamps = []
            gello_positions = []
            franka_positions = []
            franka_velocities = []
            franka_efforts = []
            ee_positions = []
            ee_orientations = []

            for data in self.data_buffer:
                timestamps.append(data.get('timestamp', 0))

                # GELLO joints
                if data.get('gello_joints') and data['gello_joints'].get('position'):
                    gello_positions.append(data['gello_joints']['position'])
                else:
                    gello_positions.append([0] * 7)

                # Franka joints
                if data.get('franka_joints'):
                    franka_positions.append(data['franka_joints'].get('position', [0] * 7))
                    franka_velocities.append(data['franka_joints'].get('velocity', [0] * 7))
                    franka_efforts.append(data['franka_joints'].get('effort', [0] * 7))
                else:
                    franka_positions.append([0] * 7)
                    franka_velocities.append([0] * 7)
                    franka_efforts.append([0] * 7)

                # End effector pose
                if data.get('end_effector_pose'):
                    pos = data['end_effector_pose']['position']
                    ori = data['end_effector_pose']['orientation']
                    ee_positions.append([pos['x'], pos['y'], pos['z']])
                    ee_orientations.append([ori['x'], ori['y'], ori['z'], ori['w']])
                else:
                    ee_positions.append([0, 0, 0])
                    ee_orientations.append([0, 0, 0, 1])

            # Save arrays to HDF5
            f.create_dataset('timestamps', data=np.array(timestamps))
            f.create_dataset('gello/positions', data=np.array(gello_positions))
            f.create_dataset('franka/positions', data=np.array(franka_positions))
            f.create_dataset('franka/velocities', data=np.array(franka_velocities))
            f.create_dataset('franka/efforts', data=np.array(franka_efforts))
            f.create_dataset('end_effector/positions', data=np.array(ee_positions))
            f.create_dataset('end_effector/orientations', data=np.array(ee_orientations))

    def run(self):
        """Main run loop"""
        if not self.connect():
            return False

        try:
            self.receive_data()
        finally:
            self.save_data()
            self.socket.close()

        return True


def main():
    parser = argparse.ArgumentParser(description='Receive and save robot data from remote bridge')
    parser.add_argument('--host', type=str, default='172.16.1.2',
                        help='Remote host IP address (default: 172.16.1.2)')
    parser.add_argument('--port', type=int, default=9999,
                        help='Remote port (default: 9999)')
    parser.add_argument('--save-dir', type=str, default='../data',
                        help='Directory to save data (default: ../data)')

    args = parser.parse_args()

    receiver = DataReceiver(
        host=args.host,
        port=args.port,
        save_dir=args.save_dir
    )

    success = receiver.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
