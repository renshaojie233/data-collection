#!/usr/bin/env python3
"""
Data Visualization Tool
Visualize collected robot data from HDF5 files
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse


def plot_joint_trajectories(hdf5_file):
    """Plot joint position trajectories"""
    with h5py.File(hdf5_file, 'r') as f:
        timestamps = f['timestamps'][:]
        gello_pos = f['gello/positions'][:]
        franka_pos = f['franka/positions'][:]

        # Convert to relative time (seconds)
        time = timestamps - timestamps[0]

        # Create figure with subplots
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle(f'Joint Trajectories - {Path(hdf5_file).stem}', fontsize=16)

        joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4',
                       'Joint 5', 'Joint 6', 'Joint 7']

        for i, (ax, name) in enumerate(zip(axes, joint_names)):
            ax.plot(time, gello_pos[:, i], label='GELLO (Input)', alpha=0.7)
            ax.plot(time, franka_pos[:, i], label='Franka (Actual)', alpha=0.7)
            ax.set_ylabel(f'{name}\n(rad)', fontsize=10)
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel('Time (s)', fontsize=12)

        plt.tight_layout()
        return fig


def plot_end_effector_path(hdf5_file):
    """Plot end effector 3D path"""
    with h5py.File(hdf5_file, 'r') as f:
        ee_pos = f['end_effector/positions'][:]

        fig = plt.figure(figsize=(12, 10))

        # 3D trajectory
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.plot(ee_pos[:, 0], ee_pos[:, 1], ee_pos[:, 2], 'b-', alpha=0.6)
        ax1.scatter(ee_pos[0, 0], ee_pos[0, 1], ee_pos[0, 2],
                   c='g', s=100, label='Start', marker='o')
        ax1.scatter(ee_pos[-1, 0], ee_pos[-1, 1], ee_pos[-1, 2],
                   c='r', s=100, label='End', marker='x')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('End Effector 3D Path')
        ax1.legend()

        # XY projection
        ax2 = fig.add_subplot(222)
        ax2.plot(ee_pos[:, 0], ee_pos[:, 1], 'b-', alpha=0.6)
        ax2.scatter(ee_pos[0, 0], ee_pos[0, 1], c='g', s=100, marker='o')
        ax2.scatter(ee_pos[-1, 0], ee_pos[-1, 1], c='r', s=100, marker='x')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('XY Projection')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')

        # XZ projection
        ax3 = fig.add_subplot(223)
        ax3.plot(ee_pos[:, 0], ee_pos[:, 2], 'b-', alpha=0.6)
        ax3.scatter(ee_pos[0, 0], ee_pos[0, 2], c='g', s=100, marker='o')
        ax3.scatter(ee_pos[-1, 0], ee_pos[-1, 2], c='r', s=100, marker='x')
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('XZ Projection')
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')

        # YZ projection
        ax4 = fig.add_subplot(224)
        ax4.plot(ee_pos[:, 1], ee_pos[:, 2], 'b-', alpha=0.6)
        ax4.scatter(ee_pos[0, 1], ee_pos[0, 2], c='g', s=100, marker='o')
        ax4.scatter(ee_pos[-1, 1], ee_pos[-1, 2], c='r', s=100, marker='x')
        ax4.set_xlabel('Y (m)')
        ax4.set_ylabel('Z (m)')
        ax4.set_title('YZ Projection')
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')

        plt.suptitle(f'End Effector Path - {Path(hdf5_file).stem}', fontsize=16)
        plt.tight_layout()

        return fig


def plot_tracking_error(hdf5_file):
    """Plot tracking error between GELLO and Franka"""
    with h5py.File(hdf5_file, 'r') as f:
        timestamps = f['timestamps'][:]
        gello_pos = f['gello/positions'][:]
        franka_pos = f['franka/positions'][:]

        # Calculate error
        error = np.abs(franka_pos - gello_pos)
        time = timestamps - timestamps[0]

        # Create figure
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle(f'Tracking Error - {Path(hdf5_file).stem}', fontsize=16)

        # Error per joint
        for i in range(7):
            axes[0].plot(time, error[:, i], label=f'Joint {i+1}', alpha=0.7)

        axes[0].set_ylabel('Absolute Error (rad)', fontsize=12)
        axes[0].legend(loc='upper right', ncol=4, fontsize=8)
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Error per Joint')

        # Total RMS error
        rms_error = np.sqrt(np.mean(error**2, axis=1))
        axes[1].plot(time, rms_error, 'r-', linewidth=2)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].set_ylabel('RMS Error (rad)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title('Root Mean Square Error')

        # Add statistics
        mean_rms = np.mean(rms_error)
        max_rms = np.max(rms_error)
        axes[1].axhline(mean_rms, color='g', linestyle='--',
                       label=f'Mean: {mean_rms:.4f} rad')
        axes[1].legend(loc='upper right', fontsize=10)

        plt.tight_layout()

        return fig


def print_statistics(hdf5_file):
    """Print data statistics"""
    with h5py.File(hdf5_file, 'r') as f:
        timestamps = f['timestamps'][:]
        franka_pos = f['franka/positions'][:]
        franka_vel = f['franka/velocities'][:]

        duration = timestamps[-1] - timestamps[0]
        num_samples = len(timestamps)
        avg_rate = num_samples / duration if duration > 0 else 0

        print("\n" + "="*60)
        print(f"Data Statistics: {Path(hdf5_file).name}")
        print("="*60)
        print(f"Total samples:     {num_samples}")
        print(f"Duration:          {duration:.2f} seconds")
        print(f"Average rate:      {avg_rate:.1f} Hz")
        print(f"\nJoint Position Range (rad):")
        for i in range(7):
            min_val = np.min(franka_pos[:, i])
            max_val = np.max(franka_pos[:, i])
            range_val = max_val - min_val
            print(f"  Joint {i+1}: [{min_val:7.3f}, {max_val:7.3f}]  Range: {range_val:6.3f}")

        print(f"\nJoint Velocity Statistics (rad/s):")
        for i in range(7):
            max_vel = np.max(np.abs(franka_vel[:, i]))
            mean_vel = np.mean(np.abs(franka_vel[:, i]))
            print(f"  Joint {i+1}: Max={max_vel:6.3f}  Mean={mean_vel:6.3f}")

        print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Visualize robot data from HDF5 files')
    parser.add_argument('file', type=str, help='Path to HDF5 file')
    parser.add_argument('--joints', action='store_true',
                       help='Plot joint trajectories')
    parser.add_argument('--ee', action='store_true',
                       help='Plot end effector path')
    parser.add_argument('--error', action='store_true',
                       help='Plot tracking error')
    parser.add_argument('--save', action='store_true',
                       help='Save plots as PNG files')
    parser.add_argument('--all', action='store_true',
                       help='Generate all plots')

    args = parser.parse_args()

    hdf5_file = Path(args.file)
    if not hdf5_file.exists():
        print(f"Error: File not found: {hdf5_file}")
        return

    # Print statistics
    print_statistics(hdf5_file)

    # Generate plots
    if args.all:
        args.joints = args.ee = args.error = True

    if not (args.joints or args.ee or args.error):
        print("Please specify at least one plot type (--joints, --ee, --error) or use --all")
        return

    figures = []

    if args.joints:
        print("Generating joint trajectory plot...")
        fig = plot_joint_trajectories(hdf5_file)
        figures.append(('joints', fig))

    if args.ee:
        print("Generating end effector path plot...")
        fig = plot_end_effector_path(hdf5_file)
        figures.append(('ee_path', fig))

    if args.error:
        print("Generating tracking error plot...")
        fig = plot_tracking_error(hdf5_file)
        figures.append(('error', fig))

    # Save or show plots
    if args.save:
        output_dir = hdf5_file.parent / 'plots'
        output_dir.mkdir(exist_ok=True)

        base_name = hdf5_file.stem
        for name, fig in figures:
            output_file = output_dir / f'{base_name}_{name}.png'
            fig.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"Saved: {output_file}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
