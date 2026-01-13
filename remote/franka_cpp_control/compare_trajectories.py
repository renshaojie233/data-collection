#!/usr/bin/env python3
"""
Compare reference and recorded robot trajectories.
Plots joint positions, tracking errors, and gripper width.
"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path
import argparse


def load_reference_trajectory(json_path):
    """Load reference trajectory from the original JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)

    joint_names = ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                   "fr3_joint5", "fr3_joint6", "fr3_joint7"]

    times = []
    joint_positions = []
    gripper_widths = []

    # Parse first entry to get indices
    first_entry = data['data'][0]
    franka_names = first_entry['franka_joints']['names']
    joint_indices = [franka_names.index(name) for name in joint_names]

    gripper_names = first_entry['gripper_joints']['names']
    finger1_idx = gripper_names.index('fr3_finger_joint1')
    finger2_idx = gripper_names.index('fr3_finger_joint2')

    for entry in data['data']:
        times.append(entry['timestamp'])

        positions = entry['franka_joints']['position']
        joint_pos = [positions[idx] for idx in joint_indices]
        joint_positions.append(joint_pos)

        gripper_pos = entry['gripper_joints']['position']
        gripper_width = gripper_pos[finger1_idx] + gripper_pos[finger2_idx]
        gripper_widths.append(gripper_width)

    # Normalize time to start from 0
    times = np.array(times)
    times = times - times[0]

    return {
        'time': times,
        'joint_positions': np.array(joint_positions),
        'gripper_width': np.array(gripper_widths)
    }


def load_recorded_trajectory(json_path):
    """Load recorded trajectory from the output JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)

    times = []
    q_actual = []
    q_desired = []
    gripper_widths = []

    for entry in data['data']:
        times.append(entry['timestamp'])
        q_actual.append(entry['q_actual'])
        q_desired.append(entry['q_desired'])
        gripper_widths.append(entry['gripper_width'])

    return {
        'time': np.array(times),
        'q_actual': np.array(q_actual),
        'q_desired': np.array(q_desired),
        'gripper_width': np.array(gripper_widths)
    }


def interpolate_trajectory(times, values, target_times):
    """Interpolate trajectory to match target time points."""
    interpolated = np.zeros((len(target_times), values.shape[1]))
    for i in range(values.shape[1]):
        interpolated[:, i] = np.interp(target_times, times, values[:, i])
    return interpolated


def plot_comparison(ref_data, rec_data, output_path=None):
    """Create comparison plots."""
    joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4",
                   "Joint 5", "Joint 6", "Joint 7"]

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))

    # Plot 1: All joints comparison
    for i in range(7):
        ax = plt.subplot(4, 2, i + 1)

        # Plot reference trajectory
        ax.plot(ref_data['time'], ref_data['joint_positions'][:, i],
                'b-', linewidth=2, label='Reference', alpha=0.7)

        # Plot desired trajectory (from controller)
        ax.plot(rec_data['time'], rec_data['q_desired'][:, i],
                'g--', linewidth=1.5, label='Desired', alpha=0.7)

        # Plot actual trajectory
        ax.plot(rec_data['time'], rec_data['q_actual'][:, i],
                'r-', linewidth=1, label='Actual', alpha=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(joint_names[i])
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    # Plot 8: Gripper width comparison
    ax = plt.subplot(4, 2, 8)
    ax.plot(ref_data['time'], ref_data['gripper_width'],
            'b-', linewidth=2, label='Reference', alpha=0.7)
    ax.plot(rec_data['time'], rec_data['gripper_width'],
            'r-', linewidth=1, label='Actual', alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Width (m)')
    ax.set_title('Gripper Width')
    ax.legend(loc='best', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved comparison plot to: {output_path}")

    plt.show()


def plot_tracking_errors(ref_data, rec_data, output_path=None):
    """Plot tracking errors for each joint."""
    joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4",
                   "Joint 5", "Joint 6", "Joint 7"]

    # Interpolate reference to match recorded time points
    ref_interpolated = interpolate_trajectory(
        ref_data['time'],
        ref_data['joint_positions'],
        rec_data['time']
    )

    # Calculate errors
    errors = rec_data['q_actual'] - ref_interpolated

    # Create figure
    fig = plt.figure(figsize=(16, 10))

    # Plot tracking errors for each joint
    for i in range(7):
        ax = plt.subplot(3, 3, i + 1)
        ax.plot(rec_data['time'], errors[:, i] * 1000, 'r-', linewidth=1)
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (mrad)')
        ax.set_title(f'{joint_names[i]} Tracking Error')
        ax.grid(True, alpha=0.3)

        # Add RMSE to title
        rmse = np.sqrt(np.mean(errors[:, i]**2)) * 1000
        max_error = np.max(np.abs(errors[:, i])) * 1000
        ax.text(0.02, 0.98, f'RMSE: {rmse:.2f} mrad\nMax: {max_error:.2f} mrad',
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                fontsize=8)

    # Plot 8: Overall tracking error (norm)
    ax = plt.subplot(3, 3, 8)
    error_norm = np.linalg.norm(errors, axis=1) * 1000
    ax.plot(rec_data['time'], error_norm, 'r-', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error Norm (mrad)')
    ax.set_title('Overall Tracking Error')
    ax.grid(True, alpha=0.3)

    rmse_norm = np.sqrt(np.mean(error_norm**2))
    max_norm = np.max(error_norm)
    ax.text(0.02, 0.98, f'RMSE: {rmse_norm:.2f} mrad\nMax: {max_norm:.2f} mrad',
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            fontsize=8)

    # Plot 9: Gripper error
    ax = plt.subplot(3, 3, 9)
    gripper_ref_interp = np.interp(rec_data['time'], ref_data['time'],
                                     ref_data['gripper_width'])
    gripper_error = (rec_data['gripper_width'] - gripper_ref_interp) * 1000
    ax.plot(rec_data['time'], gripper_error, 'r-', linewidth=1)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (mm)')
    ax.set_title('Gripper Width Error')
    ax.grid(True, alpha=0.3)

    rmse_gripper = np.sqrt(np.mean(gripper_error**2))
    max_gripper = np.max(np.abs(gripper_error))
    ax.text(0.02, 0.98, f'RMSE: {rmse_gripper:.2f} mm\nMax: {max_gripper:.2f} mm',
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            fontsize=8)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved error plot to: {output_path}")

    plt.show()


def print_statistics(ref_data, rec_data):
    """Print tracking statistics."""
    joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4",
                   "Joint 5", "Joint 6", "Joint 7"]

    # Interpolate reference to match recorded time points
    ref_interpolated = interpolate_trajectory(
        ref_data['time'],
        ref_data['joint_positions'],
        rec_data['time']
    )

    # Calculate errors
    errors = rec_data['q_actual'] - ref_interpolated

    print("\n" + "="*60)
    print("TRACKING STATISTICS")
    print("="*60)

    print("\nJoint Tracking Errors (in milliradians):")
    print("-" * 60)
    print(f"{'Joint':<10} {'RMSE':>10} {'Max':>10} {'Mean':>10} {'Std':>10}")
    print("-" * 60)

    for i in range(7):
        rmse = np.sqrt(np.mean(errors[:, i]**2)) * 1000
        max_err = np.max(np.abs(errors[:, i])) * 1000
        mean_err = np.mean(errors[:, i]) * 1000
        std_err = np.std(errors[:, i]) * 1000
        print(f"{joint_names[i]:<10} {rmse:>10.2f} {max_err:>10.2f} {mean_err:>10.2f} {std_err:>10.2f}")

    # Overall error
    error_norm = np.linalg.norm(errors, axis=1) * 1000
    rmse_norm = np.sqrt(np.mean(error_norm**2))
    max_norm = np.max(error_norm)
    mean_norm = np.mean(error_norm)
    std_norm = np.std(error_norm)
    print("-" * 60)
    print(f"{'Overall':<10} {rmse_norm:>10.2f} {max_norm:>10.2f} {mean_norm:>10.2f} {std_norm:>10.2f}")

    # Gripper error
    gripper_ref_interp = np.interp(rec_data['time'], ref_data['time'],
                                     ref_data['gripper_width'])
    gripper_error = (rec_data['gripper_width'] - gripper_ref_interp) * 1000
    rmse_gripper = np.sqrt(np.mean(gripper_error**2))
    max_gripper = np.max(np.abs(gripper_error))
    mean_gripper = np.mean(gripper_error)
    std_gripper = np.std(gripper_error)

    print("\nGripper Width Error (in millimeters):")
    print("-" * 60)
    print(f"{'RMSE':>10} {'Max':>10} {'Mean':>10} {'Std':>10}")
    print(f"{rmse_gripper:>10.2f} {max_gripper:>10.2f} {mean_gripper:>10.2f} {std_gripper:>10.2f}")
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Compare reference and recorded trajectories')
    parser.add_argument('--reference', '-r', required=True,
                        help='Path to reference trajectory JSON')
    parser.add_argument('--recorded', '-c', required=True,
                        help='Path to recorded trajectory JSON')
    parser.add_argument('--output-dir', '-o', default='.',
                        help='Output directory for plots')

    args = parser.parse_args()

    print("Loading reference trajectory...")
    ref_data = load_reference_trajectory(args.reference)

    print("Loading recorded trajectory...")
    rec_data = load_recorded_trajectory(args.recorded)

    print(f"Reference trajectory: {len(ref_data['time'])} points, "
          f"{ref_data['time'][-1]:.2f} seconds")
    print(f"Recorded trajectory: {len(rec_data['time'])} points, "
          f"{rec_data['time'][-1]:.2f} seconds")

    # Print statistics
    print_statistics(ref_data, rec_data)

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots
    print("\nGenerating comparison plots...")
    plot_comparison(ref_data, rec_data,
                   output_path=output_dir / 'trajectory_comparison.png')

    print("Generating error plots...")
    plot_tracking_errors(ref_data, rec_data,
                        output_path=output_dir / 'tracking_errors.png')

    print("\nDone!")


if __name__ == '__main__':
    main()
