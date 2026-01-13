#!/usr/bin/env python3
"""
Python implementation of Franka's MotionGenerator class.
Generates smooth joint trajectories with velocity and acceleration limits.

Based on:
- franka_fr3_arm_controllers/src/motion_generator.cpp
- Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
"""

import numpy as np
from typing import Tuple


class MotionGenerator:
    """
    Generates smooth joint position trajectories to move from start to goal.

    Features:
    - Trapezoidal velocity profile (acceleration, constant velocity, deceleration)
    - Respects velocity and acceleration limits
    - Synchronized motion (all joints finish at the same time)
    """

    # Constants
    DELTA_Q_MOTION_FINISHED = 1e-6
    NUM_JOINTS = 7

    # Joint limits (rad/s and rad/s^2) - same as Franka FR3
    DQ_MAX = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5])  # Max velocity
    DDQ_MAX_START = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # Max acceleration at start
    DDQ_MAX_GOAL = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # Max acceleration at goal

    def __init__(self, speed_factor: float, q_start: np.ndarray, q_goal: np.ndarray):
        """
        Initialize motion generator.

        Args:
            speed_factor: Speed factor in range (0, 1]. Lower is slower/safer.
            q_start: Starting joint positions [7]
            q_goal: Goal joint positions [7]
        """
        assert 0 < speed_factor <= 1.0, "speed_factor must be in (0, 1]"
        assert len(q_start) == self.NUM_JOINTS, f"q_start must have {self.NUM_JOINTS} joints"
        assert len(q_goal) == self.NUM_JOINTS, f"q_goal must have {self.NUM_JOINTS} joints"

        self.q_start = np.array(q_start, dtype=float)
        self.q_goal = np.array(q_goal, dtype=float)
        self.delta_q = self.q_goal - self.q_start

        # Apply speed factor to limits
        self.dq_max = self.DQ_MAX * speed_factor
        self.ddq_max_start = self.DDQ_MAX_START * speed_factor
        self.ddq_max_goal = self.DDQ_MAX_GOAL * speed_factor

        # Synchronized motion parameters (calculated)
        self.dq_max_sync = np.zeros(self.NUM_JOINTS)
        self.t_1_sync = np.zeros(self.NUM_JOINTS)
        self.t_2_sync = np.zeros(self.NUM_JOINTS)
        self.t_f_sync = np.zeros(self.NUM_JOINTS)
        self.q_1 = np.zeros(self.NUM_JOINTS)

        # Calculate synchronized trajectory parameters
        self._calculate_synchronized_values()

    def _calculate_synchronized_values(self):
        """
        Calculate synchronized motion parameters for all joints.
        Ensures all joints start and finish motion at the same time.
        """
        dq_max_reach = self.dq_max.copy()
        t_f = np.zeros(self.NUM_JOINTS)
        delta_t_2 = np.zeros(self.NUM_JOINTS)
        t_1 = np.zeros(self.NUM_JOINTS)
        sign_delta_q = np.sign(self.delta_q).astype(int)

        # Calculate individual joint motion times
        for i in range(self.NUM_JOINTS):
            if abs(self.delta_q[i]) > self.DELTA_Q_MOTION_FINISHED:
                # Check if joint can reach max velocity
                threshold = (3.0 / 4.0 * (self.dq_max[i]**2 / self.ddq_max_start[i]) +
                           3.0 / 4.0 * (self.dq_max[i]**2 / self.ddq_max_goal[i]))

                if abs(self.delta_q[i]) < threshold:
                    # Cannot reach max velocity, calculate reduced velocity
                    dq_max_reach[i] = np.sqrt(
                        4.0 / 3.0 * abs(self.delta_q[i]) *
                        (self.ddq_max_start[i] * self.ddq_max_goal[i]) /
                        (self.ddq_max_start[i] + self.ddq_max_goal[i])
                    )

                # Calculate time segments
                t_1[i] = 1.5 * dq_max_reach[i] / self.ddq_max_start[i]
                delta_t_2[i] = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i]
                t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + abs(self.delta_q[i]) / dq_max_reach[i]

        # Synchronize all joints to finish at the same time
        max_t_f = np.max(t_f)

        for i in range(self.NUM_JOINTS):
            if abs(self.delta_q[i]) > self.DELTA_Q_MOTION_FINISHED:
                # Recalculate velocity to match longest joint's time
                param_a = 1.5 / 2.0 * (self.ddq_max_goal[i] + self.ddq_max_start[i])
                param_b = -1.0 * max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i]
                param_c = abs(self.delta_q[i]) * self.ddq_max_goal[i] * self.ddq_max_start[i]

                delta = param_b**2 - 4.0 * param_a * param_c
                if delta < 0.0:
                    delta = 0.0

                self.dq_max_sync[i] = (-param_b - np.sqrt(delta)) / (2.0 * param_a)
                self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i]
                delta_t_2_sync = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i]
                self.t_f_sync[i] = (self.t_1_sync[i] / 2.0 + delta_t_2_sync / 2.0 +
                                   abs(self.delta_q[i]) / self.dq_max_sync[i])
                self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync
                self.q_1[i] = self.dq_max_sync[i] * sign_delta_q[i] * 0.5 * self.t_1_sync[i]

    def _calculate_desired_values(self, time: float) -> Tuple[np.ndarray, bool]:
        """
        Calculate desired joint positions at given time.

        Args:
            time: Time since trajectory start (seconds)

        Returns:
            (delta_q_d, motion_finished): Tuple of position delta and completion flag
        """
        delta_q_d = np.zeros(self.NUM_JOINTS)
        joint_motion_finished = np.zeros(self.NUM_JOINTS, dtype=bool)
        sign_delta_q = np.sign(self.delta_q).astype(int)
        t_d = self.t_2_sync - self.t_1_sync
        delta_t_2_sync = self.t_f_sync - self.t_2_sync

        for i in range(self.NUM_JOINTS):
            if abs(self.delta_q[i]) < self.DELTA_Q_MOTION_FINISHED:
                # Joint doesn't need to move
                delta_q_d[i] = 0.0
                joint_motion_finished[i] = True
            else:
                if time < self.t_1_sync[i]:
                    # Acceleration phase
                    delta_q_d[i] = (-1.0 / self.t_1_sync[i]**3 * self.dq_max_sync[i] *
                                   sign_delta_q[i] * (0.5 * time - self.t_1_sync[i]) * time**3)
                elif time >= self.t_1_sync[i] and time < self.t_2_sync[i]:
                    # Constant velocity phase
                    delta_q_d[i] = (self.q_1[i] + (time - self.t_1_sync[i]) *
                                   self.dq_max_sync[i] * sign_delta_q[i])
                elif time >= self.t_2_sync[i] and time < self.t_f_sync[i]:
                    # Deceleration phase
                    delta_q_d[i] = (
                        self.delta_q[i] +
                        0.5 * (
                            1.0 / delta_t_2_sync[i]**3 *
                            (time - self.t_1_sync[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                            (time - self.t_1_sync[i] - t_d[i])**3 +
                            (2.0 * time - 2.0 * self.t_1_sync[i] - delta_t_2_sync[i] - 2.0 * t_d[i])
                        ) * self.dq_max_sync[i] * sign_delta_q[i]
                    )
                else:
                    # Motion finished
                    delta_q_d[i] = self.delta_q[i]
                    joint_motion_finished[i] = True

        motion_finished = np.all(joint_motion_finished)
        return delta_q_d, motion_finished

    def get_desired_joint_positions(self, time: float) -> Tuple[np.ndarray, bool]:
        """
        Get desired joint positions at given time.

        Args:
            time: Time since trajectory start (seconds)

        Returns:
            (q_desired, motion_finished): Tuple of desired positions and completion flag
        """
        delta_q_d, motion_finished = self._calculate_desired_values(time)
        q_desired = self.q_start + delta_q_d
        return q_desired, motion_finished

    def get_trajectory_duration(self) -> float:
        """
        Get the total duration of the trajectory.

        Returns:
            Duration in seconds
        """
        return np.max(self.t_f_sync)


def test_motion_generator():
    """Test the motion generator with sample data."""
    print("=" * 70)
    print("Testing MotionGenerator")
    print("=" * 70)

    # Test case: move from current position to target
    q_start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    q_goal = np.array([0.5, -0.5, 0.3, -2.0, 0.2, 1.8, 1.0])
    speed_factor = 0.2  # Same as FR3 controller startup

    print(f"\nStart position: {q_start}")
    print(f"Goal position:  {q_goal}")
    print(f"Speed factor:   {speed_factor}")

    # Create motion generator
    mg = MotionGenerator(speed_factor, q_start, q_goal)
    duration = mg.get_trajectory_duration()

    print(f"\nTrajectory duration: {duration:.3f} seconds")
    print(f"Max sync velocities: {mg.dq_max_sync}")
    print(f"Time segments (t_1): {mg.t_1_sync}")
    print(f"Time segments (t_2): {mg.t_2_sync}")
    print(f"Time segments (t_f): {mg.t_f_sync}")

    # Sample trajectory at different times
    print(f"\nSampling trajectory:")
    print(f"{'Time (s)':<10} {'Joint 0 (rad)':<15} {'Joint 3 (rad)':<15} {'Finished'}")
    print("-" * 55)

    sample_times = np.linspace(0, duration + 1, 10)
    for t in sample_times:
        q_desired, finished = mg.get_desired_joint_positions(t)
        print(f"{t:<10.3f} {q_desired[0]:<15.4f} {q_desired[3]:<15.4f} {finished}")

    print("\n" + "=" * 70)
    print("✓ MotionGenerator test completed")
    print("=" * 70)


if __name__ == '__main__':
    test_motion_generator()
