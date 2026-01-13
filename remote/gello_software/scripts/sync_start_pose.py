import argparse

import numpy as np
from omegaconf import OmegaConf

from gello.dynamixel.driver import DynamixelDriver


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sync start_joints to the current GELLO pose."
    )
    parser.add_argument(
        "--config",
        default="/home/rsj/gello_software/configs/fr3_sim_gello.yaml",
        help="Path to the YAML config to update.",
    )
    parser.add_argument("--samples", type=int, default=20, help="Read samples.")
    args = parser.parse_args()

    cfg = OmegaConf.load(args.config)
    port = cfg.agent.port
    joint_offsets = np.array(cfg.agent.dynamixel_config.joint_offsets, dtype=float)
    joint_signs = np.array(cfg.agent.dynamixel_config.joint_signs, dtype=float)
    joint_ids = list(cfg.agent.dynamixel_config.joint_ids)

    g_cfg = cfg.agent.dynamixel_config.gripper_config
    open_deg, close_deg = float(g_cfg[1]), float(g_cfg[2])
    open_rad = np.deg2rad(open_deg)
    close_rad = np.deg2rad(close_deg)

    num_joints = len(joint_ids) + 1
    driver = DynamixelDriver(range(1, num_joints + 1), port=port, baudrate=57600)
    try:
        samples = []
        for _ in range(max(args.samples, 1)):
            samples.append(driver.get_joints())
        raw = np.mean(np.stack(samples, axis=0), axis=0)
    finally:
        driver.close()

    robot_raw = raw[: len(joint_ids)]
    start_robot = (robot_raw - joint_offsets) * joint_signs

    if abs(close_rad - open_rad) < 1e-6:
        g_pos = 0.0
    else:
        g_pos = (raw[len(joint_ids)] - open_rad) / (close_rad - open_rad)
        g_pos = max(0.0, min(1.0, g_pos))

    start_joints = [float(round(x, 3)) for x in start_robot] + [float(round(g_pos, 3))]
    cfg.agent.start_joints = start_joints
    OmegaConf.save(cfg, args.config)
    print("Updated start_joints:", start_joints)


if __name__ == "__main__":
    main()
