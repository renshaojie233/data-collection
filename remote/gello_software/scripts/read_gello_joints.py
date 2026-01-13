import argparse
import time

import numpy as np

from gello.dynamixel.driver import DynamixelDriver


def main() -> None:
    parser = argparse.ArgumentParser(description="Read GELLO joint positions.")
    parser.add_argument(
        "--port",
        default="/dev/gello",
        help="GELLO USB port (by-id path recommended).",
    )
    parser.add_argument("--count", type=int, default=200, help="Number of samples.")
    parser.add_argument("--hz", type=float, default=10.0, help="Sample rate.")
    args = parser.parse_args()

    driver = DynamixelDriver(range(1, 9), port=args.port, baudrate=57600)
    try:
        last = None
        period = 1.0 / max(args.hz, 1e-6)
        for _ in range(args.count):
            joints = driver.get_joints()
            if last is None:
                delta = np.zeros_like(joints)
            else:
                delta = joints - last
            print("joints:", np.round(joints, 3), "delta:", np.round(delta, 3))
            last = joints
            time.sleep(period)
    finally:
        driver.close()


if __name__ == "__main__":
    main()
