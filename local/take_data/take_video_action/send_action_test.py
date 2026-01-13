import numpy as np

from test import RemoteStateClient


def main() -> None:
    with RemoteStateClient() as client:
        state = client.get_state(timeout_s=3.0)
        if state is None:
            print("No state received.")
            return
        action = np.concatenate(
            [state["joint_position"], [state["gripper_position"]]]
        ).astype(np.float32)
        action[6] += np.deg2rad(5.0)
        client.send_action(action.reshape(1, -1))
        print("Sent action.")


if __name__ == "__main__":
    main()
