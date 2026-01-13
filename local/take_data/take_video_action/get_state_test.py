from test import RemoteStateClient
import time

def main() -> None:
    client = RemoteStateClient()
    while(1):
        state = client.get_state(timeout_s=3.0)
        print("joint_position:", state["joint_position"])
        print("gripper_position:", state["gripper_position"])
        time.sleep(0.01)


if __name__ == "__main__":
    main()






import numpy as np
import time 
from test import RemoteStateClient
from openpi_gui import  ActionPositionReader
reader = ActionPositionReader("/home/ubuntu/take_data/data/record_001/action/15HZ/action_data_20251230_012732.json")


def clamp_gripper(value: float, binarize: bool = False) -> float:
    g = float(value)
    if binarize:
        g = 1.0 if g > 0.5 else 0.0
    return max(0.0, min(1.0, g))


def main() -> None:
    
    with RemoteStateClient() as client:

        result = reader.get_next() if reader is not None else None
        position, gripper_command = result
        gripper_cmd = clamp_gripper(gripper_command, binarize=False)
        last_json_state = np.array(position + [gripper_cmd], dtype=np.float32).reshape(1, -1)

        while True:

            state = client.get_state(timeout_s=3.0)
            current_state = np.concatenate([state["joint_position"], [state["gripper_position"]]]).astype(np.float32)

            result = reader.get_next() if reader is not None else None
            position, gripper_command = result
            gripper_cmd = clamp_gripper(gripper_command, binarize=False)
            json_state = np.array(position + [gripper_cmd], dtype=np.float32).reshape(1, -1)
            print("json_state",json_state)

            delta = json_state - last_json_state
            print("delta",delta)
    
            action = current_state + delta
            client.send_action(action.reshape(1, -1))

            last_json_state = json_state

            time.sleep(1/15)


if __name__ == "__main__":
    main()
