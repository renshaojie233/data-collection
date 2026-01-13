import numpy as np
import time 
from test import RemoteStateClient
from openpi_gui import  ActionPositionReader
reader = ActionPositionReader("/home/ubuntu/take_data/data/record_001/action/15HZ/action_data_20251230_012732.json")
STEP = 3  # 15Hz -> 5Hz by skipping 2 frames


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
        client.send_action(last_json_state.reshape(1, -1))
        time.sleep(2)
        print(156)
        while True:

            state = client.get_state(timeout_s=3.0)
            current_state = np.concatenate([state["joint_position"], [state["gripper_position"]]]).astype(np.float32)
            print("current_state",current_state)

            result = reader.get_next() if reader is not None else None
            position, gripper_command = result
            gripper_cmd = clamp_gripper(gripper_command, binarize=False)
            json_state = np.array(position + [gripper_cmd], dtype=np.float32).reshape(1, -1)
            print("json_state",json_state)

            delta = json_state - last_json_state
            print("delta",delta)
    
            action = current_state + delta
            print("action",action)

            # action = json_state

            client.send_action(action.reshape(1, -1))

            last_json_state = json_state

            time.sleep(1/15)


if __name__ == "__main__":
    main()






