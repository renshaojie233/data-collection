import numpy as np
import time 

from franky_remote_client import FrankyRemoteClient
from json_action_reader import ActionPositionReader, clamp_gripper, DEFAULT_ACTION_JSON


reader = ActionPositionReader(DEFAULT_ACTION_JSON)


def main() -> None:
    
    client = FrankyRemoteClient()
        
    result = reader.get_next() if reader is not None else None
    position, gripper_command = result
    gripper_cmd = clamp_gripper(gripper_command, binarize=False)
    last_json_state = np.array(position + [gripper_cmd], dtype=np.float32)
    client.send_action(last_json_state.tolist(), wait=True)
    time.sleep(2)
    print(156)
    while True:

        state = client.get_state()
        current_state = np.array(state, dtype=np.float32)
        print("current_state",current_state)

        result = reader.get_next() if reader is not None else None
        position, gripper_command = result
        gripper_cmd = clamp_gripper(gripper_command, binarize=False)
        json_state = np.array(position + [gripper_cmd], dtype=np.float32)
        print("json_state",json_state)

        delta = json_state - last_json_state
        print("delta",delta)
    
        action = current_state + delta
        print("action",action)

        # action = json_state

        client.send_action(action.tolist(), wait=False)

        last_json_state = json_state

        time.sleep(1/15)


if __name__ == "__main__":
    main()




