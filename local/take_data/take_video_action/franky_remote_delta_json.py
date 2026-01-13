import numpy as np
import time 

from franky_remote_client import FrankyRemoteClient
from json_action_reader import ActionPositionReader, clamp_gripper, DEFAULT_ACTION_JSON


def main() -> None:
    
    reader = ActionPositionReader(DEFAULT_ACTION_JSON)
    
    client = FrankyRemoteClient(dynamics_factor=(0.2, 0.08, 0.05))
    try:
            
        result = reader.get_next()
        if result is None:
            raise SystemExit("no action data")
        position, gripper_command = result
        gripper_cmd = 1 - clamp_gripper(gripper_command, binarize=True)
        last_json_state = np.array(position + [gripper_cmd], dtype=np.float32)
        client.send_action(last_json_state.tolist(), wait=True)
        time.sleep(2)

        while True:

            state = client.get_state()

            current_state = np.array(state, dtype=np.float32)

            result = reader.get_next()
            result = reader.get_next()
            result = reader.get_next()

            

            
            
            if result is None:
                break
            position, gripper_command = result
            gripper_cmd = 1 - clamp_gripper(gripper_command, binarize=True)
            json_state = np.array(position + [gripper_cmd], dtype=np.float32)

            delta = json_state - last_json_state
            #print(np.abs(current_state - last_json_state))
        
            action = current_state + delta
            #print("delat",delta)
            # action = json_state
            action[-1] = 1 - gripper_command

            print(action)
            client.send_action(action.tolist(), wait=True)

            last_json_state = json_state

            #time.sleep(1/3)

        print("done")
    except Exception:
        # 这个应该被信号处理器捕获，但作为备用
        print("\n程序中断，正在清理资源...")
        client.close()


if __name__ == "__main__":
    main()


