import math

from franky_remote_client import FrankyRemoteClient


def main() -> None:
    client = FrankyRemoteClient(dynamics_factor=(0.2, 0.08, 0.05)) #动力学参数 https://github.com/TimSchneider42/franky/issues/74

    state = client.get_state()
    action = state[:]
    action[6] = action[6] + math.radians(5.0) # 第七个关节角度+5度
    action[7] = 1 - action[7]   # 切换开关状态
    client.send_action(action, wait=False)

    print("done")


if __name__ == "__main__":
    main()

