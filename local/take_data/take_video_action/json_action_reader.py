import json
from typing import List, Optional, Tuple


DEFAULT_ACTION_JSON_1 = (
    "/home/ubuntu/take_data/data/record_001/action/15HZ/"
    "action_data_20251230_012732.json"
)




DEFAULT_ACTION_JSON = (
    "/home/ubuntu/take_data/data/record_006/action/"
    "action_data_20260113_210756.json"
)

def clamp_gripper(value: float, binarize: bool = False) -> float:
    g = float(value)
    if binarize:
        g = 1.0 if g > 0.5 else 0.0
    return max(0.0, min(1.0, g))


class ActionPositionReader:
    def __init__(self, json_path: str = DEFAULT_ACTION_JSON):
        self.json_path = json_path
        self.positions: List[List[float]] = []
        self.gripper_commands: List[float] = []
        self.current_index = 0
        self._load_data()

    def _load_data(self) -> None:
        with open(self.json_path, "r") as f:
            data = json.load(f)
        for item in data.get("data", []):
            franka_joints = item.get("franka_joints", {})
            position = franka_joints.get("position")
            gripper_command = item.get("gripper_command", 0.0)
            if position is not None:
                self.positions.append(position)
                if gripper_command is None:
                    gripper_command = 0.0
                self.gripper_commands.append(float(gripper_command))

    def get_next(self) -> Optional[Tuple[List[float], float]]:
        if self.current_index >= len(self.positions):
            return None
        position = self.positions[self.current_index]
        gripper_command = self.gripper_commands[self.current_index]
        self.current_index += 1
        return (position, gripper_command)

    def reset(self) -> None:
        self.current_index = 0

    def has_next(self) -> bool:
        return self.current_index < len(self.positions)

    def get_total_steps(self) -> int:
        return len(self.positions)
