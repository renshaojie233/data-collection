from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional
from uu import Error
from tqdm import tqdm

import cv2
import numpy as np
import tyro
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def clamp_gripper(value: float, binarize: bool = False) -> float:
    g = float(value)
    if binarize:
        g = 1.0 if g > 0.5 else 0.0
    return max(0.0, min(1.0, g))

def get_task_list_en():
    """
    返回一个英文任务列表
    list[0] 对应编号 1
    """
    rules = [
        (1, 1,  "把桌子上的葡萄放到碗里"),
        (2, 2,  "把盘子里的葡萄放到碗里"),
        (3, 3,  "把桌子上的苹果放到盘子里"),
        (4, 4,  "把盘子里的苹果放到桌子上"),
        (5, 5,  "把桌子上的芒果放到盘子里"),
        (6, 6,  "把桌子上的橘子放到碗里"),
        (7, 15, "把桌子上的葡萄放到盘子里"),
        (16, 30,"把桌子上的葡萄放到碗里"),
        (31, 40,"把桌子上的芒果放到碗里"),
        (41, 45,"把桌子上的芒果放到盘子里"),
        (46, 50,"把盘子里的芒果放到碗里"),
        (51, 56,"把桌子上的橙子放到盘子里"),
        (57, 65,"把桌子上的红苹果放到碗里"),
        (66, 75,"把盘子里的红苹果放到碗里"),
    ]

    cn2en = {
        "把桌子上的葡萄放到碗里": "put the grapes from the table into the bowl",
        "把盘子里的葡萄放到碗里": "put the grapes from the plate into the bowl",
        "把桌子上的苹果放到盘子里": "put the apples from the table onto the plate",
        "把盘子里的苹果放到桌子上": "put the apples from the plate onto the table",
        "把桌子上的芒果放到盘子里": "put the mangoes from the table onto the plate",
        "把桌子上的橘子放到碗里": "put the oranges from the table into the bowl",
        "把桌子上的葡萄放到盘子里": "put the grapes from the table onto the plate",
        "把桌子上的芒果放到碗里": "put the mangoes from the table into the bowl",
        "把盘子里的芒果放到碗里": "put the mangoes from the plate into the bowl",
        "把桌子上的橙子放到盘子里": "put the oranges from the table onto the plate",
        "把桌子上的红苹果放到碗里": "put the red apples from the table into the bowl",
        "把盘子里的红苹果放到碗里": "put the red apples from the plate into the bowl",
    }

    tasks = []
    for start, end, cn_text in rules:
        en_text = cn2en[cn_text]
        for _ in range(start, end + 1):
            tasks.append(en_text)

    return tasks


def extract_frames_from_video(video_path: Path) -> List[np.ndarray]:
    """从视频文件中提取所有帧"""
    if not video_path.exists():
        return []
    
    cap = cv2.VideoCapture(str(video_path))
    frames = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 转换为 RGB 格式
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frames.append(frame_rgb)
    
    cap.release()
    return frames


def find_video_files(video_dir: Path) -> Dict[int, Path]:
    """查找视频文件，返回 {camera_index: video_path}"""
    video_files = {}
    for video_path in sorted(video_dir.glob("camera_*.mp4")):
        # 从文件名中提取相机编号，例如 camera_1_*.mp4 -> 1
        parts = video_path.stem.split("_")
        if len(parts) >= 2 and parts[0] == "camera":
            try:
                camera_idx = int(parts[1])
                video_files[camera_idx] = video_path
            except ValueError:
                continue
    return video_files


def load_action_data(action_dir: Path) -> List[Dict[str, Any]]:
    """从 action 目录加载所有动作数据"""
    steps: List[Dict[str, Any]] = []
    
    for json_path in sorted(action_dir.glob("action_data_*.json")):
        if json_path.name.endswith("_replay.json"):
            continue
        
        with json_path.open("r", encoding="utf-8") as f:
            payload = json.load(f)
        steps.extend(payload.get("data", []))
    
    return steps


def create_lerobot_datasets(
    original_root_dir: str | Path,
    output_dir: str | Path,
    fps: int = 10,
    action_offset: int = 50,
    video_offset: int = 1,
    push_to_hub: bool = False,
):
    """
    将自定义数据格式转换为 LeRobot 数据集格式。
    
    Args:
        original_root_dir: 原始数据根目录（包含 record_* 子目录）
        output_dir: 输出 LeRobot 数据集目录
        fps: 数据集帧率
        action_offset: 动作计算的偏移量，actions = positions[frame_idx + offset] - positions[frame_idx]
        video_offset: 视频帧的降采样步长
        push_to_hub: 是否推送到 Hugging Face Hub
    """
    root = Path(original_root_dir)
    output_path = Path(output_dir)

    if video_offset <= 0:
        raise ValueError("video_offset must be a positive integer")
    
    # 清理已存在的输出目录
    if output_path.exists():
        print(f"删除已存在的数据集目录: {output_path}")
        shutil.rmtree(output_path)
    
    # 创建 LeRobot 数据集
    # OpenPi 假设 proprio 存储在 `state` 中，动作存储在 `action` 中
    # LeRobot 假设图像数据的 dtype 为 `image`
    dataset = LeRobotDataset.create(
        repo_id=str(output_path),
        robot_type="fr3",
        fps=fps,
        features={
            "observation.images.image_0": {
                "dtype": "video",
                "shape": (256, 256, 3),
                "names": ["height", "width", "channel"],
            },
            "observation.images.image_1": {
                "dtype": "video",
                "shape": (256, 256, 3),
                "names": ["height", "width", "channel"],
            },
            "observation.images.image_2": {
                "dtype": "video",
                "shape": (256, 256, 3),
                "names": ["height", "width", "channel"],
            },
            "state": {
                "dtype": "float32",
                "shape": (8,),  # 7个关节 + 1个夹爪
                "names": ["state"],
            },
            "actions": {
                "dtype": "float32",
                "shape": (8,),  # 7个关节动作差值 + 1个夹爪状态
                "names": ["actions"],
            },
        },
        use_videos=True,
        image_writer_threads=10,
        image_writer_processes=5,
        video_backend="ffmpeg"
    )
    
    # 处理每个 record 目录
    record_dirs = sorted(root.glob("record_*"))
    total_episodes = 0
    
    for record_dir in tqdm(record_dirs):
        print(f"\n处理 {record_dir.name}...")
        
        video_dir = record_dir / "video"
        action_dir = record_dir / "action"
        
        # 检查必要的目录是否存在
        if not video_dir.is_dir():
            print(f"  跳过: 未找到 video 目录")
            continue
        if not action_dir.is_dir():
            print(f"  跳过: 未找到 action 目录")
            continue

        task_path = record_dir / "task.txt"
        if not task_path.is_file():
            print(f"  跳过: 未找到 task.txt")
            continue
        prompt_text = task_path.read_text(encoding="utf-8").strip()
        if not prompt_text:
            print(f"  跳过: task.txt 为空")
            continue
        
        # 加载动作数据
        action_steps = load_action_data(action_dir)
        if not action_steps:
            print(f"  跳过: 未找到动作数据")
            continue
        
        # 查找视频文件
        video_files = find_video_files(video_dir)
        if not video_files:
            print(f"  跳过: 未找到视频文件")
            continue
        
        # 提取视频帧
        video_frames: Dict[int, List[np.ndarray]] = {}
        for camera_idx, video_path in video_files.items():
            print(f"  提取相机 {camera_idx} 的视频帧: {video_path.name}")
            frames = extract_frames_from_video(video_path)
            if frames:
                video_frames[camera_idx] = frames
                print(f"    提取了 {len(frames)} 帧")
        
        if not video_frames:
            print(f"  跳过: 无法提取视频帧")
            continue
        
        # 确定最小帧数（确保所有相机都有相同数量的帧）
        min_frames = min(len(frames) for frames in video_frames.values())
        
        # 统一所有相机的帧数到最小值
        for camera_idx in video_frames.keys():
            original_len = len(video_frames[camera_idx])
            if original_len > min_frames:
                video_frames[camera_idx] = video_frames[camera_idx][:min_frames]
                print(f"    相机 {camera_idx} 帧数从 {original_len} 截断到 {min_frames}")
        
        num_frames = min_frames
        num_action_steps = len(action_steps)
        
        # 计算最大可用长度
        # 视频帧索引：0, 1, 2, 3, ... (连续)
        # 动作数据索引：0, offset, 2*offset, 3*offset, ... (按offset间隔)
        # 最大可用长度 = min(视频帧数(降采样后), 动作数据可以取多少个offset间隔)
        max_action_base = num_action_steps - 1 - action_offset
        max_action_indices = 0 if max_action_base < 0 else (max_action_base // action_offset + 1)
        # 确保 future_idx 不越界
        max_video_indices = (num_frames - 1) // video_offset + 1
        max_valid_length = min(max_video_indices, max_action_indices)
        
        if max_valid_length <= 0:
            print(f"  跳过: 数据不足以支持 offset ({action_offset})")
            print(f"    视频帧数: {num_frames}, 动作数据数: {num_action_steps}, 最大动作索引数: {max_action_indices}")
            continue
        
        print(f"  处理 {max_valid_length} 帧数据 (action_offset={action_offset}, video_offset={video_offset})...")
        print(f"    视频帧数: {num_frames}, 动作数据数: {num_action_steps}")
        
        # 将图像调整为 256x256
        target_size = (256, 256)
        
        # 添加每一帧到数据集
        # 维护两个索引：frame_idx (视频帧，连续) 和 action_idx (动作数据，按offset间隔)
        frame_idx = 0  # 视频帧索引，从0开始按 video_offset 递增
        action_base_idx = 0  # 动作数据基础索引，从0开始，每次增加offset
        
        for step in range(max_valid_length):
            # 获取图像（至少需要2个相机，如果有3个则使用前3个）
            images = []
            for cam_idx in sorted(video_frames.keys())[:3]:  # 最多使用3个相机
                frame = video_frames[cam_idx][frame_idx]
                # 调整大小
                frame_resized = cv2.resize(frame, target_size)
                images.append(frame_resized.astype(np.uint8))
            
            # 如果只有1个或2个相机，用最后一个相机填充
            while len(images) < 3:
                images.append(images[-1] if images else np.zeros((256, 256, 3), dtype=np.uint8))
            
            # 获取当前动作数据（action_base_idx）
            current_action_idx = action_base_idx
            action_step = action_steps[current_action_idx]
            franka_joints = action_step.get("franka_joints")
            joint_positions = franka_joints.get("position")
            
            # 确保关节位置是7个
            if len(joint_positions) != 7:
                raise Error("joint position must = 7")
            
            # 获取 offset 步后的关节位置（action_base_idx + action_offset）
            future_action_idx = action_base_idx + action_offset
            if future_action_idx >= num_action_steps:
                # 如果超出范围，使用最后一个有效值
                future_action_idx = num_action_steps - 1
            
            future_action_step = action_steps[future_action_idx]
            future_franka_joints = future_action_step.get("franka_joints")
            future_joint_positions = future_franka_joints.get("position")
            
            # 确保未来关节位置是7个
            if len(future_joint_positions) != 7:
                raise Error("joint position must = 7")
            
            # 获取夹爪命令
            gripper_command = action_step.get("gripper_command")
            future_gripper_command = future_action_step.get("gripper_command")
            
            # 构建 state: [7个关节位置, 1个夹爪]
            state = np.array(joint_positions + [float(gripper_command)], dtype=np.float32)
            
            # 构建 actions: [offset 步后的关节位置 - 当前关节位置, 当前时刻的夹爪状态]
            joint_actions = np.array(future_joint_positions, dtype=np.float32) - np.array(joint_positions, dtype=np.float32)
            actions = np.array(list(joint_actions) + [float(future_gripper_command)], dtype=np.float32)
            
            # 更新索引
            frame_idx += video_offset  # 视频帧索引按 video_offset 递增
            action_base_idx += action_offset  # 动作数据索引按offset间隔递增
            
            # 添加到数据集
            dataset.add_frame(
                {
                    "observation.images.image_0": images[0],
                    "observation.images.image_1": images[1],
                    "observation.images.image_2": images[2],
                    "state": state,
                    "actions": actions,
                },
                task=prompt_text,
            )
        
        # 保存这个 episode
        dataset.save_episode()
        total_episodes += 1
        print(f"  ✓ 完成 {record_dir.name}: {max_valid_length} 帧")
    
    print(f"\n✓ 数据集创建完成!")
    print(f"  总 episode 数: {total_episodes}")
    print(f"  数据集位置: {output_path}")
    
    if push_to_hub:
        print(f"\n推送到 Hugging Face Hub...")
        dataset.push_to_hub()
        print(f"✓ 已推送到 Hub")


def main(
    data_dir: str = "/home/ubuntu/take_data/data",
    output_dir: str = "/home/ubuntu/take_data/lerobot_data/lerobot_realworld",
    fps: int = 10,
    action_offset: int = 66,
    video_offset: int = 2,
    push_to_hub: bool = False,
):
    """
    主函数
    
    Args:
        data_dir: 原始数据目录
        output_dir: 输出数据集目录
        fps: 数据集帧率
        action_offset: 动作计算的偏移量，actions = positions[frame_idx + offset] - positions[frame_idx]
        video_offset: 视频帧的降采样步长
        push_to_hub: 是否推送到 Hugging Face Hub
    """
    create_lerobot_datasets(
        original_root_dir=data_dir,
        output_dir=output_dir,
        fps=fps,
        action_offset=action_offset,
        video_offset=video_offset,
        push_to_hub=push_to_hub,
    )


if __name__ == "__main__":
    tyro.cli(main)
