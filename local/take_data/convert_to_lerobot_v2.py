#!/usr/bin/env python3
import glob
import json
import os
import shutil
import subprocess

import cv2
import h5py
import numpy as np
import polars as pl
import imageio_ffmpeg as ffmpeg

DATA_ROOT = "/home/ubuntu/take_data/data"
OUT_ROOT = "/home/ubuntu/take_data/lerobot_data/lerobot_take_franka_gripper_30fps_v2"
FPS = 30.0
FPS_INT = int(FPS)

CAMERA_PREFIX_TO_KEY = {
    "camera_1_": "observation.images.image",
    "camera_2_": "observation.images.image_additional_view",
}
CAMERA_KEYS = ["observation.images.image", "observation.images.image_additional_view"]


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def parse_camera_key(path: str) -> str | None:
    base = os.path.basename(path)
    for prefix, key in CAMERA_PREFIX_TO_KEY.items():
        if base.startswith(prefix):
            return key
    return None


def nearest_indices(ts: np.ndarray, frame_times: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(ts, frame_times, side="left")
    idx = np.clip(idx, 1, len(ts) - 1)
    prev = idx - 1
    next_idx = idx
    use_prev = (frame_times - ts[prev]) <= (ts[next_idx] - frame_times)
    idx = np.where(use_prev, prev, next_idx)
    idx[frame_times <= ts[0]] = 0
    idx[frame_times >= ts[-1]] = len(ts) - 1
    return idx


def stats_matrix(arr: np.ndarray) -> dict:
    return {
        "min": [float(x) for x in arr.min(axis=0)],
        "max": [float(x) for x in arr.max(axis=0)],
        "mean": [float(x) for x in arr.mean(axis=0)],
        "std": [float(x) for x in arr.std(axis=0)],
        "count": [int(arr.shape[0])],
    }


def stats_vector(arr: np.ndarray) -> dict:
    return {
        "min": [float(arr.min())],
        "max": [float(arr.max())],
        "mean": [float(arr.mean())],
        "std": [float(arr.std())],
        "count": [int(arr.shape[0])],
    }


def stats_bool(arr: np.ndarray) -> dict:
    arr_f = arr.astype(np.float32)
    return {
        "min": [bool(arr.min())],
        "max": [bool(arr.max())],
        "mean": [float(arr_f.mean())],
        "std": [float(arr_f.std())],
        "count": [int(arr.shape[0])],
    }


def dir_size_mb(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            total += os.path.getsize(os.path.join(root, name))
    return int(round(total / (1024 * 1024)))


records = sorted(glob.glob(os.path.join(DATA_ROOT, "record_*")))
if not records:
    raise SystemExit(f"No records found in {DATA_ROOT}")

FFMPEG_EXE = ffmpeg.get_ffmpeg_exe()

# Prepare output dirs
if os.path.exists(OUT_ROOT):
    shutil.rmtree(OUT_ROOT)
ensure_dir(OUT_ROOT)
meta_dir = os.path.join(OUT_ROOT, "meta")
ensure_dir(meta_dir)
data_dir = os.path.join(OUT_ROOT, "data", "chunk-000")
ensure_dir(data_dir)
video_root = os.path.join(OUT_ROOT, "videos", "chunk-000")
for key in CAMERA_KEYS:
    ensure_dir(os.path.join(video_root, key))

# Determine output video size from first record
first_video = sorted(glob.glob(os.path.join(records[0], "video", "camera_1_*.mp4")))[0]
cap0 = cv2.VideoCapture(first_video)
if not cap0.isOpened():
    raise SystemExit(f"Failed to open {first_video}")
width = int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap0.release()

states_all = []
actions_all = []
timestamps_all = []
episode_indices_all = []
frame_indices_all = []
next_rewards_all = []
next_dones_all = []
indices_all = []
task_indices_all = []
episode_lengths = []

index_counter = 0

for ep_idx, record in enumerate(records):
    h5_files = glob.glob(os.path.join(record, "action", "*.h5"))
    if len(h5_files) != 1:
        raise SystemExit(f"Expected 1 h5 in {record}, found {len(h5_files)}")
    h5_path = h5_files[0]

    video_files = sorted(glob.glob(os.path.join(record, "video", "*.mp4")))
    if len(video_files) < len(CAMERA_KEYS):
        raise SystemExit(f"Expected at least {len(CAMERA_KEYS)} videos in {record}, found {len(video_files)}")

    frame_counts = []
    fps_values = []
    video_paths = {}
    for vf in video_files:
        key = parse_camera_key(vf)
        if key is None:
            continue
        cap = cv2.VideoCapture(vf)
        if not cap.isOpened():
            raise SystemExit(f"Failed to open {vf}")
        video_paths[key] = vf
        frame_counts.append(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        fps_values.append(cap.get(cv2.CAP_PROP_FPS))
        cap.release()

    if len(video_paths) != len(CAMERA_KEYS):
        raise SystemExit(f"Missing expected cameras in {record}: {sorted(video_paths.keys())}")

    frames_min = min(frame_counts)
    if frames_min <= 0:
        raise SystemExit(f"No frames in {record}")

    if any(abs(fps - FPS) > 1e-3 for fps in fps_values):
        raise SystemExit(f"FPS mismatch in {record}: {fps_values}")

    with h5py.File(h5_path, "r") as f:
        ts = f["timestamps"][:].astype(np.float64)
        franka_pos = f["franka/positions"][:].astype(np.float32)
        gripper_pos = f["gripper/positions"][:].astype(np.float32)

    frame_times = ts[0] + (np.arange(frames_min, dtype=np.float64) / FPS)
    idx = nearest_indices(ts, frame_times)
    state_arr = np.concatenate([franka_pos[idx], gripper_pos[idx]], axis=1).astype(np.float32)
    action_arr = state_arr.copy()

    episode_start = len(next_dones_all)
    frames_used = frames_min

    # Re-encode videos to AV1 for viewer compatibility.
    for key in CAMERA_KEYS:
        out_path = os.path.join(video_root, key, f"episode_{ep_idx:06d}.mp4")
        subprocess.run(
            [
                FFMPEG_EXE,
                "-y",
                "-loglevel",
                "error",
                "-i",
                video_paths[key],
                "-c:v",
                "libaom-av1",
                "-crf",
                "35",
                "-b:v",
                "0",
                "-cpu-used",
                "8",
                "-row-mt",
                "1",
                "-pix_fmt",
                "yuv420p",
                "-r",
                str(FPS_INT),
                "-frames:v",
                str(frames_used),
                "-an",
                out_path,
            ],
            check=True,
        )

    for i in range(frames_used):
        states_all.append(state_arr[i].tolist())
        actions_all.append(action_arr[i].tolist())
        timestamps_all.append(float(i / FPS))
        episode_indices_all.append(ep_idx)
        frame_indices_all.append(i)
        next_rewards_all.append(0.0)
        next_dones_all.append(False)
        indices_all.append(index_counter)
        task_indices_all.append(0)
        index_counter += 1

    if frames_used == 0:
        continue
    next_dones_all[episode_start + frames_used - 1] = True
    episode_lengths.append(frames_used)

    # Per-episode parquet
    df_ep = pl.DataFrame(
        {
            "observation.state": pl.Series(state_arr[:frames_used].tolist(), dtype=pl.List(pl.Float32)),
            "action": pl.Series(action_arr[:frames_used].tolist(), dtype=pl.List(pl.Float32)),
            "timestamp": pl.Series([float(i / FPS) for i in range(frames_used)], dtype=pl.Float32),
            "episode_index": pl.Series([ep_idx] * frames_used, dtype=pl.Int64),
            "frame_index": pl.Series(list(range(frames_used)), dtype=pl.Int64),
            "next.reward": pl.Series([0.0] * frames_used, dtype=pl.Float32),
            "next.done": pl.Series([False] * (frames_used - 1) + [True], dtype=pl.Boolean),
            "index": pl.Series(list(range(index_counter - frames_used, index_counter)), dtype=pl.Int64),
            "task_index": pl.Series([0] * frames_used, dtype=pl.Int64),
        }
    )
    df_ep.write_parquet(
        os.path.join(data_dir, f"episode_{ep_idx:06d}.parquet"),
        compression="uncompressed",
    )

# tasks.jsonl
with open(os.path.join(meta_dir, "tasks.jsonl"), "w", encoding="utf-8") as f:
    f.write(json.dumps({"task_index": 0, "task": "default"}) + "\n")

# episodes.jsonl
with open(os.path.join(meta_dir, "episodes.jsonl"), "w", encoding="utf-8") as f:
    for ep_idx, length in enumerate(episode_lengths):
        f.write(json.dumps({"episode_index": ep_idx, "tasks": ["default"], "length": length}) + "\n")

# stats.json
state_arr_all = np.array(states_all, dtype=np.float32)
action_arr_all = np.array(actions_all, dtype=np.float32)

timestamp_arr = np.array(timestamps_all, dtype=np.float32)
episode_arr = np.array(episode_indices_all, dtype=np.int64)
frame_arr = np.array(frame_indices_all, dtype=np.int64)
index_arr = np.array(indices_all, dtype=np.int64)
reward_arr = np.array(next_rewards_all, dtype=np.float32)
done_arr = np.array(next_dones_all, dtype=np.bool_)
task_arr = np.array(task_indices_all, dtype=np.int64)

stats = {
    "observation.state": stats_matrix(state_arr_all),
    "action": stats_matrix(action_arr_all),
    "timestamp": stats_vector(timestamp_arr),
    "episode_index": stats_vector(episode_arr),
    "frame_index": stats_vector(frame_arr.astype(np.float32)),
    "next.reward": stats_vector(reward_arr),
    "next.done": stats_bool(done_arr),
    "index": stats_vector(index_arr.astype(np.float32)),
    "task_index": stats_vector(task_arr.astype(np.float32)),
}

# Add image stats placeholders (values in [0,1])
stats["observation.images.image"] = {
    "min": [[[0.0]], [[0.0]], [[0.0]]],
    "max": [[[1.0]], [[1.0]], [[1.0]]],
    "mean": [[[0.5]], [[0.5]], [[0.5]]],
    "std": [[[0.2]], [[0.2]], [[0.2]]],
    "count": [len(states_all)],
}
stats["observation.images.image_additional_view"] = stats["observation.images.image"].copy()

with open(os.path.join(meta_dir, "stats.json"), "w", encoding="utf-8") as f:
    json.dump(stats, f, indent=2)

# info.json
feature_names = [
    "franka_joint_0",
    "franka_joint_1",
    "franka_joint_2",
    "franka_joint_3",
    "franka_joint_4",
    "franka_joint_5",
    "franka_joint_6",
    "gripper_pos_0",
    "gripper_pos_1",
]

features = {
    "observation.images.image": {
        "dtype": "video",
        "shape": [height, width, 3],
        "names": ["height", "width", "rgb"],
        "info": {
            "video.fps": FPS,
            "video.height": height,
            "video.width": width,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    },
    "observation.images.image_additional_view": {
        "dtype": "video",
        "shape": [height, width, 3],
        "names": ["height", "width", "rgb"],
        "info": {
            "video.fps": FPS,
            "video.height": height,
            "video.width": width,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    },
    "observation.state": {
        "dtype": "float32",
        "shape": [len(feature_names)],
        "names": {"motors": feature_names},
    },
    "action": {
        "dtype": "float32",
        "shape": [len(feature_names)],
        "names": {"motors": feature_names},
    },
    "timestamp": {
        "dtype": "float32",
        "shape": [1],
        "names": None,
    },
    "episode_index": {
        "dtype": "int64",
        "shape": [1],
        "names": None,
    },
    "frame_index": {
        "dtype": "int64",
        "shape": [1],
        "names": None,
    },
    "next.reward": {
        "dtype": "float32",
        "shape": [1],
        "names": None,
    },
    "next.done": {
        "dtype": "bool",
        "shape": [1],
        "names": None,
    },
    "index": {
        "dtype": "int64",
        "shape": [1],
        "names": None,
    },
    "task_index": {
        "dtype": "int64",
        "shape": [1],
        "names": None,
    },
}

info = {
    "codebase_version": "v2.0",
    "robot_type": "unknown",
    "total_episodes": len(records),
    "total_frames": int(len(states_all)),
    "total_tasks": 1,
    "total_videos": len(records) * len(CAMERA_KEYS),
    "total_chunks": 1,
    "chunks_size": 1000,
    "fps": FPS_INT,
    "splits": {
        "train": f"0:{len(records)}",
    },
    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
    "features": features,
}

info["data_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "data"))
info["video_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "videos"))

with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as f:
    json.dump(info, f, indent=2)

print("Done")
print(f"Output: {OUT_ROOT}")
