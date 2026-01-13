#!/usr/bin/env python3
import glob
import json
import os

import cv2
import h5py
import numpy as np
import polars as pl

DATA_ROOT = "/home/ubuntu/take_data/data"
OUT_ROOT = "/home/ubuntu/take_data/lerobot_data/lerobot_take_franka_gripper_30fps"
FPS = 30.0
FPS_INT = int(FPS)

CAMERA_KEYS = [
    "observation.images.image",
    "observation.images.image_additional_view",
]

CAMERA_PREFIX_TO_KEY = {
    "camera_1_": "observation.images.image",
    "camera_2_": "observation.images.image_additional_view",
}


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def parse_camera_key(path: str) -> str:
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

# Prepare output dirs
ensure_dir(OUT_ROOT)
meta_dir = os.path.join(OUT_ROOT, "meta")
ensure_dir(meta_dir)
data_dir = os.path.join(OUT_ROOT, "data", "chunk-000")
ensure_dir(data_dir)
video_root = os.path.join(OUT_ROOT, "videos")
for key in CAMERA_KEYS:
    ensure_dir(os.path.join(video_root, key, "chunk-000"))

# Determine output video size from first record
first_video = sorted(glob.glob(os.path.join(records[0], "video", "*.mp4")))[0]
cap0 = cv2.VideoCapture(first_video)
if not cap0.isOpened():
    raise SystemExit(f"Failed to open {first_video}")
width = int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap0.release()

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writers = {}
for key in CAMERA_KEYS:
    out_path = os.path.join(video_root, key, "chunk-000", "file-000.mp4")
    writer = cv2.VideoWriter(out_path, fourcc, FPS, (width, height))
    if not writer.isOpened():
        raise SystemExit(f"Failed to open video writer for {out_path}")
    writers[key] = writer

states = []
actions = []
timestamps = []
episode_indices = []
frame_indices = []
next_rewards = []
next_dones = []
indices = []
task_indices = []

index_counter = 0

for ep_idx, record in enumerate(records):
    h5_files = glob.glob(os.path.join(record, "action", "*.h5"))
    if len(h5_files) != 1:
        raise SystemExit(f"Expected 1 h5 in {record}, found {len(h5_files)}")
    h5_path = h5_files[0]

    video_files = sorted(glob.glob(os.path.join(record, "video", "*.mp4")))
    if len(video_files) < len(CAMERA_KEYS):
        raise SystemExit(f"Expected at least {len(CAMERA_KEYS)} videos in {record}, found {len(video_files)}")

    cap_map = {}
    frame_counts = []
    fps_values = []
    for vf in video_files:
        key = parse_camera_key(vf)
        if key is None:
            continue
        cap = cv2.VideoCapture(vf)
        if not cap.isOpened():
            raise SystemExit(f"Failed to open {vf}")
        cap_map[key] = cap
        frame_counts.append(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        fps_values.append(cap.get(cv2.CAP_PROP_FPS))

    if len(cap_map) != len(CAMERA_KEYS):
        raise SystemExit(f"Missing expected cameras in {record}: {sorted(cap_map.keys())}")

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

    episode_start = len(next_dones)
    frames_used = 0

    for i in range(frames_min):
        ok_frames = []
        frames = {}
        for key in CAMERA_KEYS:
            ok, frame = cap_map[key].read()
            ok_frames.append(ok)
            if ok:
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))
                frames[key] = frame
        if not all(ok_frames):
            break

        for key in CAMERA_KEYS:
            writers[key].write(frames[key])

        states.append(state_arr[i].tolist())
        actions.append(action_arr[i].tolist())
        timestamps.append(float(i / FPS))
        episode_indices.append(ep_idx)
        frame_indices.append(i)
        next_rewards.append(0.0)
        next_dones.append(False)
        indices.append(index_counter)
        task_indices.append(0)
        index_counter += 1
        frames_used += 1

    for cap in cap_map.values():
        cap.release()

    if frames_used == 0:
        continue
    next_dones[episode_start + frames_used - 1] = True

for writer in writers.values():
    writer.release()

if not states:
    raise SystemExit("No frames processed")

state_series = pl.Series("observation.state", states, dtype=pl.List(pl.Float32))
action_series = pl.Series("action", actions, dtype=pl.List(pl.Float32))

df = pl.DataFrame(
    {
        "observation.state": state_series,
        "action": action_series,
        "timestamp": pl.Series(timestamps, dtype=pl.Float32),
        "episode_index": pl.Series(episode_indices, dtype=pl.Int64),
        "frame_index": pl.Series(frame_indices, dtype=pl.Int64),
        "next.reward": pl.Series(next_rewards, dtype=pl.Float32),
        "next.done": pl.Series(next_dones, dtype=pl.Boolean),
        "index": pl.Series(indices, dtype=pl.Int64),
        "task_index": pl.Series(task_indices, dtype=pl.Int64),
    }
)

parquet_path = os.path.join(data_dir, "file-000.parquet")
df.write_parquet(parquet_path)

# tasks.parquet
pl.DataFrame(
    {
        "task_index": [0],
        "__index_level_0__": ["default"],
    }
).write_parquet(os.path.join(meta_dir, "tasks.parquet"))

# stats.json
state_arr = np.array(states, dtype=np.float32)
action_arr = np.array(actions, dtype=np.float32)

timestamp_arr = np.array(timestamps, dtype=np.float32)
episode_arr = np.array(episode_indices, dtype=np.int64)
frame_arr = np.array(frame_indices, dtype=np.int64)
index_arr = np.array(indices, dtype=np.int64)
reward_arr = np.array(next_rewards, dtype=np.float32)
done_arr = np.array(next_dones, dtype=np.bool_)
task_arr = np.array(task_indices, dtype=np.int64)

stats = {
    "observation.state": stats_matrix(state_arr),
    "action": stats_matrix(action_arr),
    "timestamp": stats_vector(timestamp_arr),
    "episode_index": stats_vector(episode_arr),
    "frame_index": stats_vector(frame_arr.astype(np.float32)),
    "next.reward": stats_vector(reward_arr),
    "next.done": stats_bool(done_arr),
    "index": stats_vector(index_arr.astype(np.float32)),
    "task_index": stats_vector(task_arr.astype(np.float32)),
}

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

features = {}
for key in CAMERA_KEYS:
    features[key] = {
        "dtype": "video",
        "shape": [height, width, 3],
        "names": ["height", "width", "channel"],
        "video_info": {
            "video.fps": FPS,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    }

features.update(
    {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(feature_names)],
            "names": {"motors": feature_names},
            "fps": FPS,
        },
        "action": {
            "dtype": "float32",
            "shape": [len(feature_names)],
            "names": {"motors": feature_names},
            "fps": FPS,
        },
        "timestamp": {
            "dtype": "float32",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "episode_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "frame_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "next.reward": {
            "dtype": "float32",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "next.done": {
            "dtype": "bool",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
        "task_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS,
        },
    }
)

info = {
    "codebase_version": "v3.0",
    "robot_type": "unknown",
    "total_episodes": len(records),
    "total_frames": int(len(states)),
    "total_tasks": 1,
    "chunks_size": 1000,
    "fps": FPS_INT,
    "splits": {
        "train": f"0:{len(records)}",
    },
    "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
    "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
    "features": features,
}

info["data_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "data"))
info["video_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "videos"))

with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as f:
    json.dump(info, f, indent=2)

print("Done")
print(f"Output: {OUT_ROOT}")
