#!/usr/bin/env python3
import glob
import json
import os
import shutil
import subprocess

import cv2
import h5py
import imageio_ffmpeg as ffmpeg
import numpy as np
import polars as pl

DATA_ROOT = "/home/ubuntu/take_data/data"
OUT_ROOT = "/home/ubuntu/take_data/lerobot_data/lerobot_take_droid_pi05_15fps"
FPS_IN = 30.0
FPS_OUT = 15.0
FPS_OUT_INT = int(FPS_OUT)

CAMERA_PREFIX_TO_KEY = {
    "camera_1_": "exterior_image_1_left",
    "camera_3_": "exterior_image_2_left",
    "camera_2_": "wrist_image_left",
}
CAMERA_KEYS = ["exterior_image_1_left", "exterior_image_2_left", "wrist_image_left"]

# Robotiq 2F-85 finger travel is ~0.0425 m per finger.
GRIPPER_MAX_DEFAULT = 0.0425

FFMPEG_EXE = ffmpeg.get_ffmpeg_exe()


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def parse_camera_key(path: str) -> str | None:
    base = os.path.basename(path)
    for prefix, key in CAMERA_PREFIX_TO_KEY.items():
        if base.startswith(prefix):
            return key
    return None


def read_task_text(record_path: str) -> str:
    task_path = os.path.join(record_path, "task.txt")
    if not os.path.exists(task_path):
        return "default"
    text = ""
    with open(task_path, "r", encoding="utf-8") as f:
        text = f.read().strip()
    return text or "default"


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


def dir_size_mb(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            total += os.path.getsize(os.path.join(root, name))
    return int(round(total / (1024 * 1024)))


def get_frame_count(path: str) -> int:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open video: {path}")
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    if abs(fps - FPS_IN) > 1e-3:
        raise SystemExit(f"Unexpected fps {fps} in {path}")
    return count


records = sorted(glob.glob(os.path.join(DATA_ROOT, "record_*")))
if not records:
    raise SystemExit(f"No records found under {DATA_ROOT}")
print(f"Found {len(records)} record(s)")
if not records:
    raise SystemExit(f"No records found in {DATA_ROOT}")

# Pre-scan gripper max
max_gripper = 0.0
for record in records:
    h5_files = glob.glob(os.path.join(record, "action", "*.h5"))
    if len(h5_files) != 1:
        raise SystemExit(f"Expected 1 h5 in {record}, found {len(h5_files)}")
    with h5py.File(h5_files[0], "r") as f:
        gripper_pos = f["gripper/positions"][:].astype(np.float32)
        max_gripper = max(max_gripper, float(np.max(gripper_pos)))

if max_gripper <= 0:
    max_gripper = GRIPPER_MAX_DEFAULT

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

# Determine output size from first record camera_1
first_video = sorted(glob.glob(os.path.join(records[0], "video", "camera_1_*.mp4")))[0]
cap0 = cv2.VideoCapture(first_video)
if not cap0.isOpened():
    raise SystemExit(f"Failed to open {first_video}")
width_in = int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
height_in = int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap0.release()

width_out, height_out = 320, 180
if width_in < width_out or height_in < height_out:
    raise SystemExit("Input resolution is smaller than target 320x180")

states_all = []
actions_all = []
timestamps_all = []
frame_indices_all = []
episode_indices_all = []
indices_all = []
task_indices_all = []
tasks_per_episode = []
task_to_index: dict[str, int] = {}
tasks_list: list[str] = []
video_done = 0
video_total = len(records) * len(CAMERA_KEYS)
episode_lengths = []

index_counter = 0

for ep_idx, record in enumerate(records):
    task_text = read_task_text(record)
    task_idx = task_to_index.get(task_text)
    if task_idx is None:
        task_idx = len(tasks_list)
        task_to_index[task_text] = task_idx
        tasks_list.append(task_text)
    tasks_per_episode.append(task_text)
    h5_files = glob.glob(os.path.join(record, "action", "*.h5"))
    if len(h5_files) != 1:
        raise SystemExit(f"Expected 1 h5 in {record}, found {len(h5_files)}")
    h5_path = h5_files[0]

    video_files = sorted(glob.glob(os.path.join(record, "video", "*.mp4")))
    if len(video_files) < len(CAMERA_KEYS):
        raise SystemExit(f"Expected at least {len(CAMERA_KEYS)} videos in {record}, found {len(video_files)}")

    video_paths = {}
    frame_counts = []
    for vf in video_files:
        key = parse_camera_key(vf)
        if key is None:
            continue
        video_paths[key] = vf
        frame_counts.append(get_frame_count(vf))

    if len(video_paths) != len(CAMERA_KEYS):
        raise SystemExit(f"Missing expected cameras in {record}: {sorted(video_paths.keys())}")

    frames_min_30 = min(frame_counts)
    if frames_min_30 <= 1:
        raise SystemExit(f"No frames in {record}")
    frames_min_15 = int(frames_min_30 * FPS_OUT / FPS_IN)
    if frames_min_15 <= 1:
        raise SystemExit(f"No frames after downsample in {record}")

    with h5py.File(h5_path, "r") as f:
        ts = f["timestamps"][:].astype(np.float64)
        franka_pos = f["franka/positions"][:].astype(np.float32)
        franka_vel = f["franka/velocities"][:].astype(np.float32)
        gripper_pos = f["gripper/positions"][:].astype(np.float32)

    gripper_avg = gripper_pos.mean(axis=1)
    gripper_norm = np.clip(gripper_avg / max_gripper, 0.0, 1.0).astype(np.float32)

    frame_times = ts[0] + (np.arange(frames_min_15, dtype=np.float64) / FPS_OUT)
    idx = nearest_indices(ts, frame_times)

    state_arr = np.concatenate([franka_pos[idx], gripper_norm[idx, None]], axis=1).astype(np.float32)
    action_arr = np.concatenate([franka_vel[idx], gripper_norm[idx, None]], axis=1).astype(np.float32)

    # Re-encode videos to AV1, downsample to 15 fps and 320x180 using bicubic resize.
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
                "-vf",
                f"fps={FPS_OUT_INT},scale={width_out}:{height_out}:flags=bicubic,setsar=1",
                "-frames:v",
                str(frames_min_15),
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
                "-an",
                out_path,
            ],
            check=True,
        )
        video_done += 1
        print(f"PROGRESS convert_video {video_done} {video_total}", flush=True)

    # Build per-episode parquet
    df_ep = pl.DataFrame(
        {
            "joint_position": pl.Series(
                state_arr[:, :7].tolist(), dtype=pl.Array(pl.Float32, 7)
            ),
            "gripper_position": pl.Series(
                state_arr[:, 7:8].tolist(), dtype=pl.Array(pl.Float32, 1)
            ),
            "actions": pl.Series(
                action_arr.tolist(), dtype=pl.Array(pl.Float32, 8)
            ),
            "timestamp": pl.Series(
                (np.arange(frames_min_15, dtype=np.float32) / FPS_OUT).tolist(),
                dtype=pl.Float32,
            ),
            "frame_index": pl.Series(list(range(frames_min_15)), dtype=pl.Int64),
            "episode_index": pl.Series([ep_idx] * frames_min_15, dtype=pl.Int64),
            "index": pl.Series(list(range(index_counter, index_counter + frames_min_15)), dtype=pl.Int64),
            "task_index": pl.Series([task_idx] * frames_min_15, dtype=pl.Int64),
        }
    )
    df_ep.write_parquet(
        os.path.join(data_dir, f"episode_{ep_idx:06d}.parquet"),
        compression="uncompressed",
    )

    for i in range(frames_min_15):
        states_all.append(state_arr[i])
        actions_all.append(action_arr[i])
        timestamps_all.append(float(i / FPS_OUT))
        frame_indices_all.append(i)
        episode_indices_all.append(ep_idx)
        indices_all.append(index_counter + i)
        task_indices_all.append(task_idx)

    index_counter += frames_min_15
    episode_lengths.append(frames_min_15)

# tasks.jsonl
with open(os.path.join(meta_dir, "tasks.jsonl"), "w", encoding="utf-8") as f:
    for idx, task in enumerate(tasks_list):
        f.write(json.dumps({"task_index": idx, "task": task}) + "\n")

# episodes.jsonl
with open(os.path.join(meta_dir, "episodes.jsonl"), "w", encoding="utf-8") as f:
    for ep_idx, length in enumerate(episode_lengths):
        f.write(json.dumps({"episode_index": ep_idx, "tasks": [tasks_per_episode[ep_idx]], "length": length}) + "\n")

# stats.json
state_arr_all = np.asarray(states_all, dtype=np.float32)
action_arr_all = np.asarray(actions_all, dtype=np.float32)

timestamp_arr = np.asarray(timestamps_all, dtype=np.float32)
frame_arr = np.asarray(frame_indices_all, dtype=np.int64)
episode_arr = np.asarray(episode_indices_all, dtype=np.int64)
index_arr = np.asarray(indices_all, dtype=np.int64)
task_arr = np.asarray(task_indices_all, dtype=np.int64)

stats = {
    "joint_position": stats_matrix(state_arr_all[:, :7]),
    "gripper_position": stats_matrix(state_arr_all[:, 7:8]),
    "actions": stats_matrix(action_arr_all),
    "timestamp": stats_vector(timestamp_arr),
    "frame_index": stats_vector(frame_arr.astype(np.float32)),
    "episode_index": stats_vector(episode_arr.astype(np.float32)),
    "index": stats_vector(index_arr.astype(np.float32)),
    "task_index": stats_vector(task_arr.astype(np.float32)),
    "exterior_image_1_left": {
        "min": [[[0.0]], [[0.0]], [[0.0]]],
        "max": [[[1.0]], [[1.0]], [[1.0]]],
        "mean": [[[0.5]], [[0.5]], [[0.5]]],
        "std": [[[0.2]], [[0.2]], [[0.2]]],
        "count": [len(states_all)],
    },
    "exterior_image_2_left": {
        "min": [[[0.0]], [[0.0]], [[0.0]]],
        "max": [[[1.0]], [[1.0]], [[1.0]]],
        "mean": [[[0.5]], [[0.5]], [[0.5]]],
        "std": [[[0.2]], [[0.2]], [[0.2]]],
        "count": [len(states_all)],
    },
    "wrist_image_left": {
        "min": [[[0.0]], [[0.0]], [[0.0]]],
        "max": [[[1.0]], [[1.0]], [[1.0]]],
        "mean": [[[0.5]], [[0.5]], [[0.5]]],
        "std": [[[0.2]], [[0.2]], [[0.2]]],
        "count": [len(states_all)],
    },
}

with open(os.path.join(meta_dir, "stats.json"), "w", encoding="utf-8") as f:
    json.dump(stats, f, indent=2)

# info.json
features = {
    "exterior_image_1_left": {
        "dtype": "video",
        "shape": [height_out, width_out, 3],
        "names": ["height", "width", "rgb"],
        "info": {
            "video.fps": FPS_OUT,
            "video.height": height_out,
            "video.width": width_out,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    },
    "exterior_image_2_left": {
        "dtype": "video",
        "shape": [height_out, width_out, 3],
        "names": ["height", "width", "rgb"],
        "info": {
            "video.fps": FPS_OUT,
            "video.height": height_out,
            "video.width": width_out,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    },
    "wrist_image_left": {
        "dtype": "video",
        "shape": [height_out, width_out, 3],
        "names": ["height", "width", "rgb"],
        "info": {
            "video.fps": FPS_OUT,
            "video.height": height_out,
            "video.width": width_out,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        },
    },
    "joint_position": {
        "dtype": "float32",
        "shape": [7],
        "names": ["joint_position"],
    },
    "gripper_position": {
        "dtype": "float32",
        "shape": [1],
        "names": ["gripper_position"],
    },
    "actions": {
        "dtype": "float32",
        "shape": [8],
        "names": ["actions"],
    },
    "timestamp": {
        "dtype": "float32",
        "shape": [1],
        "names": None,
    },
    "frame_index": {
        "dtype": "int64",
        "shape": [1],
        "names": None,
    },
    "episode_index": {
        "dtype": "int64",
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
    "codebase_version": "v2.1",
    "robot_type": "panda",
    "total_episodes": len(records),
    "total_frames": int(len(states_all)),
    "total_tasks": len(tasks_list),
    "total_videos": len(records) * len(CAMERA_KEYS),
    "total_chunks": 1,
    "chunks_size": 1000,
    "fps": FPS_OUT_INT,
    "splits": {"train": f"0:{len(records)}"},
    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
    "features": features,
}

with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as f:
    json.dump(info, f, indent=2)

print("Done")
print(f"Output: {OUT_ROOT}")
print(f"Gripper max used for normalization: {max_gripper:.6f}")
