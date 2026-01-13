#!/usr/bin/env python3
import glob
import json
import os
import shutil
import subprocess
from pathlib import Path

import cv2
import h5py
import imageio.v3 as iio
import imageio_ffmpeg as ffmpeg
import numpy as np
import polars as pl

DATA_ROOT = "/home/ubuntu/take_data/data"
OUT_ROOT = "/home/ubuntu/take_data/lerobot_data/lerobot_take_droid_pi05_15fps_v30"
FPS_IN = 30.0
FPS_OUT = 15.0
FPS_OUT_INT = int(FPS_OUT)

CAMERA_PREFIX_TO_KEY = {
    "camera_1_": "observation.images.exterior_image_1_left",
    "camera_3_": "observation.images.exterior_image_2_left",
    "camera_2_": "observation.images.wrist_image_left",
}
CAMERA_KEYS = [
    "observation.images.exterior_image_1_left",
    "observation.images.exterior_image_2_left",
    "observation.images.wrist_image_left",
]

WIDTH_OUT, HEIGHT_OUT = 320, 180

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


def estimate_num_samples(
    dataset_len: int,
    min_num_samples: int = 100,
    max_num_samples: int = 10_000,
    power: float = 0.75,
) -> int:
    if dataset_len < min_num_samples:
        min_num_samples = dataset_len
    return max(min_num_samples, min(int(dataset_len**power), max_num_samples))


def sample_indices(data_len: int) -> list[int]:
    num_samples = estimate_num_samples(data_len)
    if data_len <= 1:
        return [0]
    return np.round(np.linspace(0, data_len - 1, num_samples)).astype(int).tolist()


def auto_downsample_height_width(
    img: np.ndarray, target_size: int = 150, max_size_threshold: int = 300
) -> np.ndarray:
    _, height, width = img.shape
    if max(width, height) < max_size_threshold:
        return img
    downsample_factor = int(width / target_size) if width > height else int(height / target_size)
    return img[:, ::downsample_factor, ::downsample_factor]


def compute_feature_stats(
    array: np.ndarray, axis: tuple[int, ...], keepdims: bool
) -> dict[str, np.ndarray]:
    return {
        "min": np.min(array, axis=axis, keepdims=keepdims),
        "max": np.max(array, axis=axis, keepdims=keepdims),
        "mean": np.mean(array, axis=axis, keepdims=keepdims),
        "std": np.std(array, axis=axis, keepdims=keepdims),
        "count": np.array([len(array)]),
    }


def sample_video_frames(video_path: Path, indices: list[int]) -> np.ndarray:
    indices = sorted(set(indices))
    if not indices:
        raise ValueError(f"No indices to sample for {video_path}")
    frames = []
    target_pos = 0
    last_index = indices[-1]
    for i, frame in enumerate(iio.imiter(video_path)):
        if i > last_index:
            break
        if i == indices[target_pos]:
            frame = frame.transpose(2, 0, 1)  # (H, W, C) -> (C, H, W)
            frame = auto_downsample_height_width(frame)
            frames.append(frame)
            target_pos += 1
            if target_pos >= len(indices):
                break
    if not frames:
        raise ValueError(f"No frames sampled from {video_path}")
    return np.stack(frames, axis=0)


def stats_for_video(video_path: Path, num_frames: int) -> dict[str, np.ndarray]:
    indices = sample_indices(num_frames)
    frames = sample_video_frames(video_path, indices)
    stats = compute_feature_stats(frames, axis=(0, 2, 3), keepdims=True)
    # Normalize to [0, 1] and remove batch dim.
    return {
        k: (v if k == "count" else np.squeeze(v / 255.0, axis=0))
        for k, v in stats.items()
    }


def aggregate_feature_stats(stats_ft_list: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    means = np.stack([s["mean"] for s in stats_ft_list])
    variances = np.stack([s["std"] ** 2 for s in stats_ft_list])
    counts = np.stack([s["count"] for s in stats_ft_list])
    total_count = counts.sum(axis=0)
    while counts.ndim < means.ndim:
        counts = np.expand_dims(counts, axis=-1)
    weighted_means = means * counts
    total_mean = weighted_means.sum(axis=0) / total_count
    delta_means = means - total_mean
    weighted_variances = (variances + delta_means**2) * counts
    total_variance = weighted_variances.sum(axis=0) / total_count
    return {
        "min": np.min(np.stack([s["min"] for s in stats_ft_list]), axis=0),
        "max": np.max(np.stack([s["max"] for s in stats_ft_list]), axis=0),
        "mean": total_mean,
        "std": np.sqrt(total_variance),
        "count": total_count,
    }


def to_jsonable(stats: dict[str, dict[str, np.ndarray]]) -> dict:
    out: dict[str, dict[str, list]] = {}
    for key, values in stats.items():
        out[key] = {}
        for stat_key, arr in values.items():
            out[key][stat_key] = arr.tolist() if isinstance(arr, np.ndarray) else arr
    return out


records = sorted(glob.glob(os.path.join(DATA_ROOT, "record_*")))
if not records:
    raise SystemExit(f"No records found under {DATA_ROOT}")
print(f"Found {len(records)} record(s)")

# Prepare output dirs
if os.path.exists(OUT_ROOT):
    shutil.rmtree(OUT_ROOT)
ensure_dir(OUT_ROOT)
meta_dir = os.path.join(OUT_ROOT, "meta")
ensure_dir(meta_dir)
data_dir = os.path.join(OUT_ROOT, "data", "chunk-000")
ensure_dir(data_dir)
video_root = os.path.join(OUT_ROOT, "videos")
for key in CAMERA_KEYS:
    ensure_dir(os.path.join(video_root, key, "chunk-000"))

tmp_root = os.path.join(OUT_ROOT, "_tmp_videos")
if os.path.exists(tmp_root):
    shutil.rmtree(tmp_root)
ensure_dir(tmp_root)
for key in CAMERA_KEYS:
    ensure_dir(os.path.join(tmp_root, key))

# Determine output size from first record camera_1
first_video = sorted(glob.glob(os.path.join(records[0], "video", "camera_1_*.mp4")))[0]
cap0 = cv2.VideoCapture(first_video)
if not cap0.isOpened():
    raise SystemExit(f"Failed to open {first_video}")
width_in = int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
height_in = int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap0.release()
if width_in < WIDTH_OUT or height_in < HEIGHT_OUT:
    raise SystemExit("Input resolution is smaller than target 320x180")

states_all = []
actions_all = []
timestamps_all = []
episode_indices_all = []
frame_indices_all = []
next_rewards_all = []
next_dones_all = []
indices_all = []
task_indices_all = []

task_to_index: dict[str, int] = {}
tasks_list: list[str] = []
episode_lengths: list[int] = []

video_segments: dict[str, list[str]] = {key: [] for key in CAMERA_KEYS}
video_stats_by_key: dict[str, list[dict[str, np.ndarray]]] = {key: [] for key in CAMERA_KEYS}

video_done = 0
video_total = len(records) * len(CAMERA_KEYS)
stats_done = 0
stats_total = len(records) * len(CAMERA_KEYS)

index_counter = 0

for ep_idx, record in enumerate(records):
    task_text = read_task_text(record)
    task_idx = task_to_index.get(task_text)
    if task_idx is None:
        task_idx = len(tasks_list)
        task_to_index[task_text] = task_idx
        tasks_list.append(task_text)

    h5_files = glob.glob(os.path.join(record, "action", "*.h5"))
    if len(h5_files) != 1:
        raise SystemExit(f"Expected 1 h5 in {record}, found {len(h5_files)}")
    h5_path = h5_files[0]

    video_files = sorted(glob.glob(os.path.join(record, "video", "*.mp4")))
    if len(video_files) < len(CAMERA_KEYS):
        raise SystemExit(
            f"Expected at least {len(CAMERA_KEYS)} videos in {record}, found {len(video_files)}"
        )

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

    frame_times = ts[0] + (np.arange(frames_min_15, dtype=np.float64) / FPS_OUT)
    idx = nearest_indices(ts, frame_times)

    state_arr = franka_pos[idx].astype(np.float32)
    action_arr = franka_vel[idx].astype(np.float32)

    episode_start = len(next_dones_all)
    for i in range(frames_min_15):
        states_all.append(state_arr[i].tolist())
        actions_all.append(action_arr[i].tolist())
        timestamps_all.append(float(i / FPS_OUT))
        episode_indices_all.append(ep_idx)
        frame_indices_all.append(i)
        next_rewards_all.append(0.0)
        next_dones_all.append(False)
        indices_all.append(index_counter)
        task_indices_all.append(task_idx)
        index_counter += 1
    next_dones_all[episode_start + frames_min_15 - 1] = True
    episode_lengths.append(frames_min_15)

    for key in CAMERA_KEYS:
        out_path = os.path.join(tmp_root, key, f"episode_{ep_idx:06d}.mp4")
        subprocess.run(
            [
                FFMPEG_EXE,
                "-y",
                "-loglevel",
                "error",
                "-i",
                video_paths[key],
                "-vf",
                f"fps={FPS_OUT_INT},scale={WIDTH_OUT}:{HEIGHT_OUT}:flags=bicubic,setsar=1",
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
        video_segments[key].append(out_path)
        video_done += 1
        print(f"PROGRESS convert_video {video_done} {video_total}", flush=True)

print("Building parquet")
df = pl.DataFrame(
    {
        "observation.state": pl.Series(
            states_all, dtype=pl.List(pl.Float32)
        ),
        "action": pl.Series(actions_all, dtype=pl.List(pl.Float32)),
        "timestamp": pl.Series(timestamps_all, dtype=pl.Float32),
        "episode_index": pl.Series(episode_indices_all, dtype=pl.Int64),
        "frame_index": pl.Series(frame_indices_all, dtype=pl.Int64),
        "next.reward": pl.Series(next_rewards_all, dtype=pl.Float32),
        "next.done": pl.Series(next_dones_all, dtype=pl.Boolean),
        "index": pl.Series(indices_all, dtype=pl.Int64),
        "task_index": pl.Series(task_indices_all, dtype=pl.Int64),
    }
)
df.write_parquet(os.path.join(data_dir, "file-000.parquet"))

pl.DataFrame(
    {
        "task_index": list(range(len(tasks_list))),
        "__index_level_0__": tasks_list,
    }
).write_parquet(os.path.join(meta_dir, "tasks.parquet"))

print("Computing video stats")
for ep_idx, frames_len in enumerate(episode_lengths):
    for key in CAMERA_KEYS:
        video_path = Path(video_segments[key][ep_idx])
        stats = stats_for_video(video_path, frames_len)
        video_stats_by_key[key].append(stats)
        stats_done += 1
        print(f"PROGRESS stats_video {stats_done} {stats_total}", flush=True)

video_stats = {
    key: aggregate_feature_stats(video_stats_by_key[key]) for key in CAMERA_KEYS
}

state_arr_all = np.asarray(states_all, dtype=np.float32)
action_arr_all = np.asarray(actions_all, dtype=np.float32)
timestamp_arr = np.asarray(timestamps_all, dtype=np.float32)
episode_arr = np.asarray(episode_indices_all, dtype=np.int64)
frame_arr = np.asarray(frame_indices_all, dtype=np.int64)
reward_arr = np.asarray(next_rewards_all, dtype=np.float32)
done_arr = np.asarray(next_dones_all, dtype=np.bool_)
index_arr = np.asarray(indices_all, dtype=np.int64)
task_arr = np.asarray(task_indices_all, dtype=np.int64)

stats = {
    "observation.state": stats_matrix(state_arr_all),
    "action": stats_matrix(action_arr_all),
    "timestamp": stats_vector(timestamp_arr),
    "episode_index": stats_vector(episode_arr.astype(np.float32)),
    "frame_index": stats_vector(frame_arr.astype(np.float32)),
    "next.reward": stats_vector(reward_arr),
    "next.done": stats_bool(done_arr),
    "index": stats_vector(index_arr.astype(np.float32)),
    "task_index": stats_vector(task_arr.astype(np.float32)),
}
stats.update(to_jsonable(video_stats))

with open(os.path.join(meta_dir, "stats.json"), "w", encoding="utf-8") as f:
    json.dump(stats, f, indent=2)

print("Concatenating videos")
for key in CAMERA_KEYS:
    out_path = os.path.join(video_root, key, "chunk-000", "file-000.mp4")
    list_path = os.path.join(tmp_root, f"concat_{key.replace('.', '_')}.txt")
    with open(list_path, "w", encoding="utf-8") as f:
        for segment in video_segments[key]:
            f.write(f"file '{segment}'\n")
    cmd = [
        FFMPEG_EXE,
        "-y",
        "-loglevel",
        "error",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        list_path,
        "-fflags",
        "+genpts",
        "-c",
        "copy",
        out_path,
    ]
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        subprocess.run(
            [
                FFMPEG_EXE,
                "-y",
                "-loglevel",
                "error",
                "-f",
                "concat",
                "-safe",
                "0",
                "-i",
                list_path,
                "-vf",
                f"fps={FPS_OUT_INT},scale={WIDTH_OUT}:{HEIGHT_OUT}:flags=bicubic,setsar=1",
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

feature_names = [f"motor_{i}" for i in range(7)]

features = {}
for key in CAMERA_KEYS:
    features[key] = {
        "dtype": "video",
        "shape": [HEIGHT_OUT, WIDTH_OUT, 3],
        "names": ["height", "width", "channel"],
        "video_info": {
            "video.fps": FPS_OUT,
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
            "shape": [7],
            "names": {"motors": feature_names},
            "fps": FPS_OUT,
        },
        "action": {
            "dtype": "float32",
            "shape": [7],
            "names": {"motors": feature_names},
            "fps": FPS_OUT,
        },
        "timestamp": {
            "dtype": "float32",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "episode_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "frame_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "next.reward": {
            "dtype": "float32",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "next.done": {
            "dtype": "bool",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
        "task_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
            "fps": FPS_OUT,
        },
    }
)

info = {
    "codebase_version": "v3.0",
    "robot_type": "unknown",
    "total_episodes": len(records),
    "total_frames": int(len(states_all)),
    "total_tasks": len(tasks_list),
    "chunks_size": 1000,
    "fps": FPS_OUT_INT,
    "splits": {"train": f"0:{len(records)}"},
    "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
    "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
    "features": features,
}
info["data_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "data"))
info["video_files_size_in_mb"] = dir_size_mb(os.path.join(OUT_ROOT, "videos"))

with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as f:
    json.dump(info, f, indent=2)

shutil.rmtree(tmp_root, ignore_errors=True)

print("Done")
print(f"Output: {OUT_ROOT}")
