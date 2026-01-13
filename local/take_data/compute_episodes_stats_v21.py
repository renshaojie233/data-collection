#!/usr/bin/env python3
import json
import math
from pathlib import Path

import imageio.v3 as iio
import numpy as np
import polars as pl

DATASET_ROOT = Path("/home/ubuntu/take_data/lerobot_data/lerobot_take_droid_pi05_15fps")
VIDEO_KEYS = [
    "exterior_image_1_left",
    "exterior_image_2_left",
    "wrist_image_left",
]


def estimate_num_samples(dataset_len: int, min_num_samples: int = 100, max_num_samples: int = 10_000, power: float = 0.75) -> int:
    if dataset_len < min_num_samples:
        min_num_samples = dataset_len
    return max(min_num_samples, min(int(dataset_len**power), max_num_samples))


def sample_indices(data_len: int) -> list[int]:
    num_samples = estimate_num_samples(data_len)
    if data_len <= 1:
        return [0]
    return np.round(np.linspace(0, data_len - 1, num_samples)).astype(int).tolist()


def auto_downsample_height_width(img: np.ndarray, target_size: int = 150, max_size_threshold: int = 300) -> np.ndarray:
    _, height, width = img.shape
    if max(width, height) < max_size_threshold:
        return img
    downsample_factor = int(width / target_size) if width > height else int(height / target_size)
    return img[:, ::downsample_factor, ::downsample_factor]


def compute_feature_stats(array: np.ndarray, axis: tuple[int, ...], keepdims: bool) -> dict[str, np.ndarray]:
    return {
        "min": np.min(array, axis=axis, keepdims=keepdims),
        "max": np.max(array, axis=axis, keepdims=keepdims),
        "mean": np.mean(array, axis=axis, keepdims=keepdims),
        "std": np.std(array, axis=axis, keepdims=keepdims),
        "count": np.array([len(array)]),
    }


def stats_for_numeric(arr: np.ndarray) -> dict[str, np.ndarray]:
    keepdims = arr.ndim == 1
    return compute_feature_stats(arr, axis=(0,), keepdims=keepdims)


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
        k: (v if k == "count" else np.squeeze(v / 255.0, axis=0)) for k, v in stats.items()
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


def aggregate_stats(episodes_stats: list[dict[str, dict[str, np.ndarray]]]) -> dict[str, dict[str, np.ndarray]]:
    data_keys = {key for stats in episodes_stats for key in stats}
    aggregated = {}
    for key in data_keys:
        stats_with_key = [stats[key] for stats in episodes_stats if key in stats]
        aggregated[key] = aggregate_feature_stats(stats_with_key)
    return aggregated


def to_jsonable(stats: dict[str, dict[str, np.ndarray]]) -> dict:
    out = {}
    for key, values in stats.items():
        out[key] = {}
        for stat_key, arr in values.items():
            out[key][stat_key] = arr.tolist() if isinstance(arr, np.ndarray) else arr
    return out


def main() -> None:
    meta_dir = DATASET_ROOT / "meta"
    data_dir = DATASET_ROOT / "data"
    videos_dir = DATASET_ROOT / "videos"

    episodes_stats = []
    episodes_file = meta_dir / "episodes.jsonl"
    if not episodes_file.exists():
        raise FileNotFoundError(f"Missing {episodes_file}")

    with episodes_file.open() as f:
        episodes = [json.loads(line) for line in f]
    print(f"Found {len(episodes)} episode(s)")
    video_done = 0
    video_total = len(episodes) * len(VIDEO_KEYS)

    for i, ep in enumerate(episodes):
        ep_idx = ep["episode_index"]
        ep_chunk = ep_idx // 1000
        parquet_path = data_dir / f"chunk-{ep_chunk:03d}" / f"episode_{ep_idx:06d}.parquet"
        if not parquet_path.exists():
            raise FileNotFoundError(f"Missing {parquet_path}")
        df = pl.read_parquet(parquet_path)
        ep_len = df.height

        stats = {}
        for col in df.columns:
            arr = np.asarray(df[col].to_list())
            stats[col] = stats_for_numeric(arr)

        for key in VIDEO_KEYS:
            video_path = videos_dir / f"chunk-{ep_chunk:03d}" / key / f"episode_{ep_idx:06d}.mp4"
            if not video_path.exists():
                raise FileNotFoundError(f"Missing {video_path}")
            stats[key] = stats_for_video(video_path, ep_len)
            video_done += 1
            print(f"PROGRESS stats_video {video_done} {video_total}", flush=True)

        episodes_stats.append(stats)

    # Write per-episode stats
    out_lines = []
    for ep, stats in zip(episodes, episodes_stats, strict=True):
        out_lines.append({"episode_index": ep["episode_index"], "stats": to_jsonable(stats)})
    (meta_dir / "episodes_stats.jsonl").write_text(
        "\n".join(json.dumps(line) for line in out_lines) + "\n",
        encoding="utf-8",
    )

    # Write aggregated stats
    aggregated = aggregate_stats(episodes_stats)
    (meta_dir / "stats.json").write_text(json.dumps(to_jsonable(aggregated), indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
