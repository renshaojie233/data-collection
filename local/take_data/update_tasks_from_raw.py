#!/usr/bin/env python3
import json
from pathlib import Path

import polars as pl

DATA_ROOT = Path("/home/ubuntu/take_data/data")
DATASET_ROOT = Path("/home/ubuntu/take_data/lerobot_data/lerobot_take_droid_pi05_15fps")


def read_task_text(record_path: Path) -> str:
    task_path = record_path / "task.txt"
    if not task_path.exists():
        return "default"
    text = task_path.read_text(encoding="utf-8").strip()
    return text or "default"


def main() -> None:
    records = sorted(DATA_ROOT.glob("record_*"))
    if not records:
        raise SystemExit(f"No records found in {DATA_ROOT}")

    tasks_list: list[str] = []
    task_to_index: dict[str, int] = {}
    tasks_per_episode: list[str] = []

    for record in records:
        task_text = read_task_text(record)
        task_idx = task_to_index.get(task_text)
        if task_idx is None:
            task_idx = len(tasks_list)
            task_to_index[task_text] = task_idx
            tasks_list.append(task_text)
        tasks_per_episode.append(task_text)

    meta_dir = DATASET_ROOT / "meta"
    data_dir = DATASET_ROOT / "data"

    episodes = []
    for line in (meta_dir / "episodes.jsonl").read_text(encoding="utf-8").splitlines():
        episodes.append(json.loads(line))

    if len(episodes) != len(tasks_per_episode):
        raise SystemExit(
            f"Episode count mismatch: meta has {len(episodes)}, tasks has {len(tasks_per_episode)}"
        )

    # Update parquet task_index column per episode.
    for ep in episodes:
        ep_idx = ep["episode_index"]
        ep_chunk = ep_idx // 1000
        parquet_path = data_dir / f"chunk-{ep_chunk:03d}" / f"episode_{ep_idx:06d}.parquet"
        if not parquet_path.exists():
            raise SystemExit(f"Missing parquet: {parquet_path}")
        task_text = tasks_per_episode[ep_idx]
        task_idx = task_to_index[task_text]
        df = pl.read_parquet(parquet_path)
        df = df.with_columns(pl.lit(task_idx).cast(pl.Int64).alias("task_index"))
        df.write_parquet(parquet_path, compression="uncompressed")

    # Update tasks.jsonl
    tasks_lines = [json.dumps({"task_index": idx, "task": task}) for idx, task in enumerate(tasks_list)]
    (meta_dir / "tasks.jsonl").write_text("\n".join(tasks_lines) + "\n", encoding="utf-8")

    # Update episodes.jsonl (tasks only, keep length)
    episodes_lines = []
    for ep in episodes:
        ep_idx = ep["episode_index"]
        episodes_lines.append(
            json.dumps(
                {"episode_index": ep_idx, "tasks": [tasks_per_episode[ep_idx]], "length": ep["length"]}
            )
        )
    (meta_dir / "episodes.jsonl").write_text("\n".join(episodes_lines) + "\n", encoding="utf-8")

    # Update info.json total_tasks
    info_path = meta_dir / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    info["total_tasks"] = len(tasks_list)
    info_path.write_text(json.dumps(info, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
