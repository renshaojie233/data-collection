#!/usr/bin/env python3
import argparse
import json
import os
import re

import h5py


FR3_JOINT_ORDER = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]
JOINT_NUMBER_RE = re.compile(r"joint([1-7])$", re.IGNORECASE)


def build_order(names):
    if not names or len(names) < 7:
        return None

    if all(name in names for name in FR3_JOINT_ORDER):
        return [names.index(name) for name in FR3_JOINT_ORDER]

    index_map = {}
    for idx, name in enumerate(names):
        match = JOINT_NUMBER_RE.search(name)
        if not match:
            continue
        joint_num = int(match.group(1))
        if 1 <= joint_num <= 7 and joint_num not in index_map:
            index_map[joint_num] = idx

    if len(index_map) == 7:
        return [index_map[i] for i in range(1, 8)]
    return None


def reorder_joint_block(joint_block, order):
    if not order:
        return False
    changed = False
    for field in ("position", "velocity", "effort"):
        values = joint_block.get(field)
        if values is None or len(values) != 7:
            continue
        new_values = [values[i] for i in order]
        if new_values != values:
            joint_block[field] = new_values
            changed = True
    if joint_block.get("names") != FR3_JOINT_ORDER:
        joint_block["names"] = list(FR3_JOINT_ORDER)
        changed = True
    return changed


def fix_json(json_path, dry_run=False):
    with open(json_path, "r", encoding="utf-8") as f:
        payload = json.load(f)

    frames = payload.get("data")
    if not isinstance(frames, list):
        return False, None

    franka_order = None
    franka_orders_seen = set()

    for frame in frames:
        franka = frame.get("franka_joints") or {}
        names = franka.get("names") or []
        order = build_order(names)
        if order:
            franka_orders_seen.add(tuple(order))
            if franka_order is None:
                franka_order = order

    changed = False
    for frame in frames:
        franka = frame.get("franka_joints")
        if isinstance(franka, dict):
            if reorder_joint_block(franka, franka_order):
                changed = True

    if franka_order and len(franka_orders_seen) > 1:
        print(f"[warn] {json_path}: multiple franka orders detected ({len(franka_orders_seen)})")
    if changed and not dry_run:
        tmp_path = json_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp_path, json_path)

    return changed, franka_order


def fix_h5(h5_path, order, dry_run=False):
    if not order or order == list(range(7)):
        return False
    changed = False
    with h5py.File(h5_path, "r+") as f:
        for key in ("franka/positions", "franka/velocities", "franka/efforts"):
            if key not in f:
                continue
            dset = f[key]
            if dset.ndim != 2 or dset.shape[1] < 7:
                continue
            data = dset[...]
            if data.shape[1] == 7:
                new_data = data[:, order]
            else:
                new_data = data.copy()
                new_data[:, :7] = data[:, order]
            if not dry_run:
                dset[...] = new_data
            changed = True
    return changed


def iter_json_files(base_dir):
    for root, _, files in os.walk(base_dir):
        for name in files:
            if not name.startswith("action_data_"):
                continue
            if not name.endswith(".json"):
                continue
            if "_replay" in name:
                continue
            yield os.path.join(root, name)


def main():
    parser = argparse.ArgumentParser(description="Fix FR3 Franka joint order in recorded data.")
    parser.add_argument(
        "--base-dir",
        default="/home/ubuntu/take_data/data",
        help="Base directory containing record_* folders.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Scan and report without writing changes.",
    )
    args = parser.parse_args()

    json_files = list(iter_json_files(args.base_dir))
    if not json_files:
        print(f"No action_data_*.json found under {args.base_dir}")
        return

    json_changed = 0
    h5_changed = 0
    for json_path in sorted(json_files):
        changed, order = fix_json(json_path, dry_run=args.dry_run)
        if changed:
            json_changed += 1
        h5_path = json_path[:-5] + ".h5"
        if os.path.exists(h5_path):
            if fix_h5(h5_path, order, dry_run=args.dry_run):
                h5_changed += 1

    suffix = " (dry run)" if args.dry_run else ""
    print(f"JSON files updated: {json_changed}{suffix}")
    print(f"HDF5 files updated: {h5_changed}{suffix}")


if __name__ == "__main__":
    main()
