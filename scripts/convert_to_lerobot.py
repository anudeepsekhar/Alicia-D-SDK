#!/usr/bin/env python3
"""Convert collected VLA dataset to LeRobot v2 HuggingFace dataset format.

Reads the dataset produced by ``collect_data.py`` and writes a
HuggingFace-compatible LeRobot v2 dataset that can be loaded directly
with ``lerobot.common.datasets.lerobot_dataset.LeRobotDataset``.

Usage:

  # Basic conversion
  python scripts/convert_to_lerobot.py --src ./vla_dataset --dst ./lerobot_dataset

  # Push to HuggingFace Hub
  python scripts/convert_to_lerobot.py --src ./vla_dataset --dst ./lerobot_dataset \
      --repo-id your-username/alicia-d-pickplace

Requires: pip install lerobot datasets pyarrow
"""

import argparse
import json
import os
import glob

import numpy as np
import pyarrow.parquet as pq


def load_episodes_meta(src_dir: str):
    """Load episode metadata from the source dataset."""
    episodes_file = os.path.join(src_dir, "meta", "episodes.jsonl")
    episodes = []
    with open(episodes_file) as f:
        for line in f:
            line = line.strip()
            if line:
                episodes.append(json.loads(line))
    return sorted(episodes, key=lambda e: e["episode_index"])


def load_info(src_dir: str):
    info_file = os.path.join(src_dir, "meta", "info.json")
    with open(info_file) as f:
        return json.load(f)


def compute_stats(all_states: np.ndarray, all_actions: np.ndarray):
    """Compute per-column min/max/mean/std for normalization."""
    state_labels = [f"observation.state.joint_{i}" for i in range(6)] + ["observation.state.gripper"]
    action_labels = [f"action.joint_{i}" for i in range(6)] + ["action.gripper"]

    stats = {}
    for i, label in enumerate(state_labels):
        col = all_states[:, i]
        stats[label] = {
            "min": float(np.min(col)),
            "max": float(np.max(col)),
            "mean": float(np.mean(col)),
            "std": float(np.std(col)),
        }
    for i, label in enumerate(action_labels):
        col = all_actions[:, i]
        stats[label] = {
            "min": float(np.min(col)),
            "max": float(np.max(col)),
            "mean": float(np.mean(col)),
            "std": float(np.std(col)),
        }
    return stats


def convert(src_dir: str, dst_dir: str, repo_id: str = None):
    info = load_info(src_dir)
    episodes_meta = load_episodes_meta(src_dir)

    os.makedirs(os.path.join(dst_dir, "meta"), exist_ok=True)
    os.makedirs(os.path.join(dst_dir, "data"), exist_ok=True)

    has_video = bool(glob.glob(os.path.join(src_dir, "videos", "observation.image", "*.mp4")))
    if has_video:
        os.makedirs(os.path.join(dst_dir, "videos", "observation.image"), exist_ok=True)

    all_states_list = []
    all_actions_list = []
    total_frames = 0

    for ep_meta in episodes_meta:
        ep_idx = ep_meta["episode_index"]
        ep_tag = f"episode_{ep_idx:06d}"

        parquet_src = os.path.join(src_dir, "data", f"{ep_tag}.parquet")
        json_src = os.path.join(src_dir, "data", f"{ep_tag}.json")

        if os.path.exists(parquet_src):
            table = pq.read_table(parquet_src)
            df = table.to_pandas()
        elif os.path.exists(json_src):
            import pandas as pd
            with open(json_src) as f:
                rows = json.load(f)
            df = pd.DataFrame(rows)
        else:
            print(f"  WARNING: No data file for {ep_tag}, skipping")
            continue

        n = len(df)
        total_frames += n

        state_cols = [f"observation.state.joint_{i}" for i in range(6)]
        if all(c in df.columns for c in state_cols):
            states = df[state_cols + ["observation.state.gripper"]].values.astype(np.float32)
        elif "observation.state" in df.columns:
            states = np.array(df["observation.state"].tolist(), dtype=np.float32)
        else:
            print(f"  WARNING: Cannot find state columns in {ep_tag}")
            continue

        action_cols = [f"action.joint_{i}" for i in range(6)]
        if all(c in df.columns for c in action_cols):
            actions = df[action_cols + ["action.gripper"]].values.astype(np.float32)
        elif "action" in df.columns:
            actions = np.array(df["action"].tolist(), dtype=np.float32)
        else:
            print(f"  WARNING: Cannot find action columns in {ep_tag}")
            continue

        all_states_list.append(states)
        all_actions_list.append(actions)

        # Copy video if present
        if has_video:
            vid_src = os.path.join(src_dir, "videos", "observation.image", f"{ep_tag}.mp4")
            vid_dst = os.path.join(dst_dir, "videos", "observation.image", f"{ep_tag}.mp4")
            if os.path.exists(vid_src):
                import shutil
                shutil.copy2(vid_src, vid_dst)

        # Write per-episode parquet with LeRobot-expected columns
        import pyarrow as pa

        out_dict = {
            "timestamp": df["timestamp"].values.astype(np.float64),
            "episode_index": np.full(n, ep_idx, dtype=np.int64),
            "frame_index": np.arange(n, dtype=np.int64),
            "index": np.arange(total_frames - n, total_frames, dtype=np.int64),
        }

        for i in range(6):
            out_dict[f"observation.state.joint_{i}"] = states[:, i]
        out_dict["observation.state.gripper"] = states[:, 6]

        for i in range(6):
            out_dict[f"action.joint_{i}"] = actions[:, i]
        out_dict["action.gripper"] = actions[:, 6]

        if has_video:
            out_dict["observation.image.frame_index"] = np.arange(n, dtype=np.int64)

        out_table = pa.table(out_dict)
        pq.write_table(out_table, os.path.join(dst_dir, "data", f"{ep_tag}.parquet"))

        print(f"  Converted {ep_tag}: {n} frames, task=\"{ep_meta.get('task', '')}\"")

    # --- Stats ---
    if all_states_list:
        all_states = np.concatenate(all_states_list)
        all_actions = np.concatenate(all_actions_list)
        stats = compute_stats(all_states, all_actions)
    else:
        stats = {}

    with open(os.path.join(dst_dir, "meta", "stats.json"), "w") as f:
        json.dump(stats, f, indent=2)

    # --- Episodes JSONL ---
    with open(os.path.join(dst_dir, "meta", "episodes.jsonl"), "w") as f:
        for ep_meta in episodes_meta:
            f.write(json.dumps(ep_meta) + "\n")

    # --- Info JSON ---
    lerobot_info = {
        "codebase_version": "v2.0",
        "robot_type": "alicia_d_6dof",
        "total_episodes": len(episodes_meta),
        "total_frames": total_frames,
        "fps": info.get("fps", 10),
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [7],
                "names": [
                    "joint_0", "joint_1", "joint_2",
                    "joint_3", "joint_4", "joint_5",
                    "gripper",
                ],
            },
            "action": {
                "dtype": "float32",
                "shape": [7],
                "names": [
                    "joint_0", "joint_1", "joint_2",
                    "joint_3", "joint_4", "joint_5",
                    "gripper",
                ],
            },
        },
        "tasks": info.get("tasks", []),
    }

    if has_video:
        lerobot_info["features"]["observation.image"] = {
            "dtype": "video",
            "shape": [
                info.get("image_height", 480),
                info.get("image_width", 640),
                3,
            ],
            "video_info": {
                "video.fps": info.get("fps", 10),
                "video.codec": "mp4v",
            },
        }

    if repo_id:
        lerobot_info["repo_id"] = repo_id

    with open(os.path.join(dst_dir, "meta", "info.json"), "w") as f:
        json.dump(lerobot_info, f, indent=2)

    print(f"\nConversion complete: {len(episodes_meta)} episodes, {total_frames} frames")
    print(f"Output: {dst_dir}")

    if repo_id:
        print(f"\nTo push to HuggingFace Hub:")
        print(f"  huggingface-cli upload {repo_id} {dst_dir}")


def main():
    parser = argparse.ArgumentParser(description="Convert VLA dataset to LeRobot format")
    parser.add_argument("--src", required=True, help="Source dataset directory (from collect_data.py)")
    parser.add_argument("--dst", required=True, help="Destination directory for LeRobot dataset")
    parser.add_argument("--repo-id", default=None, help="HuggingFace repo ID (e.g. user/dataset-name)")
    args = parser.parse_args()

    convert(args.src, args.dst, args.repo_id)


if __name__ == "__main__":
    main()
