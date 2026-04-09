#!/usr/bin/env python3
"""
Replay one episode from a local .npy stream-compat dataset (from examples 13/14).

Dataset layout expected:
    <dataset.root>[/<dataset.repo_id>]/
        episode_000/
            actions.npy
            timestamps.npy  (optional, fallback to --control-hz when absent)
"""

import argparse
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]  # .../Alicia_custom
sys.path.insert(0, str(REPO_ROOT))

import alicia_d_sdk
# from alicia_d_sdk import create_stream_compat_robot
from alicia_d_sdk import create_robot


print("using:", alicia_d_sdk.__file__)


JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]


class AliciaReplayFollower:
    """Follower interface for replaying 14D actions onto Alicia stream-compat robots."""

    def __init__(
        self,
        arm_mode: str,
        left_port: Optional[str],
        right_port: Optional[str],
        baudrate: int,
        timeout: float,
        debug_mode: bool,
        write_speed_deg_s: float,
        write_gripper_speed_deg_s: float,
    ):
        self.arm_mode = arm_mode
        self.write_speed_deg_s = float(write_speed_deg_s)
        self.write_gripper_speed_deg_s = float(write_gripper_speed_deg_s)

        need_left = arm_mode in ("both", "left")
        need_right = arm_mode in ("both", "right")
        if need_left and not left_port:
            raise ValueError("--follower-left-port is required for arm-mode 'both' or 'left'.")
        if need_right and not right_port:
            raise ValueError("--follower-right-port is required for arm-mode 'both' or 'right'.")

        self.left_arm = None
        self.right_arm = None

        if need_left:
            # self.left_arm = create_stream_compat_robot(
            #     port=left_port,
            #     baudrate=baudrate,
            #     timeout=timeout,
            #     debug_mode=debug_mode,
            #     auto_connect=True,
            # )
            self.left_arm = create_robot(port=left_port)

        if need_right:
            # self.right_arm = create_stream_compat_robot(
            #     port=right_port,
            #     baudrate=baudrate,
            #     timeout=timeout,
            #     debug_mode=debug_mode,
            #     auto_connect=False,
            # )
            self.right_arm = create_robot(port=right_port)

    def connect(self) -> None:
        if self.left_arm is not None and not self.left_arm.connect():
            raise RuntimeError("Failed to connect left follower arm.")
        if self.right_arm is not None and not self.right_arm.connect():
            raise RuntimeError("Failed to connect right follower arm.")

    def disconnect(self) -> None:
        if self.left_arm is not None:
            self.left_arm.disconnect()
        if self.right_arm is not None:
            self.right_arm.disconnect()

    def _write_arm(self, arm, action: Dict[str, float], side: str) -> None:
        joints_deg = [float(action[f"{side}_joint_{i}.pos"]) for i in range(1, 7)]
        gripper = float(action[f"{side}_gripper.pos"])
        # ok = arm.set_robot_state(
        #     target_joints=joints_deg,
        #     gripper_value=gripper,
        #     joint_format="deg",
        #     speed_deg_s=self.write_speed_deg_s,
        #     gripper_speed_deg_s=self.write_gripper_speed_deg_s,
        #     wait_for_completion=False,
        # )
        ok = arm.set_robot_state(
            target_joints=joints_deg,
            gripper_value=gripper,
            joint_format="deg",
            speed_deg_s=60,
            gripper_speed_deg_s=60,
            wait_for_completion=False,
        )
        if not ok:
            raise RuntimeError(f"Failed to command {side} arm.")

    def send_action(self, action: Dict[str, float]) -> None:
        if self.left_arm is not None:
            self._write_arm(self.left_arm, action, "left")
        if self.right_arm is not None:
            self._write_arm(self.right_arm, action, "right")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay one .npy Alicia episode.")

    p.add_argument("--dataset-root", default="/home/rnarasim/lerobot_data")
    # p.add_argument("--dataset-root", default="/tmp/lerobot_mock")
    p.add_argument(
        "--dataset-repo-id",
        default=None,
        help="Optional dataset subdirectory under --dataset-root (kept for compatibility).",
    )
    p.add_argument("--episode-index", type=int, default=0)

    p.add_argument("--follower-left-port", default="/dev/ttyCH343USB0")
    p.add_argument("--follower-right-port", default=None)
    p.add_argument("--baudrate", type=int, default=1_000_000)
    p.add_argument("--timeout", type=float, default=0.05)
    p.add_argument("--debug-mode", action="store_true")

    p.add_argument("--arm-mode", choices=("both", "left", "right"), default="left")
    p.add_argument("--speed-scale", type=float, default=1.0)
    p.add_argument("--control-hz", type=float, default=30.0)
    p.add_argument("--dry-run", action="store_false")
    p.add_argument("--start-offset-s", type=float, default=0.0)
    p.add_argument("--max-steps", type=int, default=None)

    p.add_argument("--write-speed-deg-s", type=float, default=45.0)
    p.add_argument("--write-gripper-speed-deg-s", type=float, default=483.4)
    return p.parse_args()


def _available_episode_indices(dataset_dir: Path) -> list[int]:
    indices: list[int] = []
    for p in dataset_dir.glob("episode_*"):
        if not p.is_dir():
            continue
        suffix = p.name[len("episode_") :]
        if suffix.isdigit():
            indices.append(int(suffix))
    return sorted(indices)


def load_episode_actions_and_timestamps(dataset_dir: Path, episode_index: int) -> Tuple[np.ndarray, np.ndarray]:
    episode_dir = dataset_dir / f"episode_{episode_index:03d}"
    actions_path = episode_dir / "actions.npy"
    timestamps_path = episode_dir / "timestamps.npy"

    if not actions_path.exists():
        available = _available_episode_indices(dataset_dir)
        raise FileNotFoundError(
            f"Missing actions file: {actions_path}. "
            f"Available episode indices under {dataset_dir}: {available}"
        )

    actions = np.asarray(np.load(actions_path, allow_pickle=False), dtype=np.float32)
    if actions.ndim == 1:
        if actions.size in (7, 8, 14, 15):
            actions = actions.reshape(1, -1)
        else:
            raise ValueError(
                f"Expected actions.npy shape [T,7], [T,8], [T,14], or [T,15], got 1D shape {actions.shape} at {actions_path}"
            )
    if actions.ndim != 2 or actions.shape[1] not in (7, 8, 14, 15):
        raise ValueError(f"Expected actions.npy shape [T,7], [T,8], [T,14], or [T,15], got {actions.shape} at {actions_path}")
    # Strip trailing done flag if present (8 → 7, 15 → 14)
    if actions.shape[1] in (8, 15):
        actions = actions[:, :-1]
    if actions.shape[0] == 0:
        raise ValueError(f"Episode actions are empty: {actions_path}")

    if timestamps_path.exists():
        timestamps = np.asarray(np.load(timestamps_path, allow_pickle=False), dtype=np.float64).reshape(-1)
    else:
        timestamps = np.full((actions.shape[0],), np.nan, dtype=np.float64)

    if timestamps.shape[0] != actions.shape[0]:
        raise ValueError(
            f"timestamps/actions length mismatch for episode {episode_index}: "
            f"{timestamps.shape[0]} vs {actions.shape[0]}"
        )

    return actions, timestamps


def decode_action(action: np.ndarray, arm_mode: str) -> Dict[str, float]:
    action = np.asarray(action, dtype=np.float32).reshape(-1)
    num_values = action.shape[0]

    if num_values == 14:
        sides = ("left", "right")
    elif num_values == 7:
        if arm_mode == "both":
            raise ValueError("Single-arm actions (7D) cannot be replayed with --arm-mode both.")
        sides = (arm_mode,)
    else:
        raise ValueError(f"Expected 7 or 14 values, got {num_values}")

    out: Dict[str, float] = {}
    idx = 0
    for side in sides:
        for joint in JOINT_NAMES:
            out[f"{side}_{joint}.pos"] = float(action[idx])
            idx += 1
    return out


def has_valid_timestamp_deltas(timestamps: np.ndarray) -> bool:
    if timestamps.ndim != 1 or len(timestamps) < 2:
        return False
    if not np.all(np.isfinite(timestamps)):
        return False
    dts = np.diff(timestamps)
    return bool(np.all(dts > 0.0))


def compute_start_index(
    timestamps: np.ndarray,
    start_offset_s: float,
    control_hz: float,
) -> int:
    start_offset_s = max(0.0, float(start_offset_s))
    if has_valid_timestamp_deltas(timestamps):
        rel = timestamps - float(timestamps[0])
        return int(np.searchsorted(rel, start_offset_s, side="left"))

    if control_hz <= 0:
        raise ValueError("--control-hz must be > 0 when timestamp fallback is needed.")
    return int(start_offset_s * control_hz)


def replay_episode(
    actions: np.ndarray,
    timestamps: np.ndarray,
    follower: Optional[AliciaReplayFollower],
    arm_mode: str,
    speed_scale: float,
    control_hz: float,
    dry_run: bool,
    start_offset_s: float,
    max_steps: Optional[int],
) -> None:
    if speed_scale <= 0:
        raise ValueError("--speed-scale must be > 0.")
    if control_hz <= 0:
        raise ValueError("--control-hz must be > 0.")

    total_steps = actions.shape[0]
    if total_steps == 0:
        raise ValueError("Episode has no actions to replay.")

    start_idx = compute_start_index(timestamps=timestamps, start_offset_s=start_offset_s, control_hz=control_hz)
    start_idx = min(max(0, start_idx), total_steps)
    if start_idx >= total_steps:
        raise ValueError(
            f"start-offset-s={start_offset_s} skips all data (episode length={total_steps} steps)."
        )

    end_idx = total_steps
    if max_steps is not None:
        if max_steps <= 0:
            raise ValueError("--max-steps must be > 0 when provided.")
        end_idx = min(end_idx, start_idx + int(max_steps))

    use_timestamp_pacing = has_valid_timestamp_deltas(timestamps)
    fallback_dt = 1.0 / float(control_hz)

    print(
        f"Replay range: steps [{start_idx}, {end_idx}) of {total_steps} "
        f"(count={end_idx - start_idx}), arm_mode={arm_mode}, "
        f"timing={'timestamps' if use_timestamp_pacing else 'control_hz'}"
    )

    replay_start_t = time.perf_counter()
    for i in range(start_idx, end_idx):
        if i > start_idx:
            if use_timestamp_pacing:
                dt = float(timestamps[i] - timestamps[i - 1]) / speed_scale
            else:
                dt = fallback_dt / speed_scale
            if dt > 0:
                time.sleep(dt)

        action_dict = decode_action(actions[i], arm_mode=arm_mode)
        if True and follower is not None:
            follower.send_action(action_dict)

        local_idx = i - start_idx
        if local_idx < 5 or local_idx % 100 == 0 or i == end_idx - 1:
            left_g = action_dict.get("left_gripper.pos", float("nan"))
            right_g = action_dict.get("right_gripper.pos", float("nan"))
            print(
                f"step={i:05d} local={local_idx:05d} "
                f"left_g={left_g:7.2f} right_g={right_g:7.2f}"
            )

    elapsed = time.perf_counter() - replay_start_t
    print(f"Replay complete in {elapsed:.2f}s.")


def main() -> None:
    args = parse_args()
    dataset_dir = Path(args.dataset_root)
    if args.dataset_repo_id:
        dataset_dir = dataset_dir / args.dataset_repo_id

    actions, timestamps = load_episode_actions_and_timestamps(
        dataset_dir=dataset_dir,
        episode_index=int(args.episode_index),
    )
    print(
        f"Loaded episode {args.episode_index}: {actions.shape[0]} steps, "
        f"action_dim={actions.shape[1]} from {dataset_dir}"
    )

    follower: Optional[AliciaReplayFollower] = None
    if True:
        follower = AliciaReplayFollower(
            arm_mode=args.arm_mode,
            left_port=args.follower_left_port,
            right_port=args.follower_right_port,
            baudrate=int(args.baudrate),
            timeout=float(args.timeout),
            debug_mode=bool(args.debug_mode),
            write_speed_deg_s=float(args.write_speed_deg_s),
            write_gripper_speed_deg_s=float(args.write_gripper_speed_deg_s),
        )
        print("Connecting follower robot(s)...")
        follower.connect()

    try:
        replay_episode(
            actions=actions,
            timestamps=timestamps,
            follower=follower,
            arm_mode=args.arm_mode,
            speed_scale=float(args.speed_scale),
            control_hz=float(args.control_hz),
            dry_run=bool(args.dry_run),
            start_offset_s=float(args.start_offset_s),
            max_steps=args.max_steps,
        )
    finally:
        if follower is not None:
            print("Disconnecting follower robot(s)...")
            follower.disconnect()


if __name__ == "__main__":
    main()
