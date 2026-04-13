#!/usr/bin/env python3
"""
Record bimanual teleoperation episodes using Alicia-D stream compatibility mode.

This script replicates the core behavior of:
  lerobot-record --robot.type=bi_alicia_d_follower --teleop.type=bi_alicia_d_leader ...

Dataset layout:
    dataset_root/
        episode_000/
            camera1/
            camera2/
            camera3/
            actions.npy      # leader state, shape [T, 14], deg + gripper(0-1000)
            states.npy       # follower state, shape [T, 14], deg + gripper(0-1000)
            timestamps.npy   # epoch timestamps (seconds), shape [T]
"""

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]  # .../Alicia_anudeep
sys.path.insert(0, str(REPO_ROOT))

import alicia_d_sdk
from alicia_d_sdk import create_robot

print("using:", alicia_d_sdk.__file__)


JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]


@dataclass
class CameraConfig:
    name: str
    index_or_path: str
    width: int = 640
    height: int = 480
    fps: int = 30


class OpenCVCamera:
    def __init__(self, config: CameraConfig):
        self.config = config
        self._cap = None  # type: Optional[cv2.VideoCapture]

    def connect(self) -> None:
        self._cap = cv2.VideoCapture(int(self.config.index_or_path))
        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.config.name}: {self.config.index_or_path}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.config.fps)

    def read(self) -> np.ndarray:
        if self._cap is None:
            raise RuntimeError(f"Camera {self.config.name} is not connected.")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"Failed to read frame from camera {self.config.name}")
        return frame

    def disconnect(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class BiAliciaDLeader:
    """Leader interface compatible with bi_alicia_d_leader behavior."""

    def __init__(
        self,
        left_port: str,
        right_port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        debug_mode: bool = False,
    ):
        # self.left_arm = create_stream_compat_robot(
        #     port=left_port,
        #     baudrate=baudrate,
        #     timeout=timeout,
        #     debug_mode=debug_mode,
        #     auto_connect=False,
        # )
        self.left_arm = create_robot(
            port=left_port,
            debug_mode=debug_mode
        )
        # self.right_arm = create_stream_compat_robot(
        #     port=right_port,
        #     baudrate=baudrate,
        #     timeout=timeout,
        #     debug_mode=debug_mode,
        #     auto_connect=False,
        # )

    def connect(self) -> None:
        if not self.left_arm.connect():
            raise RuntimeError("Failed to connect left leader arm")
        # if not self.right_arm.connect():
        #     raise RuntimeError("Failed to connect right leader arm")

    def disconnect(self) -> None:
        self.left_arm.disconnect()
        # self.right_arm.disconnect()

    def _read_arm(self, arm, side: str) -> Dict[str, float]:
        state = arm.get_robot_state("joint_gripper")
        if state is None:
            raise RuntimeError(f"Failed to read {side} leader state")
        joints_deg = [float(v) * 180.0 / math.pi for v in state.angles]
        out = {f"{side}_{name}.pos": joints_deg[i] for i, name in enumerate(JOINT_NAMES[:-1])}
        out[f"{side}_gripper.pos"] = float(state.gripper)
        return out

    def get_action(self) -> Dict[str, float]:
        action: Dict[str, float] = {}
        action.update(self._read_arm(self.left_arm, "left"))
        # action.update(self._read_arm(self.right_arm, "right"))
        return action
    
    def get_observation(self) -> Dict[str, float]:
        obs: Dict[str, float] = {}
        obs.update(self._read_arm(self.left_arm, "left"))
        # obs.update(self._read_arm(self.right_arm, "right"))
        return obs


class BiAliciaDFollower:
    """Follower interface compatible with bi_alicia_d_follower behavior."""

    def __init__(
        self,
        left_port: str,
        right_port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        debug_mode: bool = False,
        write_speed_deg_s: float = 45.0,
        write_gripper_speed_deg_s: float = 483.4,
    ):
        # self.left_arm = create_stream_compat_robot(
        #     port=left_port,
        #     baudrate=baudrate,
        #     timeout=timeout,
        #     debug_mode=debug_mode,
        #     auto_connect=False,
        # )
        # self.right_arm = create_stream_compat_robot(
        #     port=right_port,
        #     baudrate=baudrate,
        #     timeout=timeout,
        #     debug_mode=debug_mode,
        #     auto_connect=False,
        # )
        # self.left_arm = create_robot(
        #     port=self.left_port,
        #     debug_mode=debug_mode
        # )
        self.write_speed_deg_s = write_speed_deg_s
        self.write_gripper_speed_deg_s = write_gripper_speed_deg_s

    def connect(self) -> None:
        if not self.left_arm.connect():
            raise RuntimeError("Failed to connect left follower arm")
        if not self.right_arm.connect():
            raise RuntimeError("Failed to connect right follower arm")

    def disconnect(self) -> None:
        self.left_arm.disconnect()
        self.right_arm.disconnect()

    def _read_arm(self, arm, side: str) -> Dict[str, float]:
        state = arm.get_robot_state("joint_gripper")
        if state is None:
            raise RuntimeError(f"Failed to read {side} follower state")
        joints_deg = [float(v) * 180.0 / math.pi for v in state.angles]
        out = {f"{side}_{name}.pos": joints_deg[i] for i, name in enumerate(JOINT_NAMES[:-1])}
        out[f"{side}_gripper.pos"] = float(state.gripper)
        return out

    def get_observation(self) -> Dict[str, float]:
        obs: Dict[str, float] = {}
        obs.update(self._read_arm(self.left_arm, "left"))
        obs.update(self._read_arm(self.right_arm, "right"))
        return obs

    def _write_arm(self, arm, action: Dict[str, float], side: str) -> bool:
        joints_deg = [float(action[f"{side}_joint_{i}.pos"]) for i in range(1, 7)]
        gripper = float(action[f"{side}_gripper.pos"])
        return arm.set_robot_state(
            target_joints=joints_deg,
            gripper_value=gripper,
            joint_format="deg",
            speed_deg_s=self.write_speed_deg_s,
            gripper_speed_deg_s=self.write_gripper_speed_deg_s,
            wait_for_completion=False,
        )

    def send_action(self, action: Dict[str, float]) -> None:
        ok_left = self._write_arm(self.left_arm, action, "left")
        ok_right = self._write_arm(self.right_arm, action, "right")
        if not (ok_left and ok_right):
            raise RuntimeError("Failed to command one or both follower arms")


def flatten_bimanual_state(state: Dict[str, float]) -> np.ndarray:
    ordered: List[float] = []
    # for side in ("left", "right"):
    for side in ("left",):
        for joint in JOINT_NAMES:
            ordered.append(float(state[f"{side}_{joint}.pos"]))
    return np.asarray(ordered, dtype=np.float32)


def make_episode_dir(dataset_root: Path, episode_idx: int, camera_names: List[str]) -> Path:
    episode_dir = dataset_root / f"episode_{episode_idx:03d}"
    episode_dir.mkdir(parents=True, exist_ok=True)
    for cam_name in camera_names:
        (episode_dir / cam_name).mkdir(parents=True, exist_ok=True)
    return episode_dir


def next_episode_index(dataset_root: Path) -> int:
    max_idx = -1
    for p in dataset_root.glob("episode_*"):
        if not p.is_dir():
            continue
        suffix = p.name[len("episode_") :]
        if suffix.isdigit():
            max_idx = max(max_idx, int(suffix))
    return max_idx + 1


def count_episode_dirs(dataset_root: Path) -> int:
    count = 0
    for p in dataset_root.glob("episode_*"):
        if not p.is_dir():
            continue
        suffix = p.name[len("episode_") :]
        if suffix.isdigit():
            count += 1
    return count


def show_frames(frames: Dict[str, np.ndarray]) -> None:
    names = sorted(frames.keys())
    resized = []
    target_h = 360
    for name in names:
        frame = frames[name]
        h, w = frame.shape[:2]
        scale = target_h / float(h)
        target_w = int(w * scale)
        img = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        cv2.putText(
            img,
            name,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        resized.append(img)
    canvas = cv2.hconcat(resized)
    cv2.imshow("bimanual_recording", canvas)
    cv2.waitKey(1)


def countdown(seconds: float, message: str) -> None:
    if seconds <= 0:
        return
    end_t = time.time() + seconds
    while True:
        remain = end_t - time.time()
        if remain <= 0:
            print(f"\r{message}: done{' ' * 20}")
            break
        print(f"\r{message}: {remain:6.1f}s", end="", flush=True)
        time.sleep(0.1)


def run_teleop_phase(
    leader: BiAliciaDLeader,
    follower: BiAliciaDFollower,
    cameras: Dict[str, OpenCVCamera],
    duration_s: float,
    dt: float,
    display_data: bool,
    episode_dir: Optional[Path] = None,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[float]]:
    """
    Run teleoperation loop for a fixed duration.

    If episode_dir is provided, this function records frames + arrays to disk buffers.
    If episode_dir is None, it only teleoperates (reset behavior like lerobot-record).
    """
    actions: List[np.ndarray] = []
    states: List[np.ndarray] = []
    timestamps: List[float] = []

    start_t = time.time()
    frame_idx = 0

    while (time.time() - start_t) < duration_s:
        step_start = time.perf_counter()
        timestamp = time.time()

        action = leader.get_action()
        # follower.send_action(action)
        # state = follower.get_observation()
        state = leader.get_observation()

        frames: Dict[str, np.ndarray] = {}
        for cam_name, cam in cameras.items():
            frame = cam.read()
            frames[cam_name] = frame

            if episode_dir is not None:
                out_path = episode_dir / cam_name / f"frame_{frame_idx:06d}.jpg"
                cv2.imwrite(str(out_path), frame)

        # if display_data:
        #     show_frames(frames)

        if episode_dir is not None:
            actions.append(flatten_bimanual_state(action))
            states.append(flatten_bimanual_state(state))
            timestamps.append(timestamp)
            frame_idx += 1

        elapsed = time.perf_counter() - step_start
        sleep_s = dt - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    return actions, states, timestamps


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record bimanual Alicia-D stream-compat episodes.")
    parser.add_argument("--follower-left-port", default="/dev/cu.usbmodem5B140413001")
    parser.add_argument("--follower-right-port", default=None)
    parser.add_argument("--leader-left-port", default="/dev/cu.usbmodem5AE60936831")
    parser.add_argument("--leader-right-port", default=None)
    parser.add_argument("--teleop-left-port", default="/dev/cu.usbmodem5AE60936831")
    parser.add_argument("--teleop-right-port", default=None)

    parser.add_argument("--camera1", default="0")
    parser.add_argument("--camera2", default="1")
    parser.add_argument("--camera3", default="/dev/video4")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)

    parser.add_argument("--dataset-root", default="/tmp/lerobot_mock")
    parser.add_argument("--dataset-repo-id", default="ubuntu/bimanual-dataset")
    parser.add_argument("--dataset-num-episodes", type=int, default=50)
    parser.add_argument("--dataset-episode-time-s", type=float, default=15.0)
    parser.add_argument("--dataset-reset-time-s", type=float, default=5.0)
    parser.add_argument("--dataset-single-task", default="Grab the screwdriver")
    parser.add_argument("--dataset-push-to-hub", action="store_true")

    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--display-data", action="store_true")
    parser.add_argument("--debug-mode", action="store_true")
    parser.add_argument(
        "--append",
        action="store_true",
        default=True,
        help="Append new episodes after the highest existing episode_NNN directory.",
    )
    return parser.parse_args()


def write_metadata(dataset_root: Path, args: argparse.Namespace, camera_names: List[str]) -> None:
    metadata_path = dataset_root / "metadata.json"
    episode_count = count_episode_dirs(dataset_root)

    if metadata_path.exists():
        with metadata_path.open("r", encoding="utf-8") as f:
            metadata = json.load(f)
        if not isinstance(metadata, dict):
            raise ValueError(f"Expected JSON object in {metadata_path}, got {type(metadata).__name__}")
        metadata["num_episodes"] = int(episode_count)
    else:
        metadata = {
            "robot_type": "bi_alicia_d_follower",
            "teleop_type": "bi_alicia_d_leader",
            "dataset_repo_id": args.dataset_repo_id,
            "dataset_push_to_hub": bool(args.dataset_push_to_hub),
            "num_episodes": int(episode_count),
            "episode_time_s": float(args.dataset_episode_time_s),
            "reset_time_s": float(args.dataset_reset_time_s),
            "task_description": args.dataset_single_task,
            "control_hz": float(args.control_hz),
            "cameras": {
                name: {
                    "type": "opencv",
                    "index_or_path": getattr(args, name),
                    "width": int(args.camera_width),
                    "height": int(args.camera_height),
                    "fps": int(args.camera_fps),
                }
                for name in camera_names
            },
            "created_unix_time": time.time(),
        }

    with metadata_path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)


def main() -> None:
    args = parse_args()
    leader_left_port = args.leader_left_port or args.teleop_left_port
    leader_right_port = args.leader_right_port or args.teleop_right_port
    # if not leader_left_port or not leader_right_port:
    #     raise ValueError(
    #         "Leader ports are required. Provide --leader-left-port/--leader-right-port "
    #         "or --teleop-left-port/--teleop-right-port."
    #     )

    if not leader_left_port:
        raise ValueError(
            "Leader ports are required. Provide --leader-left-port/--leader-right-port "
            "or --teleop-left-port/--teleop-right-port."
        )

    dataset_root = Path(args.dataset_root)
    dataset_root.mkdir(parents=True, exist_ok=True)
    start_episode_idx = next_episode_index(dataset_root) if args.append else 0

    camera_configs = [
        CameraConfig("camera1", args.camera1, args.camera_width, args.camera_height, args.camera_fps),
        CameraConfig("camera2", args.camera2, args.camera_width, args.camera_height, args.camera_fps),
        # CameraConfig("camera3", args.camera3, args.camera_width, args.camera_height, args.camera_fps),
    ]
    cameras = {cfg.name: OpenCVCamera(cfg) for cfg in camera_configs}
    camera_names = [cfg.name for cfg in camera_configs]

    leader = BiAliciaDLeader(
        left_port=leader_left_port,
        right_port=leader_right_port,
        debug_mode=args.debug_mode,
    )
    follower = BiAliciaDFollower(
        left_port=args.follower_left_port,
        right_port=args.follower_right_port,
        debug_mode=args.debug_mode,
    )

    print("Connecting leader/follower arms and cameras...")
    for cam in cameras.values():
        cam.connect()
    leader.connect()
    # follower.connect()
    write_metadata(dataset_root, args, camera_names)
    print("Connected.")
    if args.append:
        print(f"Append mode enabled: starting from episode_{start_episode_idx:03d}")

    dt = 1.0 / float(args.control_hz)

    try:
        for episode_offset in range(args.dataset_num_episodes):
            episode_idx = start_episode_idx + episode_offset
            print(f"\n=== Episode {episode_idx:03d} (new {episode_offset + 1}/{args.dataset_num_episodes}) ===")
            print(f"Task: {args.dataset_single_task}")
            print("Reset phase: teleop active (not recorded)")
            run_teleop_phase(
                leader=leader,
                follower=follower,
                cameras=cameras,
                duration_s=args.dataset_reset_time_s,
                dt=dt,
                display_data=args.display_data,
                episode_dir=None,
            )
            print("Teleop phase: recording started")

            episode_dir = make_episode_dir(dataset_root, episode_idx, camera_names)
            actions, states, timestamps = run_teleop_phase(
                leader=leader,
                follower=follower,
                cameras=cameras,
                duration_s=args.dataset_episode_time_s,
                dt=dt,
                display_data=args.display_data,
                episode_dir=episode_dir,
            )

            actions_arr = np.asarray(actions, dtype=np.float32)
            done_col = np.zeros((len(actions_arr), 1), dtype=np.float32)
            done_col[-1, 0] = 1.0
            actions_arr = np.concatenate([actions_arr, done_col], axis=1)
            np.save(episode_dir / "actions.npy", actions_arr)
            np.save(episode_dir / "states.npy", np.asarray(states, dtype=np.float32))
            np.save(episode_dir / "timestamps.npy", np.asarray(timestamps, dtype=np.float64))
            write_metadata(dataset_root, args, camera_names)
            print(f"Saved {len(timestamps)} steps to {episode_dir}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Shutting down devices...")
        if args.display_data:
            cv2.destroyAllWindows()
        for cam in cameras.values():
            cam.disconnect()
        leader.disconnect()
        # follower.disconnect()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()


