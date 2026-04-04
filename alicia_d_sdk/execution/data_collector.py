"""
Data Collection Module for VLA Training

Provides synchronized recording of robot state + camera images during
teleoperation (leader-follower or drag teaching). Saves episodes in a
flat structure that can be directly converted to LeRobot / HuggingFace
dataset format.

Episode directory layout::

    dataset_name/
      meta/
        info.json          # dataset-level metadata
        episodes.jsonl     # one JSON object per episode
        stats.json         # per-column normalization stats (populated by convert)
      data/
        episode_000000.parquet   # timestep-level tabular data
        episode_000001.parquet
        ...
      videos/
        observation.image/
          episode_000000.mp4
          episode_000001.mp4
          ...
"""

import os
import json
import time
import threading
import queue
from datetime import datetime
from typing import Optional, List, Dict, Any

import cv2
import numpy as np

from alicia_d_sdk.utils import precise_sleep
from alicia_d_sdk.utils.logger import logger


class CameraRecorder:
    """Threaded camera capture with frame queue."""

    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self._cap: Optional[cv2.VideoCapture] = None
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        self._cap = cv2.VideoCapture(self.camera_index)
        if not self._cap.isOpened():
            logger.error(f"Cannot open camera at index {self.camera_index}")
            return False
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        for _ in range(10):
            self._cap.read()
        logger.info(f"Camera {self.camera_index} opened ({self.width}x{self.height})")
        return True

    def start(self):
        """Start background capture thread that continuously grabs the latest frame."""
        self._running.set()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running.is_set():
            ret, frame = self._cap.read()
            if ret:
                frame = cv2.resize(frame, (self.width, self.height))
                with self._lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.001)

    def grab(self) -> Optional[np.ndarray]:
        """Return the most recent frame (BGR, uint8). Thread-safe."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
        return None

    def stop(self):
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
        logger.info("Camera stopped")


class DataCollector:
    """
    Collect demonstration episodes from a leader-follower or drag-teaching setup.

    Typical usage::

        collector = DataCollector(
            follower=follower_robot,
            leader=leader_robot,          # None for drag-teaching
            camera_indices=[0],
            dataset_dir="./my_dataset",
            fps=10,
        )
        collector.start()                 # interactive loop
    """

    def __init__(
        self,
        follower,
        leader=None,
        camera_indices: Optional[List[int]] = None,
        dataset_dir: str = "./vla_dataset",
        fps: float = 10.0,
        image_width: int = 640,
        image_height: int = 480,
    ):
        """
        :param follower: Follower SynriaRobotAPI (the arm executing actions)
        :param leader: Leader SynriaRobotAPI (teleoperation source). None = drag-teaching mode.
        :param camera_indices: List of camera device indices. Empty/None = no camera.
        :param dataset_dir: Root directory for the dataset.
        :param fps: Recording frequency in Hz.
        :param image_width: Captured image width in pixels.
        :param image_height: Captured image height in pixels.
        """
        self.follower = follower
        self.leader = leader
        self.fps = fps
        self.dataset_dir = dataset_dir
        self.image_width = image_width
        self.image_height = image_height

        self.cameras: List[CameraRecorder] = []
        for idx in (camera_indices or []):
            self.cameras.append(CameraRecorder(idx, image_width, image_height))

        self._data_dir = os.path.join(dataset_dir, "data")
        self._video_dir = os.path.join(dataset_dir, "videos", "observation.image")
        self._meta_dir = os.path.join(dataset_dir, "meta")
        os.makedirs(self._data_dir, exist_ok=True)
        os.makedirs(self._video_dir, exist_ok=True)
        os.makedirs(self._meta_dir, exist_ok=True)

        self._episode_index = self._next_episode_index()
        self._recording = threading.Event()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Open cameras and enter the interactive collection loop."""
        for cam in self.cameras:
            if not cam.open():
                raise RuntimeError(f"Failed to open camera {cam.camera_index}")
            cam.start()

        try:
            self._interactive_loop()
        except KeyboardInterrupt:
            print("\nCollection interrupted.")
        finally:
            for cam in self.cameras:
                cam.stop()
            self._write_info_json()
            print(f"\nDataset saved to {self.dataset_dir}  ({self._episode_index} episodes)")

    # ------------------------------------------------------------------
    # Interactive loop
    # ------------------------------------------------------------------

    def _interactive_loop(self):
        mode = "leader-follower" if self.leader else "drag-teaching"
        print(f"\n{'='*60}")
        print(f"  VLA Data Collection  ({mode})")
        print(f"  Dataset : {self.dataset_dir}")
        print(f"  FPS     : {self.fps}")
        print(f"  Cameras : {[c.camera_index for c in self.cameras]}")
        print(f"{'='*60}\n")

        while True:
            ep = self._episode_index
            print(f"--- Episode {ep} ---")
            task = input("Task instruction (or 'q' to quit): ").strip()
            if task.lower() in ("q", "quit", "exit"):
                break
            if not task:
                print("Task instruction cannot be empty.")
                continue

            if self.leader is None:
                print("Disabling follower torque for drag teaching...")
                self.follower.torque_control("off")
                input("Drag the arm to the start pose, then press Enter.")

            input("Press Enter to START recording...")

            episode_data = self._record_episode(task)

            if self.leader is None:
                self.follower.torque_control("on")
                print("Torque re-enabled.")

            if episode_data is None or len(episode_data["timestamps"]) < 2:
                print("Episode too short — discarded.\n")
                continue

            keep = input(f"Recorded {len(episode_data['timestamps'])} steps. Save? (y/n): ").strip().lower()
            if keep != "y":
                print("Episode discarded.\n")
                continue

            self._save_episode(episode_data, task)
            print(f"Episode {ep} saved.\n")
            self._episode_index += 1

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def _record_episode(self, task: str) -> Optional[Dict[str, Any]]:
        """Record a single episode until the user presses Enter."""
        timestamps: List[float] = []
        states: List[List[float]] = []
        actions: List[List[float]] = []
        grippers_state: List[float] = []
        grippers_action: List[float] = []
        frames: List[List[np.ndarray]] = []

        interval = 1.0 / self.fps
        spin_threshold = 0.002 if interval <= 0.010 else 0.005

        stop_event = threading.Event()

        def _wait_for_enter():
            input()
            stop_event.set()

        waiter = threading.Thread(target=_wait_for_enter, daemon=True)
        waiter.start()

        print("Recording... press Enter to STOP.")
        t0 = time.time()

        while not stop_event.is_set():
            loop_start = time.perf_counter()

            follower_state = self.follower.get_robot_state("joint_gripper")
            if follower_state is None:
                precise_sleep(interval - (time.perf_counter() - loop_start), spin_threshold=spin_threshold)
                continue

            obs_joints = list(follower_state.angles)
            obs_gripper = follower_state.gripper

            if self.leader is not None:
                leader_state = self.leader.get_robot_state("joint_gripper")
                if leader_state is None:
                    precise_sleep(interval - (time.perf_counter() - loop_start), spin_threshold=spin_threshold)
                    continue
                act_joints = list(leader_state.angles)
                act_gripper = leader_state.gripper

                self.follower.set_robot_state(
                    target_joints=act_joints,
                    gripper_value=int(act_gripper),
                    joint_format="rad",
                    speed_deg_s=120,
                    wait_for_completion=False,
                )
            else:
                act_joints = obs_joints
                act_gripper = obs_gripper

            cam_frames = []
            for cam in self.cameras:
                frame = cam.grab()
                if frame is not None:
                    cam_frames.append(frame)

            timestamps.append(time.time() - t0)
            states.append(obs_joints)
            grippers_state.append(obs_gripper)
            actions.append(act_joints)
            grippers_action.append(act_gripper)
            frames.append(cam_frames)

            precise_sleep(interval - (time.perf_counter() - loop_start), spin_threshold=spin_threshold)

        actual_fps = len(timestamps) / (timestamps[-1] if timestamps and timestamps[-1] > 0 else 1.0)
        print(f"Stopped. {len(timestamps)} steps, ~{actual_fps:.1f} FPS")

        return {
            "timestamps": timestamps,
            "states": states,
            "grippers_state": grippers_state,
            "actions": actions,
            "grippers_action": grippers_action,
            "frames": frames,
        }

    # ------------------------------------------------------------------
    # Saving
    # ------------------------------------------------------------------

    def _save_episode(self, episode_data: Dict[str, Any], task: str):
        ep_idx = self._episode_index
        ep_tag = f"episode_{ep_idx:06d}"

        timestamps = episode_data["timestamps"]
        states = np.array(episode_data["states"], dtype=np.float32)
        grippers_state = np.array(episode_data["grippers_state"], dtype=np.float32)
        actions = np.array(episode_data["actions"], dtype=np.float32)
        grippers_action = np.array(episode_data["grippers_action"], dtype=np.float32)
        frames = episode_data["frames"]
        n_steps = len(timestamps)

        # --- Parquet ---
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq

            state_cols = {f"observation.state.joint_{i}": states[:, i] for i in range(states.shape[1])}
            state_cols["observation.state.gripper"] = grippers_state

            action_cols = {f"action.joint_{i}": actions[:, i] for i in range(actions.shape[1])}
            action_cols["action.gripper"] = grippers_action

            table_dict = {
                "timestamp": np.array(timestamps, dtype=np.float64),
                "episode_index": np.full(n_steps, ep_idx, dtype=np.int64),
                "frame_index": np.arange(n_steps, dtype=np.int64),
                "task": [task] * n_steps,
            }
            table_dict.update(state_cols)
            table_dict.update(action_cols)

            if self.cameras:
                table_dict["observation.image.frame_index"] = np.arange(n_steps, dtype=np.int64)

            table = pa.table(table_dict)
            pq.write_table(table, os.path.join(self._data_dir, f"{ep_tag}.parquet"))

        except ImportError:
            logger.warning("pyarrow not installed — falling back to JSON")
            json_path = os.path.join(self._data_dir, f"{ep_tag}.json")
            rows = []
            for i in range(n_steps):
                rows.append({
                    "timestamp": timestamps[i],
                    "episode_index": ep_idx,
                    "frame_index": i,
                    "task": task,
                    "observation.state": states[i].tolist() + [float(grippers_state[i])],
                    "action": actions[i].tolist() + [float(grippers_action[i])],
                })
            with open(json_path, "w") as f:
                json.dump(rows, f)

        # --- Video ---
        if self.cameras and frames and frames[0]:
            video_path = os.path.join(self._video_dir, f"{ep_tag}.mp4")
            h, w = frames[0][0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(video_path, fourcc, self.fps, (w, h))
            for step_frames in frames:
                if step_frames:
                    writer.write(step_frames[0])
            writer.release()

        # --- Episode metadata ---
        ep_meta = {
            "episode_index": ep_idx,
            "task": task,
            "length": n_steps,
            "fps": self.fps,
            "timestamp": datetime.now().isoformat(),
        }
        ep_file = os.path.join(self._meta_dir, "episodes.jsonl")
        with open(ep_file, "a") as f:
            f.write(json.dumps(ep_meta) + "\n")

    def _write_info_json(self):
        """Write / overwrite the dataset-level info.json."""
        episodes_file = os.path.join(self._meta_dir, "episodes.jsonl")
        total_episodes = 0
        total_frames = 0
        tasks = set()
        if os.path.exists(episodes_file):
            with open(episodes_file) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    ep = json.loads(line)
                    total_episodes += 1
                    total_frames += ep.get("length", 0)
                    tasks.add(ep.get("task", ""))

        info = {
            "robot_type": "alicia_d_6dof",
            "total_episodes": total_episodes,
            "total_frames": total_frames,
            "fps": self.fps,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "tasks": sorted(tasks),
            "observation_space": {
                "state": {
                    "joint_angles": {"shape": [6], "dtype": "float32", "unit": "rad"},
                    "gripper": {"shape": [1], "dtype": "float32", "range": [0, 1000]},
                },
                "image": {
                    "shape": [self.image_height, self.image_width, 3],
                    "dtype": "uint8",
                    "encoding": "BGR",
                },
            },
            "action_space": {
                "joint_angles": {"shape": [6], "dtype": "float32", "unit": "rad"},
                "gripper": {"shape": [1], "dtype": "float32", "range": [0, 1000]},
            },
            "mode": "leader-follower" if self.leader else "drag-teaching",
            "created_at": datetime.now().isoformat(),
        }
        with open(os.path.join(self._meta_dir, "info.json"), "w") as f:
            json.dump(info, f, indent=2)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _next_episode_index(self) -> int:
        """Scan existing episodes to find the next index."""
        episodes_file = os.path.join(self._meta_dir, "episodes.jsonl")
        if not os.path.exists(episodes_file):
            return 0
        max_idx = -1
        with open(episodes_file) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                ep = json.loads(line)
                max_idx = max(max_idx, ep.get("episode_index", -1))
        return max_idx + 1
