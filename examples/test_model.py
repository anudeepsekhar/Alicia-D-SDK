#!/usr/bin/env python3
"""
Mock version of 19_run_alicia_inference.py — no real hardware required.

Uses V4L2 cameras (default ``/dev/video*`` on Linux) and a simulated follower
robot so the full inference pipeline can be tested end-to-end without physical
Alicia-D arms. Observations are assembled in the same shape as
``NpyEpisodeDataset`` + ``default_collate``, then run through ``preprocessor``,
``policy.forward`` (as in ``lerobot_infer_npy.evaluate_on_test_episodes``),
and ``policy.select_action`` + ``postprocessor`` to command the mock follower.

Usage:

    python examples/20_run_alicia_inference_mock.py \\
        --checkpoint outputs/train/my_run/checkpoints/050000/pretrained_model \\
        --dataset-root /path/to/lerobot_data
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import torch
from torch.utils.data import default_collate

# try:
#     import cv2
# except Exception:
#     cv2 = None
import cv2
import alicia_d_sdk
from alicia_d_sdk import create_robot

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]

# #region agent log
_AGENT_DEBUG_LOG_PATH = "/home/rnarasim/Alicia-D-SDK/.cursor/debug-91f161.log"


def _agent_debug_log(
    hypothesis_id: str,
    location: str,
    message: str,
    data: dict,
    run_id: str = "pre-fix",
) -> None:
    try:
        with open(_AGENT_DEBUG_LOG_PATH, "a", encoding="utf-8") as _f:
            _f.write(
                json.dumps(
                    {
                        "sessionId": "91f161",
                        "timestamp": int(time.time() * 1000),
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "runId": run_id,
                    },
                    default=str,
                )
                + "\n"
            )
    except Exception:
        pass


def _agent_log_opencv_environment() -> None:
    import importlib.metadata as imm

    dists = {}
    for name in ("opencv-python", "opencv-python-headless"):
        try:
            dists[name] = imm.version(name)
        except imm.PackageNotFoundError:
            dists[name] = None
    gui_lines: List[str] = []
    try:
        bi = cv2.getBuildInformation()
        gui_lines = [
            ln.strip()
            for ln in bi.splitlines()
            if any(k in ln.upper() for k in ("GTK", "GUI:", "HIGHGUI", "QT:", "WIN32UI", "FFMPEG"))
        ][:20]
    except Exception as e:
        gui_lines = [f"getBuildInformation_error: {e!r}"]
    _agent_debug_log(
        "A",
        "test_model.py:_agent_log_opencv_environment",
        "cv2 module and pip distributions",
        {
            "cv2_file": getattr(cv2, "__file__", None),
            "cv2_version": getattr(cv2, "__version__", None),
            "distributions": dists,
        },
    )
    _agent_debug_log(
        "B",
        "test_model.py:_agent_log_opencv_environment",
        "display-related env",
        {
            "DISPLAY": os.environ.get("DISPLAY"),
            "WAYLAND_DISPLAY": os.environ.get("WAYLAND_DISPLAY"),
            "SSH_CONNECTION": os.environ.get("SSH_CONNECTION"),
        },
    )
    _agent_debug_log(
        "D",
        "test_model.py:_agent_log_opencv_environment",
        "OpenCV build lines (GUI stack)",
        {"gui_related_lines": gui_lines},
    )


# #endregion


def _gui_preview_available() -> tuple[bool, str]:
    """Return (True, '') if ``cv2.imshow`` should work; else (False, user-facing reason)."""
    if cv2 is None:
        return False, "OpenCV is not available."
    try:
        bi = cv2.getBuildInformation()
    except Exception as e:
        return False, f"Could not read OpenCV build info: {e}"
    for ln in bi.splitlines():
        if "GUI:" in ln and "NONE" in ln.upper():
            return (
                False,
                "This OpenCV build has no GUI backend (common with opencv-python-headless). "
                "Try: uv pip uninstall opencv-python-headless && uv pip install opencv-python",
            )
    if sys.platform.startswith("linux"):
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            return (
                False,
                "No DISPLAY or WAYLAND_DISPLAY (typical over SSH). "
                "Use --no-display, or enable X11 forwarding / run on a desktop session.",
            )
    return True, ""


def _dev_video_number(path: str) -> Optional[int]:
    if not path.startswith("/dev/video"):
        return None
    tail = path[len("/dev/video") :]
    return int(tail) if tail.isdigit() else None


def _linux_video_device_path(index_or_path: str) -> Optional[str]:
    """Path under /dev/video* for permission checks, if known."""
    if index_or_path.startswith("/dev/video"):
        return index_or_path
    if index_or_path.isdigit():
        return f"/dev/video{int(index_or_path)}"
    return None


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


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
        raw = self.config.index_or_path
        linux = sys.platform.startswith("linux")
        if linux:
            dev = _linux_video_device_path(raw)
            if dev is not None and Path(dev).exists():
                if not os.access(dev, os.R_OK | os.W_OK):
                    raise RuntimeError(
                        f"Permission denied for {dev} (camera {self.config.name}). "
                        "Add your user to the 'video' group: "
                        "sudo usermod -aG video $USER — then log out and back in (or reboot). "
                        "If you run from Cursor/VS Code Remote, restart the remote server so "
                        "the new group is applied."
                    )

        tries: List[tuple[Any, int]] = []

        if raw.isdigit():
            n = int(raw)
            if linux:
                tries.append((n, cv2.CAP_V4L2))
            tries.append((n, cv2.CAP_ANY))
        else:
            if linux:
                tries.append((raw, cv2.CAP_V4L2))
            tries.append((raw, cv2.CAP_ANY))
            dn = _dev_video_number(raw)
            if dn is not None:
                if linux:
                    tries.append((dn, cv2.CAP_V4L2))
                tries.append((dn, cv2.CAP_ANY))

        self._cap = None
        for arg, api in tries:
            cap = cv2.VideoCapture(arg, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                self._cap = cap
                break
            if cap is not None:
                cap.release()

        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(
                f"Failed to open camera {self.config.name}: {self.config.index_or_path}. "
                "On Linux, check 'video' group membership, that no other process holds the device, "
                "and try an alternate /dev/video* node if the driver exposes several."
            )

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



# ---------------------------------------------------------------------------
# Robot helpers
# ---------------------------------------------------------------------------

class AliciaFollower:
    """Thin wrapper around one or two stream-compat follower arms."""

    def __init__(
        self,
        arm_mode: str,
        left_port: Optional[str],
        right_port: Optional[str],
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        debug_mode: bool = False,
        write_speed_deg_s: float = 45.0,
        write_gripper_speed_deg_s: float = 483.4,
    ):
        self.arm_mode = arm_mode
        self.write_speed_deg_s = write_speed_deg_s
        self.write_gripper_speed_deg_s = write_gripper_speed_deg_s

        need_left = arm_mode in ("both", "left")
        need_right = arm_mode in ("both", "right")

        self.left_arm = None
        self.right_arm = None

        if need_left:
            # if not left_port:
            #     raise ValueError("--follower-left-port is required for arm_mode 'both' or 'left'.")
            # self.left_arm = create_robot(
            #     port=left_port,
            #     debug_mode=debug_mode
            # )
            self.left_arm = create_robot()
        if need_right:
            # if not right_port:
            #     raise ValueError("--follower-right-port is required for arm_mode 'both' or 'right'.")
            # self.right_arm = create_robot(
            #     port=right_port, baudrate=baudrate, timeout=timeout,
            #     debug_mode=debug_mode, auto_connect=False,
            # )
            # self.right_arm = create_robot(
            #     port=right_port,
            #     debug_mode=debug_mode
            # )
            self.right_arm = create_robot()

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
        if self.left_arm is not None:
            obs.update(self._read_arm(self.left_arm, "left"))
        if self.right_arm is not None:
            obs.update(self._read_arm(self.right_arm, "right"))
        return obs

    def _write_arm(self, arm, joints_deg: List[float], gripper: float) -> bool:
        return arm.set_robot_state(
            target_joints=joints_deg,
            gripper_value=gripper,
            joint_format="deg",
            speed_deg_s=self.write_speed_deg_s,
            gripper_speed_deg_s=self.write_gripper_speed_deg_s,
            wait_for_completion=False,
        )


    def send_action(self, action: np.ndarray) -> None:
        """Send a flat action vector (7 or 14 floats: 6 joints + gripper per arm)."""
        action = action.flatten()
        if self.left_arm is not None:
            offset = 0
            joints = action[offset : offset + 6].tolist()
            gripper = float(action[offset + 6])
            if not self._write_arm(self.left_arm, joints, gripper):
                raise RuntimeError("Failed to command left arm.")
        if self.right_arm is not None:
            offset = 7 if action.shape[0] == 14 else 0
            joints = action[offset : offset + 6].tolist()
            gripper = float(action[offset + 6])
            if not self._write_arm(self.right_arm, joints, gripper):
                raise RuntimeError("Failed to command right arm.")


def flatten_observation_state(obs_dict: Dict[str, float]) -> np.ndarray:
    """Flatten the robot joint dict into a float32 array matching dataset order."""
    ordered: List[float] = []
    sides = []
    if any(k.startswith("left_") for k in obs_dict):
        sides.append("left")
    if any(k.startswith("right_") for k in obs_dict):
        sides.append("right")
    for side in sides:
        for joint in JOINT_NAMES:
            ordered.append(float(obs_dict[f"{side}_{joint}.pos"]))
    return np.asarray(ordered, dtype=np.float32)

# ---------------------------------------------------------------------------
# Shared helpers (same as 19_run_alicia_inference.py)
# ---------------------------------------------------------------------------

def flatten_observation_state(obs_dict: Dict[str, float]) -> np.ndarray:
    ordered: List[float] = []
    sides = []
    if any(k.startswith("left_") for k in obs_dict):
        sides.append("left")
    if any(k.startswith("right_") for k in obs_dict):
        sides.append("right")
    for side in sides:
        for joint in JOINT_NAMES:
            ordered.append(float(obs_dict[f"{side}_{joint}.pos"]))
    return np.asarray(ordered, dtype=np.float32)


def load_policy(
    checkpoint: str,
    dataset_root: str,
    device: str,
    camera_height: int,
    camera_width: int,
):
    lerobot_root = REPO_ROOT.parent / "lerobot"
    lerobot_src = lerobot_root / "src"
    if lerobot_src.exists() and str(lerobot_src) not in sys.path:
        sys.path.insert(0, str(lerobot_src))

    from lerobot.scripts.lerobot_infer_npy import build_inference_pipeline

    return build_inference_pipeline(
        checkpoint=checkpoint,
        dataset_root=dataset_root,
        device=device,
        camera_height=camera_height,
        camera_width=camera_width,
    )


# ---------------------------------------------------------------------------
# Display helper
# ---------------------------------------------------------------------------

def show_camera_frames(frames: Dict[str, np.ndarray], task: str) -> int:
    if cv2 is None:
        return 0
    resized = []
    target_h = 360
    for name in sorted(frames.keys()):
        frame = frames[name]
        h, w = frame.shape[:2]
        scale = target_h / float(h)
        target_w = int(w * scale)
        img = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        cv2.putText(img, name, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                     0.7, (0, 255, 0), 2, cv2.LINE_AA)
        resized.append(img)

    canvas = cv2.hconcat(resized) if resized else np.zeros((target_h, 480, 3), dtype=np.uint8)
    cv2.putText(canvas, f"Task: {task}", (12, canvas.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Press 'q' when task is done", (12, canvas.shape[0] - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
    # #region agent log
    _agent_debug_log(
        "E",
        "test_model.py:show_camera_frames",
        "about to call cv2.imshow",
        {"task": task, "canvas_shape": list(canvas.shape)},
    )
    # #endregion
    try:
        cv2.imshow("Alicia Inference (mock)", canvas)
        return cv2.waitKey(1) & 0xFF
    except cv2.error as e:
        # #region agent log
        _agent_debug_log(
            "F",
            "test_model.py:show_camera_frames",
            "cv2.imshow/waitKey failed",
            {"error": str(e)},
            run_id="post-fix",
        )
        # #endregion
        return 0


# ---------------------------------------------------------------------------
# NpyEpisodeDataset-shaped batch (preprocessor input), matching lerobot_train_npy
# ---------------------------------------------------------------------------


def _dims_from_dataset_meta(dataset_meta) -> tuple[int, int]:
    st = dataset_meta.stats["observation.state"]["mean"]
    ac = dataset_meta.stats["action"]["mean"]
    return int(st.numel()), int(ac.numel())


def policy_image_camera_names(dataset_meta, policy) -> List[str]:
    """Basenames for policy cameras (e.g. ``camera1`` for ``observation.images.camera1``).

    ``build_inference_pipeline`` builds ``NpyEpisodeDataset(..., load_images=False)``,
    which sets ``dataset_meta.camera_keys`` to ``[]`` even though the policy and
    ``dataset_stats.json`` still use ``observation.images.*``. Without a fallback,
    the mock batch omits image tensors and ``*_is_pad`` keys that the preprocessor
    and ``policy.forward`` expect (matching a real DataLoader batch).
    """
    keys = list(dataset_meta.camera_keys)
    if keys:
        return [k.replace("observation.images.", "") for k in keys]
    prefix = "observation.images."
    names: List[str] = []
    for feat_key in policy.config.image_features:
        if feat_key.startswith(prefix):
            names.append(feat_key[len(prefix) :])
        else:
            names.append(feat_key)
    return names


def _len_delta(indices: Any) -> int:
    if indices is None:
        return 0
    return len(indices)


def _align_state_vec(state_vec: np.ndarray, state_dim: int) -> np.ndarray:
    v = np.asarray(state_vec, dtype=np.float32).reshape(-1)
    if v.size == state_dim:
        return v
    if v.size < state_dim:
        out = np.zeros(state_dim, dtype=np.float32)
        out[: v.size] = v
        return out
    return v[:state_dim].copy()


def _frame_rgb_uint8_to_chw_float(
    raw: np.ndarray,
    camera_height: int,
    camera_width: int,
) -> torch.Tensor:
    """Match NpyEpisodeDataset._load_image: RGB float CHW in [0, 1]."""
    if raw is None:
        return torch.zeros(3, camera_height, camera_width, dtype=torch.float32)
    if cv2 is not None:
        img = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (camera_width, camera_height), interpolation=cv2.INTER_AREA)
    else:
        img = raw[:camera_height, :camera_width, ::-1].copy()
    t = torch.from_numpy(img).float().permute(2, 0, 1) / 255.0
    return t


def build_npy_style_sample(
    *,
    state_vec: np.ndarray,
    frames_bgr_uint8: Dict[str, np.ndarray],
    camera_names_for_policy: List[str],
    camera_height: int,
    camera_width: int,
    task: str,
    step: int,
    control_hz: float,
    dataset_meta,
    policy,
) -> dict[str, Any]:
    """One sample dict as returned by NpyEpisodeDataset.__getitem__ (before collate)."""
    state_dim, action_dim = _dims_from_dataset_meta(dataset_meta)
    cfg = policy.config
    action_deltas = cfg.action_delta_indices
    obs_deltas = cfg.observation_delta_indices

    state_np = _align_state_vec(state_vec, state_dim)
    state_1 = torch.from_numpy(state_np).float()

    item: dict[str, Any] = {
        "timestamp": np.float32(step / max(control_hz, 1e-6)),
        "episode_index": 0,
        "frame_index": step,
        "index": step,
        "task": task,
    }

    if _len_delta(action_deltas) > 0:
        n_a = _len_delta(action_deltas)
        item["action"] = torch.zeros(n_a, action_dim, dtype=torch.float32)
        item["action_is_pad"] = torch.zeros(n_a, dtype=torch.bool)
    else:
        item["action"] = torch.zeros(action_dim, dtype=torch.float32)

    if _len_delta(obs_deltas) > 0:
        n_o = _len_delta(obs_deltas)
        item["observation.state"] = state_1.unsqueeze(0).expand(n_o, -1).clone()
        item["observation.state_is_pad"] = torch.zeros(n_o, dtype=torch.bool)
        for cam_name in camera_names_for_policy:
            raw = frames_bgr_uint8.get(cam_name)
            chw = _frame_rgb_uint8_to_chw_float(raw, camera_height, camera_width)
            key = f"observation.images.{cam_name}"
            item[key] = chw.unsqueeze(0).expand(n_o, -1, -1, -1).clone()
            item[f"{key}_is_pad"] = torch.zeros(n_o, dtype=torch.bool)
    else:
        item["observation.state"] = state_1
        for cam_name in camera_names_for_policy:
            raw = frames_bgr_uint8.get(cam_name)
            chw = _frame_rgb_uint8_to_chw_float(raw, camera_height, camera_width)
            item[f"observation.images.{cam_name}"] = chw

    return item


def collate_npy_batch(sample: dict[str, Any]) -> dict[str, Any]:
    """Batch size 1 via the same default_collate used by DataLoader."""
    return default_collate([sample])


# ---------------------------------------------------------------------------
# Main control loop for a single task
# ---------------------------------------------------------------------------

def run_task(
    task: str,
    policy,
    preprocessor,
    postprocessor,
    dataset_meta,
    follower: AliciaFollower,
    cameras: Dict[str, OpenCVCamera],
    camera_names_for_policy: List[str],
    camera_height: int,
    camera_width: int,
    device: str,
    control_hz: float,
    display: bool,
    log_forward_loss: bool = False,
) -> None:
    """Run the inference control loop until the user signals the task is done.

    Matches the validation path in lerobot_infer_npy.evaluate_on_test_episodes:
    NpyEpisodeDataset-shaped batch -> default_collate -> preprocessor -> policy.forward.
    The mock follower is driven via policy.select_action + postprocessor on the same
    preprocessed batch (forward alone does not emit actions for e.g. SmolVLA).
    """
    dt = 1.0 / control_hz
    torch_device = torch.device(device)
    use_amp = torch_device.type in ("cuda", "mps")
    amp_ctx = torch.autocast(device_type=torch_device.type) if use_amp else nullcontext()
    step = 0
    interrupted = False

    def _sigint_handler(sig, frame):
        nonlocal interrupted
        interrupted = True

    prev_handler = signal.signal(signal.SIGINT, _sigint_handler)

    policy.reset()
    policy.eval()

    try:
        while not interrupted:
            t0 = time.perf_counter()

            robot_obs = follower.get_observation()
            state_vec = flatten_observation_state(robot_obs)

            frames: Dict[str, np.ndarray] = {}
            for cam_name, cam in cameras.items():
                frames[cam_name] = cam.read()

            sample = build_npy_style_sample(
                state_vec=state_vec,
                frames_bgr_uint8=frames,
                camera_names_for_policy=camera_names_for_policy,
                camera_height=camera_height,
                camera_width=camera_width,
                task=task,
                step=step,
                control_hz=control_hz,
                dataset_meta=dataset_meta,
                policy=policy,
            )
            batch = collate_npy_batch(sample)
            batch = preprocessor(batch)

            with torch.inference_mode(), amp_ctx:
                loss, _output_dict = policy.forward(batch)
                action = policy.select_action(batch)
            action = postprocessor(action)

            action_np = action.cpu().numpy().astype(np.float32)
            follower.send_action(action_np)

            if step % 100 == 0:
                line = f"  [step {step}] action={action_np[:4]}..."
                if log_forward_loss:
                    line += f"  forward_loss={float(loss):.6f}"
                print(line)

            if display:
                key = show_camera_frames(frames, task)
                if key == ord("q"):
                    print("\nTask marked as done by user (pressed 'q').")
                    break

            step += 1

            elapsed = time.perf_counter() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        signal.signal(signal.SIGINT, prev_handler)
        if interrupted:
            print("\nTask interrupted (Ctrl-C).")

    print(f"Task '{task}' finished after {step} steps.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Mock Alicia-D inference — no hardware required.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # p.add_argument("--checkpoint", default="/home/rnarasim/lerobot/src/lerobot/scripts/outputs/train/2026-04-13/17-33-10_npy_pi05/checkpoints/050000/pretrained_model",
    #                 help="Path to pretrained_model directory (config.json + model.safetensors).")
    p.add_argument("--checkpoint", default="/home/rnarasim/lerobot/src/lerobot/scripts/outputs/train/2026-04-16/15-10-03_npy_pi05/checkpoints/050000/pretrained_model",
                    help="Path to pretrained_model directory (config.json + model.safetensors).")
    p.add_argument("--dataset-root", default="/home/rnarasim/lerobot_data",
                    help="Path to the npy dataset root (for normalization stats / metadata).")

    p.add_argument("--arm-mode", choices=("both", "left", "right"), default="left")
    p.add_argument("--num-cameras", type=int, default=2,
                    help="Number of cameras (/dev/video0, /dev/video1, ...).")
    p.add_argument("--camera-width", type=int, default=640)
    p.add_argument("--camera-height", type=int, default=480)
    p.add_argument("--camera-fps", type=int, default=30)
    p.add_argument("--seed", type=int, default=42)

    p.add_argument("--control-hz", type=float, default=30.0)
    p.add_argument("--device", default=(
        "mps" if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available()
        else "cuda" if torch.cuda.is_available()
        else "cpu"
    ))
    p.add_argument("--no-display", action="store_true",
                    help="Disable the OpenCV camera preview window.")
    p.add_argument(
        "--log-forward-loss",
        action="store_true",
        help="Every 100 steps, also print policy.forward training loss (vs synthetic action).",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    # #region agent log
    if cv2 is not None:
        _agent_log_opencv_environment()
    # #endregion

    # -- Mock cameras --------------------------------------------------------
    camera_configs: List[CameraConfig] = []
    i = 0
    max_camera_index = args.num_cameras
    while i < max_camera_index:
        name = f"camera{i + 1}"
        path = f"/dev/video{i}"
        if not __import__("os").path.exists(path):
            break
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                max_camera_index += 1
                i += 1
                print(f"Failed to open camera {path}")
                continue
        finally:
            cap.release()
        camera_configs.append(CameraConfig(
            name=name,
            index_or_path=f"/dev/video{i}",
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        ))
        i += 1

    cameras = {
        cfg.name: OpenCVCamera(cfg) for i, cfg in enumerate(camera_configs)
    }

    print("Connecting cameras...")
    for cam in cameras.values():
        cam.connect()
    print(f"  {len(cameras)} camera(s) connected.")

    # -- Mock follower -------------------------------------------------------
    print("Creating mock follower robot...")
    follower = AliciaFollower(arm_mode=args.arm_mode, left_port=None, right_port=None)
    follower.connect()

    # -- Load policy ---------------------------------------------------------
    print(f"Loading policy from {args.checkpoint} ...")
    policy, preprocessor, postprocessor, dataset_meta = load_policy(
        checkpoint=args.checkpoint,
        dataset_root=args.dataset_root,
        device=args.device,
        camera_height=args.camera_height,
        camera_width=args.camera_width,
    )
    policy_camera_names = policy_image_camera_names(dataset_meta, policy)
    print(f"  Policy expects cameras: {policy_camera_names}")
    print(f"  Device: {args.device}")
    print("Policy loaded.\n")

    display = not args.no_display
    if display and cv2 is None:
        print("OpenCV not available; disabling display.")
        display = False
    if display and cv2 is not None:
        ok_gui, gui_reason = _gui_preview_available()
        if not ok_gui:
            # #region agent log
            _agent_debug_log(
                "verify",
                "test_model.py:main",
                "auto-disabled GUI preview",
                {"reason": gui_reason},
                run_id="post-fix",
            )
            # #endregion
            print(f"Camera preview disabled: {gui_reason}")
            display = False

    # -- Interactive task loop -----------------------------------------------
    try:
        while True:
            print("-" * 50)
            task = input("Enter task description (or 'quit' to exit): ").strip()
            if not task:
                print("Empty task, please try again.")
                continue
            if task.lower() in ("exit", "quit", "q"):
                print("Exiting.")
                break

            print(f"Starting task: {task}")
            print("Press 'q' in the camera window or Ctrl-C to end the current task.\n")

            run_task(
                task=task,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset_meta=dataset_meta,
                follower=follower,
                cameras=cameras,
                camera_names_for_policy=policy_camera_names,
                camera_height=args.camera_height,
                camera_width=args.camera_width,
                device=args.device,
                control_hz=args.control_hz,
                display=display,
                log_forward_loss=args.log_forward_loss,
            )
            print()

    except (KeyboardInterrupt, EOFError):
        print("\nShutting down...")
    finally:
        if display and cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        for cam in cameras.values():
            cam.disconnect()
        follower.disconnect()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
