# Training a VLA Model for the Alicia-D Robot

## Robot Overview

- **Alicia-D**: 6-DOF serial manipulator + 1 gripper (7 controllable dimensions)
- **Control**: Joint-space position commands (radians), Cartesian pose via IK, gripper 0–1000
- **Feedback**: Joint angles, gripper, velocities, temperatures, FK-derived TCP pose
- **No built-in camera** — an external camera is required (the SDK has a `vision_agent.py` example using `cv2.VideoCapture`)
- **No Gym env** defined yet — one needs to be created

---

## Phase 1: Hardware & Environment Setup

1. **Mount camera(s)** — At minimum one wrist-mounted or third-person RGB camera. Most VLA papers (RT-2, OpenVLA, Octo) use 1–2 cameras at 224×224 or 256×256 resolution.
2. **Calibrate camera intrinsics/extrinsics** if you need depth or precise spatial reasoning (optional for pure VLA).
3. **Define your task space** — Pick 3–5 concrete manipulation tasks (e.g., pick-and-place, stacking, drawer open/close).

---

## Phase 2: Define Action & Observation Spaces

4. **Wrap the robot in a Gym-like interface** — Create an environment class that exposes:
   - **Observations**: RGB image(s) + proprioception (6 joint angles + gripper state + TCP pose)
   - **Actions**: 6 joint deltas (or absolute targets) + 1 gripper command (7-dim action vector)
   - **Control frequency**: Typically 5–10 Hz for VLA (the SDK serial throughput can support this)

5. **Normalize everything** — Actions and observations should be scaled to `[-1, 1]` or `[0, 1]` for stable training.

---

## Phase 3: Data Collection (Most Critical Step)

6. **Build a teleoperation system** — Options:
   - **Leader-follower**: The SDK supports a `leader_ur` variant — use a second arm as a leader with `torque_control('off')` for demonstration recording (like the `08_demo_drag_teaching.py` example).
   - **Keyboard/gamepad**: Simpler but lower quality demonstrations.
   - **VR controller**: Higher quality, more investment.

7. **Record demonstration datasets** — Each episode should capture at every timestep:
   - RGB image(s) from camera(s)
   - Joint angles (6 floats, rad)
   - Gripper state
   - TCP pose (from FK)
   - Action taken (the command sent)
   - Language instruction describing the task

8. **Target dataset size**:
   - Fine-tuning a pretrained VLA (OpenVLA, Octo): **50–200 demos per task**
   - Training from scratch: **1000+ demos per task** (not recommended without significant compute)

9. **Save in a standard format** — Use [LeRobot's dataset format](https://github.com/huggingface/lerobot) (HuggingFace) or [RLDS / Open X-Embodiment format](https://github.com/google-deepmind/open_x_embodiment) for compatibility with existing VLA codebases.

---

## Phase 4: Model Selection & Training

10. **Choose a VLA architecture** — Recommended options for a small lab:

| Model | Pretrained? | Fine-tune Data Needed | Compute |
|-------|-------------|----------------------|---------|
| **OpenVLA** (open source) | Yes (OXE data) | 50–200 demos/task | 1× A100 (fine-tune) |
| **Octo** (generalist) | Yes (OXE data) | 50–100 demos/task | 1× A100 (fine-tune) |
| **LeRobot ACT / Diffusion Policy** | No (but simpler) | 50–100 demos/task | 1× consumer GPU |
| **RT-2 style** | Needs VLM base | 500+ demos/task | 4+ A100s |

11. **Recommended starting point — LeRobot + ACT or Diffusion Policy**:
    - Native support for custom robots
    - Works with consumer GPUs (RTX 3090/4090)
    - Active community, well-documented
    - Not a "true" VLA (no language conditioning) but gets manipulation working fast

12. **For a true VLA — Fine-tune OpenVLA** on your data:
    - Converts (image, language instruction, proprioception) → action
    - Requires converting data to their format
    - ~8 hours on 1× A100 to fine-tune

---

## Phase 5: Training Pipeline

13. **Preprocess data** — Resize images, normalize actions, create train/val splits.
14. **Fine-tune the model** — Use LoRA or full fine-tuning depending on compute budget.
15. **Evaluate offline** — Check action prediction error on held-out demonstrations.
16. **Iterate on data quality** — Bad demonstrations hurt more than small dataset size.

---

## Phase 6: Deployment & Inference

17. **Build an inference loop** on the host machine:

```python
while running:
    image = camera.read()                    # RGB from webcam
    state = robot.get_robot_state()          # Joint angles + gripper
    language = "pick up the red block"       # Task instruction

    action = vla_model.predict(image, state, language)  # 7-dim action

    robot.set_robot_state(
        target_joints=action[:6],
        gripper_value=action[6],
        joint_format='rad'
    )
    time.sleep(1 / control_freq)  # 5-10 Hz
```

18. **Safety** — Add joint limit checks, velocity limits, and an emergency stop before commanding the real robot.
19. **Real-world tuning** — Use action chunking (predict N future actions) and temporal ensembling to smooth out jerky behavior.

---

## Recommended Concrete Path (Least to Most Effort)

1. **Start with LeRobot** — best tooling for data collection, training, and deployment on custom robots.
2. **Use drag teaching** (`08_demo_drag_teaching.py` already exists) to record demonstrations with language labels.
3. **Train ACT or Diffusion Policy** on a single task first.
4. **Once working**, scale to OpenVLA for language-conditioned multi-task behavior.

---

## Useful Links

- [LeRobot (HuggingFace)](https://github.com/huggingface/lerobot)
- [OpenVLA](https://openvla.github.io/)
- [Octo](https://octo-models.github.io/)
- [Open X-Embodiment](https://github.com/google-deepmind/open_x_embodiment)
- [ACT (Action Chunking with Transformers)](https://tonyzhaozh.github.io/aloha/)
- [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/)
