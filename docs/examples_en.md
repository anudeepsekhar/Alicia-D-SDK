# Example Code Guide

The `examples/` directory contains multiple demonstration scripts showing how to use the Alicia-D SDK to control the robotic arm.

---

## 📁 File Structure

```
examples/
├── 00_demo_read_version.py          # Read version number
├── 01_torque_switch.py               # Torque control
├── 02_demo_zero_calibration.py      # Zero calibration
├── 03_demo_read_state.py            # Read state
├── 04_demo_move_gripper.py          # Gripper control
├── 05_demo_move_joint.py            # Joint space motion
├── 06_demo_move_cartesian.py        # Cartesian space motion
├── 07_demo_forward_kinematics.py    # Forward kinematics
├── 08_demo_inverse_kinematics.py    # Inverse kinematics
├── 09_demo_drag_teaching.py         # Drag teaching
└── 10_demo_sparkvis.py              # SparkVis UI bidirectional sync
```

## 📜 Example List

### 0. `00_demo_read_version.py`
**Read Robot Firmware Version**
- Always run this script first to detect firmware version before use
- If version is 6.x.x, SDK will automatically use new firmware mode (direct control)
- If version is below 6.x.x, SDK will use old firmware mode (interpolation control)
- If timeout or no version output is displayed, baud rate may need adjustment

**Usage:**
```bash
python 00_demo_read_version.py
```

---

### 1. `01_torque_switch.py`
Switch the torque state (power on/off) of all robot joints. After power off, the robot arm can be freely dragged.

**Parameters:**
- `--port`: Serial port (optional)

**Features:**
- First disable torque, then enable torque
- Robot arm can be manually dragged after torque is disabled
- Re-enable torque after teaching is complete

**Notes:**
- Manually support the robot arm before disabling torque to prevent sudden drop
- Always re-enable torque after teaching is complete

**Usage:**
```bash
python 01_torque_switch.py
```

---

### 2. `02_demo_zero_calibration.py`
Set the current position of the robot arm as the new zero point. **This operation is irreversible, use with caution.**

**Parameters:**
- `--port`: Serial port (optional)

**Use Cases:**
- First use of robot arm or after long period of non-use
- When joint angles show deviation
- When need to re-establish zero reference

**Usage:**
```bash
python 02_demo_zero_calibration.py
```

---

### 3. `03_demo_read_state.py`
Continuously read and print robot arm joint angles, end-effector pose, and gripper state. Supports single or continuous printing mode.

**Parameters:**
- `--port`: Serial port (optional)
- `--robot_version`: Robot version, default `v5_6`
- `--gripper_type`: Gripper model, default `50mm`
- `--format`: Angle display format, optional `rad` (radians) or `deg` (degrees), default `deg`
- `--single`: Print state once, default is continuous printing

**Use Cases:**
- Debugging and troubleshooting
- Real-time monitoring of robot arm state
- Verifying control command execution effects

**Usage:**
```bash
# Continuous printing (press Ctrl+C to stop)
python 03_demo_read_state.py --robot_version v5_6 --gripper_type 50mm

# Single print
python 03_demo_read_state.py --single
```

---

### 4. `04_demo_move_gripper.py`
Gripper control: Control gripper to open or close to specified angle. Gripper value range is 0-1000 (0 is fully closed, 1000 is fully open).

**Parameters:**
- `--port`: Serial port (optional)

**Features:**
- Automatic demo execution: fully open → fully close → half open
- Uses `set_robot_target(gripper_value=...)` to control gripper
- Supports waiting for gripper motion completion

**Usage:**
```bash
python 04_demo_move_gripper.py
```

---

### 5. `05_demo_move_joint.py`
Use joint space motion to control robot arm movement to set angles. Supports degree and radian input, automatically performs joint angle interpolation.

**Parameters:**
- `--port`: Serial port (optional)
- `--speed_deg_s`: Joint motion speed (degrees/second), default 10, range 5-400

**Features:**
- Automatic demo execution: home → move to target angle → home
- Uses unified joint and gripper target interface `set_robot_target()`
- Supports `wait_for_completion=True` to wait for motion completion

**Usage:**
```bash
python 05_demo_move_joint.py --port /dev/ttyUSB0 --speed_deg_s 10
```

---

### 6. `06_demo_move_cartesian.py`
Multi-point Cartesian trajectory planning. Supports manual drag teaching to record multiple waypoints, then execute continuous or step-by-step trajectory motion.

**Parameters:**
- `--port`: Serial port (optional)
- `--speed_deg_s`: Joint motion speed (degrees/second), default 10
- `--move_duration`: Movement time for each waypoint (seconds), default 3.0
- `--num_points`: Trajectory interpolation points, default 200
- `--ik_method`: Inverse kinematics solving method, optional `dls`, `pinv`, `lm`, default `dls`
- `--visualize`: Enable trajectory visualization

**Features:**
- First move to initial position (home)
- Manual drag to record waypoints (using `CartesianWaypointPlanner`)
- Choose execution mode: continuous or step-by-step
- Use inverse kinematics to solve and execute trajectory

**Usage:**
```bash
python 06_demo_move_cartesian.py --port /dev/ttyUSB0 --speed_deg_s 10 --move_duration 3.0 --num_points 200 --ik_method dls
```

---

### 7. `07_demo_forward_kinematics.py`
Forward kinematics solving. Calculate end-effector pose based on current joint angles, display position, rotation matrix, Euler angles, quaternion and other representation forms.

**Parameters:**
- `--port`: Serial port (optional)

**Features:**
- Uses RoboCore library robot model
- Displays robot model information and kinematic chain
- Returns and displays: position, rotation matrix, Euler angles (radians/degrees), quaternion, homogeneous transformation matrix
- Note: Quaternions q and -q represent the same rotation

**Usage:**
```bash
python 07_demo_forward_kinematics.py --port /dev/ttyUSB0
```

---

### 8. `08_demo_inverse_kinematics.py`
Demonstrate inverse kinematics solving: Calculate and optionally execute joint angles based on given end-effector target pose. Supports multiple IK solving methods and multi-start optimization.

**Parameters:**
- `--port`: Serial port (optional)
- `--speed_deg_s`: Joint motion speed (degrees/second), default 10
- `--end-pose`: Target pose (7 floats: px py pz qx qy qz qw), default value is preset
- `--method`: IK method, optional `dls` (damped least squares), `pinv` (pseudo-inverse), `transpose` (Jacobian transpose), default `dls`
- `--max-iters`: Maximum iterations, default 100
- `--multi-start`: Multi-start attempt count, 0 means disabled, recommended 5-10
- `--use-random-init`: Use random initial value (default uses current joint angles)
- `--execute`: Execute movement to solved position

**Features:**
- Displays detailed solving results (success/failure, iteration count, position error, orientation error)
- If movement is executed, automatically returns to initial position
- Supports multi-start optimization to improve success rate

**Usage:**
```bash
# Only calculate inverse solution, do not execute motion
python 08_demo_inverse_kinematics.py

# Calculate inverse solution and control robot arm to move to target pose
python 08_demo_inverse_kinematics.py --execute --speed_deg_s 10

# Use multi-start optimization to improve success rate
```

---

### 9. `09_demo_drag_teaching.py`
Drag teaching demonstration: Record a series of trajectory points by manually dragging the robot arm, and can replay. Supports manual interpolation, automatic fast, and replay-only modes.

**Parameters:**
- `--port`: Serial port (optional)
- `--speed_deg_s`: Joint motion speed (degrees/second), default 10
- `--mode`: Drag teaching mode, optional `manual` (manual interpolation), `auto` (automatic fast), or `replay_only` (replay only), default `auto`
- `--sample-hz`: Automatic mode sampling frequency, default 300.0 Hz
- `--save-motion`: Motion name (recording mode: new motion name; replay mode: existing motion name)
- `--list-motions`: List all available motions and exit

**Features:**
- Disable torque to enter teaching mode
- Manually drag robot arm to record trajectory
- Supports manual mode (record key points) and automatic mode (continuous sampling)
- Replay recorded trajectory

**Usage:**
```bash
# List all available motions
python 09_demo_drag_teaching.py --list-motions

# Automatic mode record trajectory named "my_demo"
python 09_demo_drag_teaching.py --mode auto --save-motion my_demo

# Manual mode record key point trajectory
python 09_demo_drag_teaching.py --mode manual --save-motion key_points

# Replay existing trajectory "my_demo"
python 09_demo_drag_teaching.py --mode replay_only --save-motion my_demo
```

---

### 10. `10_demo_sparkvis.py`
SparkVis UI bidirectional synchronization and data logging. Start WebSocket server to achieve UI ↔ Robot bidirectional sync, supports data logging to CSV file.

**Parameters:**
- `--port`: Serial port (optional)
- `--host`: WebSocket host, default `localhost`
- `--websocket-port`: WebSocket port, default 8765
- `--output-file`: CSV output path (empty string means no logging)
- `--log-source`: Log source, optional `ui` (UI commands), `robot` (robot state), or `both`, default `ui`
- `--enable-robot-sync`: Enable robot→UI state synchronization
- `--robot-sync-rate`: Robot state broadcast frequency (Hz), default 50.0

**Features:**
- UI → Robot: Receive joint_update and send directly to real robot
- Robot → UI: Periodically broadcast current robot state to UI (toggleable)
- Data logging: Record joint data from UI commands to CSV (optional)

**Usage:**
```bash
# Basic usage (need to start SparkVis backend and web server first)
python 10_demo_sparkvis.py 

# Enable robot state sync and log data
python 10_demo_sparkvis.py  --enable-robot-sync --output-file data.csv --log-source both
```

**Note**: Using this feature requires running simultaneously:
1. SparkVis backend server: `cd SparkVis && python backend_server.py`
2. SparkVis web server: `cd SparkVis && python -m http.server 8080`
3. This robot bridge script: `python 10_demo_sparkvis.py --port /dev/ttyUSB0`
4. Open browser and visit: `http://localhost:8080`

---

## ⚙️ Common Parameter Adjustment Scenarios

### Motion Control Parameter Adjustment:

*New Firmware (6.1.0 and above)*

- **Speed too fast**: Reduce `speed_deg_s` (range: 5-400 degrees/second)
- **Motion not smooth**: Increase `num_points` (trajectory interpolation points)
- **Movement time too long**: Reduce `move_duration` (movement time per waypoint)

### IK Solving Parameter Adjustment:
- **Solving fails**: Try different `method` (`dls`, `pinv`, `transpose`), increase `multi_start` (recommended 5-10)
- **Local optimum**: Use `use_random_init=True` or increase `multi_start`
- **High precision required**: Reduce `tolerance` to 1e-5 or smaller, increase `max_iters`
- **High speed required**: Use `method='pinv'` or `method='transpose'`

### Drag Teaching Parameter Adjustment:
- **Sampling frequency**: Adjust `--sample-hz` (default 300.0 Hz), higher frequency records more detail but larger files
- **Mode selection**: `manual` mode suitable for recording key points, `auto` mode suitable for continuous trajectory

---

