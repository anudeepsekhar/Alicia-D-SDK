# API Reference

This section introduces the core classes and method interfaces of the Alicia-D SDK.

---

## Initialization Interface: `create_robot`

```python
from alicia_d_sdk import create_robot

robot = create_robot(
    port="",                    # Serial port (empty string for auto-detection)
    robot_version="v5_6",       # Robot structure version
    gripper_type="50mm",        # Gripper type
    debug_mode=False,           # Debug mode
)
```

---

## Control Interface: `alicia_d_sdk.api.synria_robot_api.SynriaRobotAPI`

```python
from alicia_d_sdk import create_robot

robot = create_robot()
```

### Main Methods:

#### Connection Management:
- `connect()`  
  Connect to the robot and detect firmware version

- `disconnect()`  
  Disconnect from the robot and stop update thread

- `is_connected()`  
  Check if the robot is connected

#### Motion Control:
- `set_home(speed_deg_s=10)`  
  Move the robot to the initial position

- `set_robot_target(target_joints=None, gripper_value=None, joint_format='rad', speed_deg_s=10, wait_for_completion=True)`  
  Unified interface for setting joint and gripper targets, supports simultaneous setting of joint angles and gripper position

- `set_joint_target(target_joints, joint_format='rad')`  
  Move the robot to target joint angles (direct setting, new firmware)

- `set_joint_target_interplotation(target_joints, joint_format='rad', speed_factor=1.0, T_default=4.0, n_steps_ref=200, visualize=False)`  
  Smoothly move the robot to target joint angles using interpolation (old firmware)

- `set_pose_target(target_pose, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_deg_s=10, execute=True)`  
  Move the end-effector to target pose using inverse kinematics

- `move_joint_trajectory(q_end, duration=2.0, method='cubic', num_points=100, visualize=False)`  
  Execute smooth joint trajectory to target position

- `move_cartesian_linear(target_pose, duration=2.0, num_points=50, ik_method='dls', visualize=False)`  
  Execute Cartesian linear trajectory to target pose

#### Status Retrieval:
- `get_robot_state(robot_type=None)`  
  Get current complete robot state, returns `JointState` object containing:
  - `angles`: List of six joint angles (radians)
  - `gripper`: Gripper opening value (0-1000, 0 is fully closed, 1000 is fully open)
  - `timestamp`: Timestamp (seconds)
  - `run_status_text`: Run status text, possible values:
    - `"idle"`: Idle state
    - `"locked"`: Locked state
    - `"sync"`: Sync state
    - `"sync_locked"`: Sync locked state
    - `"overheat"`: Overheat state
    - `"overheat_protect"`: Overheat protection state
    - `"unknown"`: Unknown state

- `get_joints(robot_type=None)`  
  Return current joint angles list (radians), extracted from `get_robot_state()`

- `get_pose()`  
  Get current end-effector position and orientation, returns a dictionary containing `transform`, `position`, `rotation`, `euler_xyz`, `quaternion_xyzw`

- `get_gripper()`  
  Return current gripper opening (0-1000, 0 is fully closed, 1000 is fully open), extracted from `get_robot_state()`

- `get_temperature(timeout=5.0)`  
  Get current servo temperatures list (Celsius)

- `get_velocity(timeout=1.0)`  
  Get current servo velocities list (degrees per second)

- `get_self_check(timeout=1.0)`  
  Execute machine self-check (servo health status), returns detailed status dictionary

- `get_firmware_version(timeout=5.0, send_interval=0.2)`  
  Query robot firmware version, returns dictionary with `serial_number`, `hardware_version`, `firmware_version`

- `print_state(continuous=False, output_format='deg')`  
  Print current robot information, supports continuous printing, supports angle/radian format. Includes joint angles, gripper state, end-effector pose, temperature, velocity, and more

#### Gripper Control:
- `set_gripper_target(command=None, value=None, wait_for_completion=True, timeout=5.0, tolerance=1.0)`  
  Control gripper position, command can be 'open' or 'close', value range 0-100

- `set_robot_target(gripper_value=...)`  
  Control gripper through unified interface, gripper_value range 0-1000 (0 is fully closed, 1000 is fully open)

#### System Control:
- `torque_control(command)`  
  Enable or disable torque ('on' or 'off')

- `zero_calibration()`  
  Execute zero calibration process: disable torque → manual drag → re-enable torque → record zero point


---

## Hardware Layer Interface: `alicia_d_sdk.hardware.ServoDriver`

Provides low-level serial communication, data parsing, and motor control functionality.

Main methods include:
- `connect()` / `disconnect()`
- `get_joint_angles()` / `set_joint_angles(...)`
- `get_joint_state()` / `get_gripper_data()`
- `set_gripper(...)`
- `enable_torque()` / `disable_torque()`
- `set_zero_position()`

It is not recommended for users to use this class directly. It is recommended to operate through the `SynriaRobotAPI` high-level interface.


---

## RoboCore Integration

The SDK integrates the [RoboCore](https://github.com/Synria-Robotics/RoboCore) library, providing high-performance kinematics and trajectory planning functionality:

### Kinematics Functions (from robocore.kinematics):
- `forward_kinematics(robot_model, q, backend='numpy', return_end=True)`
- `inverse_kinematics(robot_model, pose, q_init, backend='numpy', method='dls', ...)`
- `jacobian(robot_model, q, backend='numpy', method='analytic')`

### Trajectory Planning Functions (from robocore.planning):
- `cubic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `quintic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `linear_joint_trajectory(q_start, q_end, duration, num_points)`
- `linear_cartesian_trajectory(robot_model, pose_start, pose_end, duration, ...)`
- `trapezoidal_velocity_profile(distance, max_vel, max_acc)`

---

For more details, please refer to the source code documentation or check the log files in the `logs/` directory.

