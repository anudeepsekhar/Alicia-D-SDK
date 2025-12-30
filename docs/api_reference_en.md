# API Reference

This section introduces the core classes and method interfaces of the Alicia-D SDK.

---

## Initialization Interface: `create_robot`

```python
from alicia_d_sdk import create_robot

robot = create_robot(
    port="",                    # Serial port (empty string for auto-detection)
    gripper_type=None,          # Gripper type ("50mm" or "100mm"), None to read from cache or default "50mm"
    debug_mode=False,           # Debug mode
    auto_connect=True,          # Auto-connect on creation
    base_link="base_link",      # Base link name
    end_link="tool0"            # End-effector link name
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

- `set_robot_state(target_joints=None, gripper_value=None, joint_format='rad', speed_deg_s=10, tolerance=0.1, timeout=10.0, wait_for_completion=True)`  
  Unified interface for setting joint and gripper targets, supports simultaneous setting of joint angles and gripper position
  
  **Parameters:**
  - `target_joints`: Optional target joint angles list (radians or degrees). If None, keeps current angles
  - `gripper_value`: Optional gripper value (0-1000, 0 is fully closed, 1000 is fully open). If None, keeps current value
  - `joint_format`: Unit format for joints, 'rad' (radians) or 'deg' (degrees), default 'rad'
  - `speed_deg_s`: Joint motion speed (degrees per second), range 0-360, default 10
  - `tolerance`: Joint target tolerance (radians), default 0.1
  - `timeout`: Maximum wait time (seconds), default 10.0
  - `wait_for_completion`: If True, wait until target reached, default True
  
  **Returns:** True if successful, False otherwise

- `set_pose(target_pose, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_deg_s=10, execute=True)`  
  Move the end-effector to target pose using inverse kinematics
  
  **Parameters:**
  - `target_pose`: Target pose as [x, y, z, qx, qy, qz, qw] (position + quaternion)
  - `backend`: Computation backend, 'numpy' or 'torch', default 'numpy'
  - `method`: IK solver method, 'dls' (damped least squares), 'pinv' (pseudo-inverse), or 'transpose', default 'dls'
  - `display`: Display solution details, default True
  - `tolerance`: Position and orientation tolerance, default 1e-4
  - `max_iters`: Maximum number of iterations, default 100
  - `multi_start`: Number of multi-start attempts, 0 to disable, default 0
  - `use_random_init`: Use random initial guess, default False (uses current joint angles)
  - `speed_deg_s`: Motion speed (degrees per second), default 10
  - `execute`: Execute motion if True, default True
  
  **Returns:** Dictionary with success, q, iters, pos_err, ori_err, message

#### Trajectory Planning:
- `plan_joint_trajectory(waypoints, planner_type='b_spline', duration=None, num_points=800, ...)`  
  Plan joint space trajectory, supports B-Spline and multi-segment planners

- `plan_cartesian_trajectory(waypoints, duration=None, num_points=100, backend='numpy')`  
  Plan Cartesian space spline trajectory

- `solve_ik_for_trajectory(target_poses, q_init=None, method='dls', ...)`  
  Batch solve inverse kinematics for a sequence of Cartesian poses

#### Status Retrieval:
- `get_robot_state(info_type="joint_gripper", timeout=1.0)`  
  Unified interface for retrieving robot state information. Returns different data types based on `info_type`:
  
  **Parameters:**
  - `info_type`: Type of information to retrieve. Options:
    - `"joint_gripper"`: Returns `JointState` object (default), containing:
      - `angles`: List of six joint angles (radians)
      - `gripper`: Gripper opening value (0-1000, 0 is fully closed, 1000 is fully open)
      - `timestamp`: Timestamp (seconds)
      - `run_status_text`: Run status text ("idle", "locked", "sync", "sync_locked", "overheat", "overheat_protect", "unknown")
    - `"joint"`: Returns only joint angles as `List[float]` (radians)
    - `"gripper"`: Returns only gripper value as `float` (0-1000)
    - `"version"`: Returns version info dictionary with `serial_number`, `hardware_version`, `firmware_version`
    - `"temperature"`: Returns servo temperatures as `List[float]` (Celsius)
    - `"velocity"`: Returns servo velocities as `List[float]` (degrees per second)
    - `"self_check"`: Returns self-check status dictionary with `raw_mask`, `bits`, `timestamp`
    - `"gripper_type"`: Returns gripper type string (e.g., "50mm" or "100mm")
  - `timeout`: Maximum wait time in seconds (default: 1.0)
  
  **Returns:** Data of the requested type, or `None` if failed

- `get_pose()`  
  Get current end-effector position and orientation, returns a dictionary containing `transform`, `position`, `rotation`, `euler_xyz`, `quaternion_xyzw`

- `print_state(continuous=False, output_format='deg')`  
  Print current robot information, supports continuous printing, supports angle/radian format. Includes joint angles, gripper state, end-effector pose, temperature, velocity, and more

#### Gripper Control:
- `set_robot_state(gripper_value=...)`  
  Control gripper through unified interface, gripper_value range 0-1000 (0 is fully closed, 1000 is fully open)
  
  **Examples:**
  ```python
  # Open gripper
  robot.set_robot_state(gripper_value=1000)
  
  # Close gripper
  robot.set_robot_state(gripper_value=0)
  
  # Set joints and gripper together
  robot.set_robot_state(target_joints=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], gripper_value=500)
  ```

#### System Control:
- `torque_control(command)`  
  Enable or disable torque ('on' or 'off')

- `zero_calibration()`  
  Execute zero calibration process: disable torque → manual drag → re-enable torque → record zero point


---

---

## RoboCore Integration

The SDK integrates the [RoboCore](https://github.com/Synria-Robotics/RoboCore) library, providing high-performance kinematics and trajectory planning functionality:

### Kinematics Functions (from robocore.kinematics):
- `forward_kinematics(robot_model, q, backend='numpy', return_end=True)`
- `inverse_kinematics(robot_model, pose, q_init, backend='numpy', method='dls', ...)`
- `jacobian(robot_model, q, backend='numpy', method='analytic')`


---

For more details, please refer to the source code documentation or check the log files in the `logs/` directory.

