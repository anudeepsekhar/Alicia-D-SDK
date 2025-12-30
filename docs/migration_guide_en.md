# API Migration Guide: Upgrading from v6.0.0 to v6.1.0

This guide will help you migrate your code from `alicia_d_sdk` v6.0.0 to v6.1.0.

---

## 📋 Major Changes Overview

### 1. Speed Parameter Changes
- **Old Version**: Used `speed_factor` (speed multiplier, float)
- **New Version**: Uses `speed_deg_s` (speed in degrees/second, integer, range 5-400)

### 2. Gripper Value Range Changes
- **Old Version**: Gripper value range 0-100
- **New Version**: Gripper value range 0-1000 (0 is fully closed, 1000 is fully open)

### 3. New Unified Interface
- New `set_robot_state()` method that can set joint angles and gripper position simultaneously

### 4. Unified Status Retrieval Interface
- **New Version**: Use unified `get_robot_state(info_type)` interface for all status information
- Old methods `get_joints()`, `get_gripper()`, `get_version()`, `get_temperature()`, `get_velocity()`, `get_self_check()` have been removed
- `get_robot_state()` returns different data types based on `info_type` parameter

---

## 🔄 Detailed Migration Steps

### 1. Speed Parameter Migration

#### v6.0.0 Code
```python
robot.set_home(speed_factor=1.0)
robot.set_pose_target(target_pose, speed_factor=1.2)
```

#### v6.1.0 Code
```python
robot.set_home(speed_deg_s=10)
robot.set_pose_target(target_pose, speed_deg_s=20)
```

**Migration Notes**:
- `speed_factor` was a relative speed multiplier (>1 faster, <1 slower)
- `speed_deg_s` is absolute speed (degrees/second), range 5-400
- Recommended migration: `speed_factor=1.0` → `speed_deg_s=10`, `speed_factor=1.2` → `speed_deg_s=12`

---

### 2. Gripper Control Migration

#### v6.0.0 Code
```python
# Gripper value range 0-100
robot.set_gripper_target(value=50)  # 50% open
robot.set_gripper_target(command='open')  # Fully open
```

#### v6.1.0 Code
```python
# Gripper value range 0-1000
robot.set_robot_state(gripper_value=500)  # 50% open (recommended unified interface)
robot.set_gripper_target(value=500)  # Still supported, but value range changed
robot.set_gripper_target(command='open')  # Still supported
```

**Migration Notes**:
- Old values need to be multiplied by 10: `value=50` → `gripper_value=500`
- Recommended to use new unified `set_robot_state()` interface

---

### 3. Unified Interface Usage

#### v6.0.0 Code
```python
# Set joint and gripper separately
robot.set_joint_target(target_joints, joint_format='deg')
robot.set_gripper_target(value=50)
```

#### v6.1.0 Code
```python
# Use unified interface to set both simultaneously
robot.set_robot_state(
    target_joints=target_joints,
    gripper_value=500,
    joint_format='deg',
    speed_deg_s=10,
    wait_for_completion=True
)
```

**Advantages**:
- Single call for multiple operations
- Better synchronization control
- Supports wait for completion option

---

### 4. Status Retrieval Migration

#### v6.0.0 Code
```python
joints = robot.get_joints()
gripper = robot.get_gripper()
version = robot.get_firmware_version()
```

#### v6.1.0 Code
```python
# Unified interface: Get complete state (recommended)
state = robot.get_robot_state("joint_gripper")
joints = state.angles
gripper = state.gripper
run_status = state.run_status_text  # Run status

# Unified interface: Get separately
joints = robot.get_robot_state("joint")  # Joint angles only
gripper = robot.get_robot_state("gripper")  # Gripper value only

# Version info
version_info = robot.get_robot_state("version")  # Returns dictionary
serial_number = version_info['serial_number']
hardware_version = version_info['hardware_version']
firmware_version = version_info['firmware_version']

# Other status information
temperatures = robot.get_robot_state("temperature")  # Servo temperatures
velocities = robot.get_robot_state("velocity")  # Servo velocities
self_check = robot.get_robot_state("self_check")  # Self-check status
gripper_type = robot.get_robot_state("gripper_type")  # Gripper type
```

**Migration Notes**:
- All status retrieval methods are now unified as `get_robot_state(info_type)`
- `info_type` parameter determines the return data type
- Use `"joint_gripper"` to get both joint and gripper data in one call (more efficient)
- Old methods `get_joints()`, `get_gripper()`, `get_version()`, `get_temperature()`, `get_velocity()`, `get_self_check()` have been removed

---

### 5. Motion Control Method Migration

#### `set_home()` Method

#### v6.0.0 Code
```python
robot.set_home(speed_factor=1.0)
```

#### v6.1.0 Code
```python
robot.set_home(speed_deg_s=10)
```

---

#### `set_pose_target()` Method

#### v6.0.0 Code
```python
robot.set_pose_target(
    target_pose,
    speed_factor=1.0,
    execute=True
)
```

#### v6.1.0 Code
```python
robot.set_pose_target(
    target_pose,
    speed_deg_s=10,
    execute=True
)
```

---

#### `move_cartesian_linear()` Method

#### v6.0.0 Code
```python
robot.move_cartesian_linear(
    target_pose,
    duration=2.0,
    num_points=50
)
```

#### v6.1.0 Code
```python
robot.move_cartesian_linear(
    target_pose,
    speed_deg_s=10,  # New parameter
    duration=2.0,
    num_points=50
)
```

---

## 📝 Complete Migration Examples

### Example 1: Basic Motion Control

#### v6.0.0
```python
from alicia_d_sdk import create_robot

robot = create_robot()
if robot.connect():
    # Move to home position
    robot.set_home(speed_factor=1.0)
    
    # Set joint angles
    target_joints = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
    robot.set_joint_target(target_joints, joint_format='rad')
    
    # Control gripper
    robot.set_gripper_target(value=50)
    
    robot.disconnect()
```

#### v6.1.0
```python
from alicia_d_sdk import create_robot

robot = create_robot()
if robot.connect():
    # Move to home position
    robot.set_home(speed_deg_s=10)
    
    # Use unified interface to set both joint and gripper
    target_joints = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
    robot.set_robot_state(
        target_joints=target_joints,
        gripper_value=500,  # 50% open (old value 50 * 10)
        joint_format='rad',
        speed_deg_s=10,
        wait_for_completion=True
    )
    
    robot.disconnect()
```

---

### Example 2: Status Monitoring

#### v6.0.0
```python
robot.connect()
joints = robot.get_joints()
gripper = robot.get_gripper()
print(f"Joints: {joints}, Gripper: {gripper}")
```

#### v6.1.0
```python
robot.connect()

# Use unified interface to get complete state
state = robot.get_robot_state("joint_gripper")
print(f"Joints: {state.angles}")
print(f"Gripper: {state.gripper}")
print(f"Status: {state.run_status_text}")

# Or get separately
joints = robot.get_robot_state("joint")
gripper = robot.get_robot_state("gripper")

# Get additional information
temperatures = robot.get_robot_state("temperature")
velocities = robot.get_robot_state("velocity")
print(f"Temperatures: {temperatures}")
print(f"Velocities: {velocities}")
```

---

## ⚠️ Important Notes

1. **Speed Parameter Conversion**:
   - Old code with `speed_factor=1.0` typically corresponds to `speed_deg_s=10`
   - Adjust speed values according to actual needs, range 5-400 degrees/second

2. **Gripper Value Conversion**:
   - All gripper values need to be multiplied by 10
   - Example: `value=50` → `gripper_value=500`

3. **API Changes**:
   - `get_joints()`, `get_gripper()`, `get_version()`, `get_temperature()`, `get_velocity()`, `get_self_check()` have been removed
   - Use unified `get_robot_state(info_type)` interface
   - `set_joint_target()`, `set_gripper_target()`, `set_joint_target_interplotation()` have been removed
   - Use unified `set_robot_state()` interface
   - `set_pose_target()` has been renamed to `set_pose()`

4. **Version Check**:
   - After upgrade, recommend running `00_demo_read_version.py` to verify version
   - Check if firmware version is compatible

---

## 🔍 Frequently Asked Questions

### Q: How to determine appropriate speed value?
A: Recommend starting with `speed_deg_s=10`, adjust according to actual motion effects. Range 5-400 degrees/second.

### Q: How to convert gripper values?
A: Multiply old value by 10. Example: `50` → `500`, `100` → `1000`.

### Q: Can old code still work?
A: Most old code still works, but need to pay attention to parameter changes. Recommend gradually migrating to new interfaces.

### Q: How to check run status?
A: Use `get_robot_state()` to get `run_status_text` field, which shows current run status.

---

## 📚 Related Documentation

- [API Reference](api_reference_en.md)
- [Examples Guide](examples_en.md)
- [Installation Guide](installation_en.md)

---

If you have questions, please refer to the [API Reference](api_reference_en.md) or check the example code.

