# API 迁移指南：从 v6.0.0 升级到 v6.1.0

本指南将帮助您将代码从 `alicia_d_sdk` v6.0.0 迁移到 v6.1.0。

---

## 📋 主要变更概述

### 1. 速度参数变更
- **旧版本**: 使用 `speed_factor`（速度倍率，浮点数）
- **新版本**: 使用 `speed_deg_s`（速度，度/秒，整数，范围 5-400）

### 2. 夹爪值范围变更
- **旧版本**: 夹爪值范围 0-100
- **新版本**: 夹爪值范围 0-1000（0为完全闭合，1000为完全张开）

### 3. 新增统一接口
- 新增 `set_robot_target()` 方法，可同时设置关节角度和夹爪位置

### 4. 状态获取增强
- 新增 `get_robot_state()` 返回完整状态对象
- 新增 `get_temperature()`、`get_velocity()`、`get_self_check()` 方法
- `get_version()` 现在返回完整版本信息字典

---

## 🔄 详细迁移步骤

### 1. 速度参数迁移

#### v6.0.0 代码
```python
robot.set_home(speed_factor=1.0)
robot.set_pose_target(target_pose, speed_factor=1.2)
```

#### v6.1.0 代码
```python
robot.set_home(speed_deg_s=10)
robot.set_pose_target(target_pose, speed_deg_s=20)
```

**迁移说明**:
- `speed_factor` 是相对速度倍率（>1更快，<1更慢）
- `speed_deg_s` 是绝对速度（度/秒），范围 5-400
- 建议迁移值：`speed_factor=1.0` → `speed_deg_s=10`，`speed_factor=1.2` → `speed_deg_s=12`

---

### 2. 夹爪控制迁移

#### v6.0.0 代码
```python
# 夹爪值范围 0-100
robot.set_gripper_target(value=50)  # 50% 开合
robot.set_gripper_target(command='open')  # 完全打开
```

#### v6.1.0 代码
```python
# 夹爪值范围 0-1000
robot.set_robot_target(gripper_value=500)  # 50% 开合（推荐使用统一接口）
robot.set_gripper_target(value=500)  # 仍然支持，但值范围已改变
robot.set_gripper_target(command='open')  # 仍然支持
```

**迁移说明**:
- 旧值需要乘以 10：`value=50` → `gripper_value=500`
- 推荐使用新的 `set_robot_target()` 统一接口

---

### 3. 统一接口使用

#### v6.0.0 代码
```python
# 分别设置关节和夹爪
robot.set_joint_target(target_joints, joint_format='deg')
robot.set_gripper_target(value=50)
```

#### v6.1.0 代码
```python
# 使用统一接口同时设置
robot.set_robot_target(
    target_joints=target_joints,
    gripper_value=500,
    joint_format='deg',
    speed_deg_s=10,
    wait_for_completion=True
)
```

**优势**:
- 一次调用完成多个操作
- 更好的同步控制
- 支持等待完成选项

---

### 4. 状态获取迁移

#### v6.0.0 代码
```python
joints = robot.get_joints()
gripper = robot.get_gripper()
version = robot.get_firmware_version()
```

#### v6.1.0 代码
```python
# 方式1：获取完整状态（推荐）
state = robot.get_robot_state()
joints = state.angles
gripper = state.gripper
run_status = state.run_status_text  # 新增：运行状态

# 方式2：分别获取（仍然支持）
joints = robot.get_joints()
gripper = robot.get_gripper()

# 版本信息（增强）
version_info = robot.get_version()  # 返回字典
serial_number = version_info['serial_number']
hardware_version = version_info['hardware_version']
firmware_version = version_info['firmware_version']

# 新增状态获取方法
temperatures = robot.get_temperature()  # 舵机温度
velocities = robot.get_velocity()  # 舵机速度
self_check = robot.get_self_check()  # 自检状态
```

**新增功能**:
- `run_status_text`: 运行状态文本（"idle", "locked", "sync", "overheat" 等）
- `get_temperature()`: 获取舵机温度
- `get_velocity()`: 获取舵机速度
- `get_self_check()`: 执行机器自检

---

### 5. 运动控制方法迁移

#### `set_home()` 方法

#### v6.0.0 代码
```python
robot.set_home(speed_factor=1.0)
```

#### v6.1.0 代码
```python
robot.set_home(speed_deg_s=10)
```

---

#### `set_pose_target()` 方法

#### v6.0.0 代码
```python
robot.set_pose_target(
    target_pose,
    speed_factor=1.0,
    execute=True
)
```

#### v6.1.0 代码
```python
robot.set_pose_target(
    target_pose,
    speed_deg_s=10,
    execute=True
)
```

---

#### `move_cartesian_linear()` 方法

#### v6.0.0 代码
```python
robot.move_cartesian_linear(
    target_pose,
    duration=2.0,
    num_points=50
)
```

#### v6.1.0 代码
```python
robot.move_cartesian_linear(
    target_pose,
    speed_deg_s=10,  # 新增参数
    duration=2.0,
    num_points=50
)
```

---

## 📝 完整迁移示例

### 示例 1：基本运动控制

#### v6.0.0
```python
from alicia_d_sdk import create_robot

robot = create_robot()
if robot.connect():
    # 移动到初始位置
    robot.set_home(speed_factor=1.0)
    
    # 设置关节角度
    target_joints = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
    robot.set_joint_target(target_joints, joint_format='rad')
    
    # 控制夹爪
    robot.set_gripper_target(value=50)
    
    robot.disconnect()
```

#### v6.1.0
```python
from alicia_d_sdk import create_robot

robot = create_robot()
if robot.connect():
    # 移动到初始位置
    robot.set_home(speed_deg_s=10)
    
    # 使用统一接口同时设置关节和夹爪
    target_joints = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
    robot.set_robot_target(
        target_joints=target_joints,
        gripper_value=500,  # 50% 开合（旧值50 * 10）
        joint_format='rad',
        speed_deg_s=10,
        wait_for_completion=True
    )
    
    robot.disconnect()
```

---

### 示例 2：状态监控

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

# 获取完整状态
state = robot.get_robot_state()
print(f"Joints: {state.angles}")
print(f"Gripper: {state.gripper}")
print(f"Status: {state.run_status_text}")  # 新增

# 获取额外信息
temperatures = robot.get_temperature()
velocities = robot.get_velocity()
print(f"Temperatures: {temperatures}")
print(f"Velocities: {velocities}")
```

---

## ⚠️ 注意事项

1. **速度参数转换**:
   - 旧代码中的 `speed_factor=1.0` 通常对应 `speed_deg_s=10`
   - 根据实际需求调整速度值，范围 5-400 度/秒

2. **夹爪值转换**:
   - 所有夹爪值需要乘以 10
   - 例如：`value=50` → `gripper_value=500`

3. **向后兼容性**:
   - `get_joints()` 和 `get_gripper()` 仍然可用
   - `set_gripper_target()` 仍然可用，但值范围已改变
   - 建议使用新的统一接口 `set_robot_target()`

4. **版本检查**:
   - 升级后建议运行 `00_demo_read_version.py` 验证版本
   - 检查固件版本是否兼容

---

## 🔍 常见问题

### Q: 如何确定合适的速度值？
A: 建议从 `speed_deg_s=10` 开始，根据实际运动效果调整。范围 5-400 度/秒。

### Q: 夹爪值如何转换？
A: 旧值乘以 10。例如：`50` → `500`，`100` → `1000`。

### Q: 旧代码还能用吗？
A: 大部分旧代码仍然可用，但需要注意参数变更。建议逐步迁移到新接口。

### Q: 如何检查运行状态？
A: 使用 `get_robot_state()` 获取 `run_status_text` 字段，可以查看当前运行状态。

---

## 📚 相关文档

- [API 参考](api_reference.md)
- [示例代码](examples.md)
- [安装指南](installation.md)

---

如有问题，请参考 [API 参考文档](api_reference.md) 或查看示例代码。

