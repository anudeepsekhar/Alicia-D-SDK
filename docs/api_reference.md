# API 参考文档

本节介绍 Alicia-D SDK 的核心类与方法接口。

---

##  初始化接口：`create_robot`

```python
from alicia_d_sdk import create_robot

robot = create_robot(
    port="",                    # 串口（留空自动查找）
    baudrate=1000000,           # 波特率
    robot_version="v5_6",       # 机械臂结构版本
    gripper_type="50mm",        # 夹爪类型
    firmware_version=None,      # 固件版本（自动检测）
    debug_mode=False,           # 调试模式
    speed_deg_s=20.0           # 默认速度（度/秒），建议使用默认值
)
```

---

##  控制接口：`alicia_d_sdk.api.synria_robot_api.SynriaRobotAPI`

```python
from alicia_d_sdk import create_robot

robot = create_robot()
```

### 主要方法一览：

#### 连接管理：
- `connect()`  
  连接机械臂并检测固件版本

- `disconnect()`  
  断开机械臂连接并停止更新线程

- `is_connected()`  
  检查机械臂是否连接

#### 运动控制：
- `set_home(speed_factor=1.0)`  
  移动机械臂到初始位置

- `set_joint_target(target_joints, joint_format='rad')`  
  移动机械臂到目标关节角度（直接设置，新固件）

- `set_joint_target_interplotation(target_joints, joint_format='rad', speed_factor=1.0, T_default=4.0, n_steps_ref=200, visualize=False)`  
  使用插值平滑移动机械臂到目标关节角度（旧固件）

- `set_pose_target(target_pose, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_factor=1.0, execute=True)`  
  使用逆运动学移动末端执行器到目标位姿

- `move_joint_trajectory(q_end, duration=2.0, method='cubic', num_points=100, visualize=False)`  
  执行平滑的关节轨迹到目标位置

- `move_cartesian_linear(target_pose, duration=2.0, num_points=50, ik_method='dls', visualize=False)`  
  执行笛卡尔直线轨迹到目标位姿

#### 状态获取：
- `get_joints()`  
  返回当前关节角度（弧度）

- `get_pose()`  
  获取当前末端执行器位置与姿态，返回字典包含 `transform`, `position`, `rotation`, `euler_xyz`, `quaternion_xyzw`

- `get_gripper()`  
  返回当前夹爪开合度（0-100）

- `get_firmware_version(timeout=5.0, send_interval=0.2)`  
  查询机械臂固件版本

- `print_state(continuous=False, output_format='deg')`  
  打印当前机械臂信息，可持续打印，支持角度/弧度格式

#### 夹爪控制：
- `set_gripper_target(command=None, value=None, wait_for_completion=True, timeout=5.0, tolerance=1.0)`  
  控制夹爪位置，command 可选 'open' 或 'close'，value 范围 0-100

#### 系统控制：
- `torque_control(command)`  
  启用或关闭扭矩（'on' 或 'off'）

- `zero_calibration()`  
  执行归零校准流程：关闭扭矩 → 手动拖动 → 重启扭矩 → 记录零点

- `set_speed(speed_deg_s)`  
  设置机械臂运动速度（度/秒，仅新固件）

- `set_acceleration(acceleration)`  
  设置机械臂加速度

---

##  硬件层接口：`alicia_d_sdk.hardware.ServoDriver`

提供底层串口通信、数据解析和电机控制功能。

主要方法包括：
- `connect()` / `disconnect()`
- `get_joint_angles()` / `set_joint_angles(...)`
- `get_joint_state()` / `get_gripper_data()`
- `set_gripper(...)`
- `enable_torque()` / `disable_torque()`
- `set_zero_position()`
- `set_speed(...)` / `set_acceleration(...)`

不推荐用户直接使用此类，建议通过 `SynriaRobotAPI` 高级接口操作。


---

##  RoboCore 集成

SDK 集成了 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 库，提供高性能运动学和轨迹规划功能：

### 运动学功能（来自 robocore.kinematics）：
- `forward_kinematics(robot_model, q, backend='numpy', return_end=True)`
- `inverse_kinematics(robot_model, pose, q_init, backend='numpy', method='dls', ...)`
- `jacobian(robot_model, q, backend='numpy', method='analytic')`

### 轨迹规划功能（来自 robocore.planning）：
- `cubic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `quintic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `linear_joint_trajectory(q_start, q_end, duration, num_points)`
- `linear_cartesian_trajectory(robot_model, pose_start, pose_end, duration, ...)`
- `trapezoidal_velocity_profile(distance, max_vel, max_acc)`

---

如需更多细节，请参考源码文档或查看 `logs/` 下日志文件输出。