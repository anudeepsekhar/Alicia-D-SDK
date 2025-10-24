# 例程代码说明

`examples/` 目录包含了多个演示脚本，用于展示如何使用 Alicia-D SDK 控制机械臂。

---

## 📁 文件结构

```
examples/
├── 00_demo_read_version.py          # 读取版本号
├── 01_torque_switch.py               # 扭矩控制
├── 02_demo_zero_calibration.py      # 归零校准
├── 03_demo_read_state.py            # 读取状态
├── 04_demo_move_gripper.py          # 夹爪控制
├── 05_demo_move_joint.py            # 关节空间运动
├── 06_demo_move_cartesian.py        # 笛卡尔空间运动
├── 07_demo_forward_kinematics.py    # 正向运动学
├── 08_demo_inverse_kinematics.py    # 逆向运动学
└── 09_demo_drag_teaching.py         # 拖动示教
```

## 📜 例程列表

### 0. `00_demo_read_version.py`
**读取机械臂固件版本号**
- 在使用前务必先运行此脚本检测固件版本
- 如果版本号为 6.x.x，SDK将自动使用新固件模式（直接控制）
- 如果版本号低于 6.x.x，SDK将使用旧固件模式（插值控制）
- 如果显示超时或没有版本号输出，可能需要调整波特率

**使用方式：**
```bash
python 00_demo_read_version.py
```

---

### 1. `01_torque_switch.py`
演示如何关闭/开启扭矩，用于进入或退出拖动示教模式。

**功能说明：**
- `command='off'`：关闭扭矩，进入拖动示教模式
- `command='on'`：开启扭矩，退出示教模式，恢复正常控制

**注意事项：**
- 关闭扭矩后机械臂可以手动拖动，但要注意安全防止机械臂砸落损坏
- 示教完成后务必重新开启扭矩

**使用方式：**
```bash
python 01_torque_switch.py
```

---

### 2. `02_demo_zero_calibration.py`
执行归零流程：关闭扭矩 → 手动拖动 → 开启扭矩 → 设置零点。

**适用场景：**
- 机械臂首次使用或长时间未使用后
- 关节角度出现偏差时
- 需要重新建立零点参考时

**使用方式：**
```bash
python 02_demo_zero_calibration.py
```

---

### 3. `03_demo_read_state.py`
打印机械臂状态：关节角度、末端位置、姿态和夹爪角度。支持持续输出。

**参数说明：**
- `continuous=True`：持续输出状态信息（按 Ctrl+C 停止）
- `output_format='deg'`：输出角度单位（'deg' 或 'rad'）

**使用场景：**
- 调试和故障排查
- 实时监控机械臂状态
- 验证控制指令执行效果

**使用方式：**
```bash
python 03_demo_read_state.py
```

---

### 4. `04_demo_move_gripper.py`
夹爪控制：演示开/关与角度设置三种模式。

**参数说明：**
- `command='open'`：完全打开夹爪
- `command='close'`：完全关闭夹爪
- `value=50`：设置夹爪到指定开合度（0-100）

**调整建议：**
- 根据夹持物体大小调整开合度
- 不同材质的物体可能需要不同的夹持力度

**使用方式：**
```bash
python 04_demo_move_gripper.py
```

---

### 5. `05_demo_move_joint.py`
使用关节空间运动控制机械臂移动到设定角度。

**API 方法：**
- 新固件（6.x）：`set_joint_target(target_joints, joint_format='rad')`
- 旧固件（<6.x）：`set_joint_target_interplotation(target_joints, joint_format='rad', speed_factor=1.0, T_default=4.0, n_steps_ref=200)`

**参数说明：**
- `joint_format='deg'`：输入角度单位（'deg' 或 'rad'）
- `target_joints`：目标关节角度列表
- `speed_factor=1.0`：速度倍率（>1更快，<1更慢，仅旧固件）
- `T_default=4.0`：默认插值总时长（秒，仅旧固件）
- `n_steps_ref=200`：参考插值步数（仅旧固件）
- `visualize=False`：是否可视化轨迹（仅旧固件）

**调整建议：**
- 根据运动距离调整 `T_default`：短距离用2-3秒，长距离用5-8秒
- 精细操作时降低 `speed_factor` 到0.5-0.8
- 快速运动时可提高 `speed_factor` 到1.2-1.5

**使用方式：**
```bash
python 05_demo_move_joint.py
```

---

### 6. `06_demo_move_cartesian.py`
使用笛卡尔空间运动控制机械臂末端执行器移动到目标位姿。

**API 方法：**
- `move_cartesian_linear(target_pose, duration=2.0, num_points=50, ik_method='dls', visualize=False)`

**参数说明：**
- `target_pose`：目标位姿 `[x, y, z, qx, qy, qz, qw]`（位置+四元数）
- `duration`：轨迹执行时长（秒）
- `num_points`：轨迹点数
- `ik_method`：逆运动学求解方法（'dls', 'pinv', 'transpose'）
- `visualize`：是否可视化轨迹

**调整建议：**
- 简单路径用 duration=2-3秒，num_points=30-50
- 复杂路径用 duration=5-10秒，num_points=80-100
- IK求解失败时可尝试不同的 `ik_method`

**使用方式：**
```bash
python 06_demo_move_cartesian.py
```

---

### 7. `07_demo_forward_kinematics.py`
演示正向运动学计算：给定关节角度，计算末端执行器位姿。

**功能：**
- 使用 RoboCore 库的 `forward_kinematics` 函数
- 支持 NumPy 和 PyTorch 后端
- 返回变换矩阵、位置和旋转矩阵

**使用方式：**
```bash
python 07_demo_forward_kinematics.py
```

---

### 8. `08_demo_inverse_kinematics.py`
演示逆向运动学求解：给定目标位姿，计算关节角度。

**API 方法：**
- `set_pose_target(target_pose, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_factor=1.0, execute=True)`

**参数说明：**
- `target_pose`：目标位姿 `[x, y, z, qx, qy, qz, qw]`
- `backend`：计算后端（'numpy' 或 'torch'）
- `method`：IK求解方法（'dls', 'pinv', 'transpose'）
- `tolerance`：位置和姿态容差
- `max_iters`：最大迭代次数
- `multi_start`：多起点尝试次数（0表示禁用）
- `use_random_init`：使用随机初始猜测
- `speed_factor`：运动速度倍率
- `execute`：是否执行运动

**调整建议：**
- 求解失败时尝试 `multi_start=5-10`
- 精确任务时降低 `tolerance` 到 1e-5
- 使用 `use_random_init=True` 探索不同配置

**使用方式：**
```bash
python 08_demo_inverse_kinematics.py
```

---

### 9. `09_demo_drag_teaching.py`
拖动示教演示：手动拖动机械臂记录轨迹，然后回放。

**功能：**
- 关闭扭矩进入示教模式
- 手动拖动机械臂
- 记录关键点或连续轨迹
- 回放记录的轨迹

**使用方式：**
```bash
python 09_demo_drag_teaching.py
```

---

## ⚙️ 常见参数调整场景

### 运动控制参数调整：
- **速度过快**：降低 `speed_factor` 或 `speed_deg_s`
- **运动不平滑**：增加 `num_points` 或 `n_steps_ref`
- **精度不够**：降低 `tolerance`，增加 `max_iters`
- **响应过慢**：降低 `duration` 或 `T_default`

### IK 求解参数调整：
- **求解失败**：尝试不同 `method`，增加 `multi_start`
- **局部最优**：使用 `use_random_init=True`
- **精度要求高**：降低 `tolerance` 到 1e-5 或更小
- **速度要求高**：使用 `method='pinv'` 或 `method='transpose'`

---

## 🔧 固件版本说明

SDK 自动检测固件版本并选择合适的控制模式：

### 新固件（6.x.x）：
- 使用 `set_joint_target()` 直接设置关节目标
- 支持 `set_speed()` 设置运动速度
- 更快的响应速度
- 更平滑的运动轨迹

### 旧固件（<6.x.x）：
- 使用 `set_joint_target_interplotation()` 插值控制
- 需要手动调整 `speed_factor` 和 `T_default`
- SDK 自动生成插值轨迹

---

这些脚本可作为功能测试或二次开发的起点。你也可以结合多个模块编写更复杂的应用逻辑。