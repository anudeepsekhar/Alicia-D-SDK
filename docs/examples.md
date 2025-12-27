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
├── 06_demo_forward_kinematics.py    # 正向运动学
├── 07_demo_inverse_kinematics.py    # 逆向运动学
├── 08_demo_drag_teaching.py         # 拖动示教
├── 09_demo_joint_traj.py            # 关节空间轨迹规划
├── 10_demo_cartesian_traj.py        # 笛卡尔空间轨迹规划
├── 11_demo_sparkvis.py              # SparkVis UI 双向同步
├── 12_benchmark_read_joints.py      # 关节读取性能测试
└── 13_utmostFPS.py                  # 最大帧率测试
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
切换机械臂所有关节的力矩状态（上电/掉电）。调电后可以自由拖动机械臂。

**参数说明：**
- `--port`: 串口端口（可选）

**功能说明：**
- 先关力矩后开力矩
- 关闭扭矩后机械臂可以手动拖动
- 示教完成后重新开启扭矩

**注意事项：**
- 关闭扭矩前请手动托住机械臂以免其突然掉落
- 示教完成后务必重新开启扭矩

**使用方式：**
```bash
python 01_torque_switch.py
```

---

### 2. `02_demo_zero_calibration.py`
将机械臂当前位置设置为新的零点。**此操作不可逆，请谨慎使用。**

**参数说明：**
- `--port`: 串口端口（可选）

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
持续读取并打印机械臂的关节角度、末端姿态和夹爪状态。支持单次或持续打印模式。

**参数说明：**
- `--port`: 串口端口（可选）
- `--robot_version`: 机器人版本，默认 `v5_6`
- `--gripper_type`: 夹爪型号，默认 `50mm`
- `--format`: 角度显示格式，可选 `rad`（弧度）或 `deg`（角度），默认 `deg`
- `--single`: 单次打印状态，默认持续打印

**使用场景：**
- 调试和故障排查
- 实时监控机械臂状态
- 验证控制指令执行效果

**使用方式：**
```bash
# 持续打印状态（按 Ctrl+C 停止）
python 03_demo_read_state.py --robot_version v5_6 --gripper_type 50mm

# 单次打印状态
python 03_demo_read_state.py --single
```

---

### 4. `04_demo_move_gripper.py`
夹爪控制：控制夹爪张开或闭合到指定角度。夹爪值范围为 0-1000（0为完全闭合，1000为完全张开）。

**参数说明：**
- `--port`: 串口端口（可选）

**功能说明：**
- 演示自动执行：完全张开 → 完全闭合 → 半开
- 使用 `set_robot_target(gripper_value=...)` 控制夹爪
- 支持等待夹爪运动完成

**使用方式：**
```bash
python 04_demo_move_gripper.py
```

---

### 5. `05_demo_move_joint.py`
使用关节空间运动控制机械臂移动到设定角度。支持度数和弧度输入，自动进行关节角度插值。

**参数说明：**
- `--port`: 串口端口（可选）
- `--speed_deg_s`: 关节运动速度（度/秒），默认 10，范围 5-400

**功能说明：**
- 演示自动执行：回零 → 移动到目标角度 → 回零
- 使用统一的关节和夹爪目标接口 `set_robot_target()`
- 支持 `wait_for_completion=True` 等待运动完成

**使用方式：**
```bash
python 05_demo_move_joint.py --speed_deg_s 10
```

---

---

### 6. `06_demo_forward_kinematics.py`
正运动学求解。根据当前关节角度计算末端执行器的位姿，显示位置、旋转矩阵、欧拉角、四元数等多种表示形式。

**参数说明：**
- `--port`: 串口端口（可选）

**功能说明：**
- 使用 RoboCore 库的机器人模型
- 显示机器人模型信息和运动学链
- 返回并显示：位置、旋转矩阵、欧拉角（弧度/角度）、四元数、齐次变换矩阵
- 注意：四元数 q 和 -q 表示相同的旋转

**使用方式：**
```bash
python 07_demo_forward_kinematics.py
```

---

### 7. `07_demo_inverse_kinematics.py`
演示逆向运动学求解：根据给定的末端目标位姿，计算并可选地执行关节角度。支持多种IK求解方法和多起点优化。

**参数说明：**
- `--port`: 串口端口（可选）
- `--speed_deg_s`: 关节运动速度（度/秒），默认 10
- `--end-pose`: 目标位姿（7个浮点数：px py pz qx qy qz qw），默认值已预设
- `--method`: IK方法，可选 `dls`（阻尼最小二乘）、`pinv`（伪逆）、`transpose`（雅可比转置），默认 `dls`
- `--max-iters`: 最大迭代次数，默认 100
- `--multi-start`: 多起点尝试次数，0表示禁用，建议 5-10
- `--use-random-init`: 使用随机初始值（默认使用当前关节角度）
- `--execute`: 执行移动到求解的位置

**功能说明：**
- 显示详细的求解结果（成功/失败、迭代次数、位置误差、姿态误差）
- 如果执行移动，会自动返回初始位置
- 支持多起点优化提高成功率

**使用方式：**
```bash
# 仅计算逆解，不执行动作
python 08_demo_inverse_kinematics.py

# 计算逆解并控制机械臂移动到目标位姿
python 08_demo_inverse_kinematics.py --execute --speed_deg_s 10

# 使用多起点优化提高成功率
python 08_demo_inverse_kinematics.py --execute
```

---

### 8. `08_demo_drag_teaching.py`
拖动示教演示：通过手动拖动机械臂来录制一系列轨迹点，并可以回放。支持手动插值、自动快速和仅回放三种模式。

**参数说明：**
- `--port`: 串口端口（可选）
- `--speed_deg_s`: 关节运动速度（度/秒），默认 10
- `--mode`: 拖动示教模式，可选 `manual`（手动插值）、`auto`（自动快速）或 `replay_only`（仅回放），默认 `auto`
- `--sample-hz`: 自动模式采样频率，默认 300.0 Hz
- `--save-motion`: 动作名称（录制模式：新动作名；回放模式：已有动作名）
- `--list-motions`: 列出所有可用的动作并退出

**功能说明：**
- 关闭扭矩进入示教模式
- 手动拖动机械臂记录轨迹
- 支持手动模式（记录关键点）和自动模式（连续采样）
- 回放记录的轨迹

**使用方式：**
```bash
# 列出所有可用的动作
python 09_demo_drag_teaching.py --list-motions

# 自动模式录制名为 "my_demo" 的轨迹
python 09_demo_drag_teaching.py --mode auto --save-motion my_demo

# 手动模式录制关键点轨迹
python 09_demo_drag_teaching.py --mode manual --save-motion key_points

# 回放已有轨迹 "my_demo"
python 09_demo_drag_teaching.py --mode replay_only --save-motion my_demo
```

---

### 9. `09_demo_joint_traj.py`
**关节空间轨迹规划与执行**

演示如何使用关节空间轨迹规划生成平滑的关节轨迹，并通过多个路径点执行。

**参数说明：**
- `--port`: 串口端口（可选）
- `--gripper_type`: 夹爪类型，默认 `50mm`
- `--base_link`: 基座链路名称，默认 `base_link`
- `--end_link`: 末端执行器链路名称，默认 `tool0`
- `--no-record`: 禁用记录模式
- `--save-file`: 保存记录的路径点文件路径
- `--waypoints-file`: 从JSON文件加载路径点
- `--num-waypoints`: 随机生成的路径点数量，默认 6
- `--planner`: 规划器类型，可选 `b_spline` 或 `multi_segment`，默认 `b_spline`
- `--duration`: 轨迹持续时间（B-Spline），默认 2.0 秒
- `--num-points`: 轨迹点数（B-Spline），默认 800
- `--bspline-degree`: B-Spline 度数，可选 3 或 5，默认 5
- `--speed-deg-s`: 关节运动速度（度/秒），默认 20
- `--plot`: 禁用轨迹可视化

**功能说明：**
- 支持手动记录路径点或从文件加载
- 使用 B-Spline 或 Multi-Segment 规划器生成平滑轨迹
- 支持夹爪轨迹插值
- 可视化轨迹（可选）

**使用方式：**
```bash
# 使用默认参数运行
python 09_demo_joint_traj.py

# 从文件加载路径点
python 09_demo_joint_traj.py --waypoints-file waypoints.json

# 使用 Multi-Segment 规划器
python 09_demo_joint_traj.py --planner multi_segment
```

---

### 10. `10_demo_cartesian_traj.py`
**笛卡尔空间轨迹规划与执行**

演示如何使用笛卡尔空间样条轨迹规划生成平滑的末端执行器轨迹，并通过逆运动学求解执行。

**参数说明：**
- `--port`: 串口端口（可选）
- `--gripper_type`: 夹爪类型，默认 `50mm`
- `--base_link`: 基座链路名称，默认 `base_link`
- `--end_link`: 末端执行器链路名称，默认 `tool0`
- `--no-record`: 禁用记录模式
- `--save-file`: 保存记录的路径点文件路径
- `--waypoints-file`: 从JSON文件加载路径点
- `--num-waypoints`: 随机生成的路径点数量，默认 2
- `--duration`: 轨迹持续时间（秒），默认 1.0
- `--num-points`: 轨迹点数，默认 10
- `--method`: IK方法，可选 `dls`、`pinv`、`transpose`，默认 `dls`
- `--max-iters`: 最大IK迭代次数，默认 100
- `--pos-tol`: 位置容差（米），默认 1e-2
- `--ori-tol`: 姿态容差（弧度），默认 1e-2
- `--num-inits`: 初始猜测数量，默认 5
- `--init-strategy`: 初始猜测策略，默认 `current`
- `--speed-deg-s`: 关节运动速度（度/秒），默认 20
- `--plot`: 禁用轨迹可视化

**功能说明：**
- 支持手动记录笛卡尔路径点或从文件加载
- 使用样条曲线生成平滑的笛卡尔轨迹
- 批量求解逆运动学（确保连续性）
- 验证路径点是否被准确通过
- 可视化轨迹和IK结果（可选）

**使用方式：**
```bash
# 使用默认参数运行
python 10_demo_cartesian_traj.py

# 从文件加载路径点
python 10_demo_cartesian_traj.py --waypoints-file cartesian_waypoints.json

# 增加轨迹点数以提高平滑度
python 10_demo_cartesian_traj.py --num-points 100
```

---

### 11. `11_demo_sparkvis.py`
SparkVis UI 双向同步和数据记录。启动 WebSocket 服务器，实现 UI ↔ 机器人双向同步，支持数据记录到 CSV 文件。

**参数说明：**
- `--port`: 串口端口（可选）
- `--host`: WebSocket 主机，默认 `localhost`
- `--websocket-port`: WebSocket 端口，默认 8765
- `--output-file`: CSV 输出路径（留空不记录）
- `--log-source`: 记录来源，可选 `ui`（UI指令）、`robot`（机器人状态）或 `both`（二者），默认 `ui`
- `--enable-robot-sync`: 启用机器人→UI 状态同步
- `--robot-sync-rate`: 机器人状态广播频率（Hz），默认 50.0

**功能说明：**
- UI → Robot: 接收 joint_update 并直接发送到真实机器人
- Robot → UI: 定期广播当前机器人状态到 UI（可切换）
- 数据记录: 将 UI 命令的关节数据记录到 CSV（可选）

**使用方式：**
```bash
# 基本使用（需要先启动 SparkVis 后端和 Web 服务器）
python 11_demo_sparkvis.py

# 启用机器人状态同步并记录数据
python 11_demo_sparkvis.py --enable-robot-sync --output-file data.csv --log-source both
```

**注意**: 使用此功能需要同时运行：
1. SparkVis 后端服务器：`cd SparkVis && python backend_server.py`
2. SparkVis Web 服务器：`cd SparkVis && python -m http.server 8080`
3. 此机器人桥接脚本：`python 11_demo_sparkvis.py`
4. 在浏览器中访问：`http://localhost:8080`

---

### 12. `12_benchmark_read_joints.py`
**关节读取性能测试**

测试关节角度读取的API调用频率和实际数据更新频率。

**参数说明：**
- `--port`: 串口端口（可选）
- `--gripper_type`: 夹爪类型，默认 `50mm`
- `--duration`: 测试持续时间（秒），默认 5.0
- `--fast`: 启用快速模式（设置更新间隔为1ms）

**功能说明：**
- 测试API读取频率（从Python内存读取缓存数据的速度）
- 测试数据更新频率（从机器人实际接收新数据的速率）
- 显示串口统计信息（处理的帧数、丢弃的帧数等）

**使用方式：**
```bash
# 基本测试（5秒）
python 12_benchmark_read_joints.py

# 快速模式测试
python 12_benchmark_read_joints.py --fast --duration 10
```

---

### 13. `13_utmostFPS.py`
**最大帧率测试**

测试串口通信的最大发送和接收帧率。

**功能说明：**
- 测试串口通信的极限性能
- 显示发送帧率、接收帧率和同步率
- 适用于性能优化和硬件测试

**注意**: 此脚本包含硬编码的串口路径，需要根据实际环境修改。

---

## ⚙️ 常见参数调整场景

### 运动控制参数调整：

*新固件（6.1.0及以上）*

- **速度过快**：降低 `speed_deg_s`（范围：5-400度/秒）
- **运动不流畅**：增加 `num_points`（轨迹插值点数）
- **移动时间过长**：减少 `move_duration`（每个路径点的移动时间）

### IK 求解参数调整：
- **求解失败**：尝试不同 `method`（`dls`、`pinv`、`transpose`），增加 `multi_start`（建议 5-10）
- **局部最优**：使用 `use_random_init=True` 或增加 `multi_start`
- **精度要求高**：降低 `tolerance` 到 1e-5 或更小，增加 `max_iters`
- **速度要求高**：使用 `method='pinv'` 或 `method='transpose'`

### 拖动示教参数调整：
- **采样频率**：调整 `--sample-hz`（默认 300.0 Hz），更高频率记录更详细但文件更大
- **模式选择**：`manual` 模式适合记录关键点，`auto` 模式适合连续轨迹

---
