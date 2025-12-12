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
├── 09_demo_drag_teaching.py         # 拖动示教
└── 10_demo_sparkvis.py              # SparkVis UI 双向同步
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

### 6. `06_demo_move_cartesian.py`
多点笛卡尔轨迹规划。支持手动拖动示教记录多个路径点，然后执行连续或逐步的轨迹运动。

**参数说明：**
- `--port`: 串口端口（可选）
- `--speed_deg_s`: 关节运动速度（度/秒），默认 10
- `--move_duration`: 每个路径点的移动时间（秒），默认 3.0
- `--num_points`: 轨迹插值点数，默认 200
- `--ik_method`: 逆运动学求解方法，可选 `dls`、`pinv`、`lm`，默认 `dls`
- `--visualize`: 启用轨迹可视化

**功能说明：**
- 先移动到初始位置（home）
- 手动拖动记录路径点（使用 `CartesianWaypointPlanner`）
- 选择执行模式：连续执行或逐步执行
- 使用逆运动学求解并执行轨迹

**使用方式：**
```bash
python 06_demo_move_cartesian.py --speed_deg_s 10 --move_duration 3.0 --num_points 200 --ik_method dls
```

---

### 7. `07_demo_forward_kinematics.py`
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

### 8. `08_demo_inverse_kinematics.py`
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

### 9. `09_demo_drag_teaching.py`
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

### 10. `10_demo_sparkvis.py`
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
python 10_demo_sparkvis.py

# 启用机器人状态同步并记录数据
python 10_demo_sparkvis.py --enable-robot-sync --output-file data.csv --log-source both
```

**注意**: 使用此功能需要同时运行：
1. SparkVis 后端服务器：`cd SparkVis && python backend_server.py`
2. SparkVis Web 服务器：`cd SparkVis && python -m http.server 8080`
3. 此机器人桥接脚本：`python 10_demo_sparkvis.py`
4. 在浏览器中访问：`http://localhost:8080`

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
