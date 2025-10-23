# Alicia-D SDK  


[English Version](README_EN.md) | [中文版](README.md) | [官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D 产品手册（中文）](https://docs.sparklingrobo.com/)

<p align="center"><img src="./imgs/Alicia_D_v5_5.jpg" width="500" /></p>



**Alicia-D SDK** 是一个用于控制【灵动 Alicia-D】系列六轴机械臂（带夹爪）的 Python 工具包。它基于 `RoboCore` 库构建，提供通过串口通信控制机械臂运动、操作夹爪、读取姿态与状态数据等功能。


## RoboCore: 统一化高性能机器人库



<p align="center"><img src="./imgs/logo.jpeg" width="400" /></p>


[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

**由 [Synria Robotics Co., Ltd.](https://synriarobotics.ai) 开发** 🤖


---

### ✨ 核心功能

| 模块 | 功能 | 状态 |
|---|---|---|
| **建模** | URDF/MJCF 解析, 机器人模型抽象 | ✅ Stable |
| **正向运动学** | 支持 NumPy/PyTorch 后端, 批处理 | ✅ Stable |
| **逆向运动学** | 支持 DLS/Pinv/Transpose 多种求解器, 多起点求解 | ✅ Stable |
| **雅可比矩阵** | 支持解析法/数值法/自动微分法 | ✅ Stable |
| **坐标变换** | SE(3)/SO(3) 刚体变换, 多种格式转换 | ✅ Stable |
| **运动学分析** | 工作空间/奇异点分析 | ✅ Beta |
| **轨迹规划** | 轨迹生成 | 🚧 Alpha |
| **可视化** | 运动学链可视化 | ✅ Stable |
| **配置管理** | 基于 YAML 的配置管理 | ✅ Stable |






## 主要特性

*   **关节控制**：支持设置与读取六个关节的角度，提供平滑插值执行。
*   **末端轨迹**：基于 Cartesian末端姿态轨迹。
*   **夹爪控制**：支持精确角度控制或一键开关。
*   **力矩控制**：开启或关闭关节电机扭矩，实现自由拖动（示教）。
*   **零点设置**：将当前位置设置为新的零点。
*   **状态读取**：实时获取关节角、夹爪角与末端姿态。
*   **自动串口连接**：自动搜索串口或手动指定。
*   **教学模式**：拖动记录姿态点并执行轨迹。
*   **智能日志系统**：支持日志级别过滤，可控制控制台输出详细程度。

## 项目结构

```
├── alicia_d_sdk
│   ├── api
│   ├── execution
│   ├── hardware
│   ├── __init__.py
│   └── utils
├── docs
│   ├── api_reference.md
│   ├── examples.md
│   ├── installation.md
│   └── logger_levels.md
├── examples
│   ├── 00_demo_read_version.py
│   ├── 01_torque_switch.py
│   ├── 02_demo_zero_calibration.py
│   ├── 03_demo_read_state.py
│   ├── 04_demo_move_gripper.py
│   ├── 05_demo_move_joint.py
│   ├── 06_demo_move_cartesian.py
│   ├── 07_demo_forward_kinematics.py
│   ├── 08_demo_inverse_kinematics.py
│   ├── 09_demo_drag_teaching.py
│   ├── 10_demo_sparkvis.py
```

## 快速开始

1.  安装：请参见 [安装指南](docs/installation.md)
2.  运行示例：
```bash
cd examples
python3 02_demo_read_state.py      # 读取状态
python3 03_demo_mvoe_gripper.py         # 夹爪控制
python3 04_demo_move_joint.py      # 关节移动
```

## 文档

*   [安装指南](docs/installation.md)
*   [示例说明](docs/examples.md)
*   [API 参考](docs/api_reference.md)
*   [日志级别](docs/logger_levels.md)