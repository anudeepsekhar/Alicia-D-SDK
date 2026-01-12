# Alicia-D SDK


[English Version](README_EN.md) | [中文版](README.md) | [Official Taobao Store](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D Product Manual (CN)](https://docs.sparklingrobo.com/)


<p align="center"><img src="./imgs/Alicia_D_v5_5.jpg" width="500" /></p>





The **Alicia-D SDK** is a Python toolkit for controlling the "Alicia-D" series of 6-axis robotic arms (with gripper). Built on top of the `RoboCore` library, it provides functionalities to control the arm's movement, operate the gripper, and read posture and status data via serial communication.




# RoboCore: Unified High-Throughput Robotics Library



<p align="center"><img src="./imgs/logo.jpeg" width="400" /></p>

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

**Developed by [Synria Robotics Co., Ltd.](https://synriarobotics.ai)** 🤖

---

## ✨ Features

| Module | Functionality | Status |
|--------|---------------|--------|
| **Modeling** | URDF/MJCF parsing, Robot model abstraction | ✅ Stable |
| **Forward Kinematics** | NumPy/PyTorch backends, Batch processing | ✅ Stable |
| **Inverse Kinematics** | DLS/Pinv/Transpose methods, Multi-start | ✅ Stable |
| **Jacobian** | Analytic/Numeric/Autograd methods | ✅ Stable |
| **Transform** | SE(3)/SO(3) operations, Conversions | ✅ Stable |
| **Analysis** | Workspace/Singularity analysis | ✅ Beta |
| **Planning** | Trajectory generation | 🚧 Alpha |
| **Visualization** | Kinematic tree display | ✅ Stable |
| **Configuration** | YAML-based config management | ✅ Stable |




## Key Features

*   **Joint Control**: Supports setting and reading the angles of the six joints, with smooth interpolation for execution.
*   **End-Effector Trajectory**: Cartesian end-effector pose-based trajectory planning and execution.
*   **Gripper Control**: Supports precise angle control or one-click open/close.
*   **Torque Control**: Enable or disable joint motor torque for free-drag teaching.
*   **Zero-Point Setting**: Set the current position as the new zero point.
*   **Status Reading**: Real-time retrieval of joint angles, gripper angle, and end-effector pose.
*   **Automatic Serial Connection**: Automatically searches for serial ports or allows manual specification.
*   **Teaching Mode**: Record pose points by dragging and execute the trajectory.
*   **Smart Logging System**: Supports log level filtering to control console output verbosity.
*   **RoboCore Integration**: Integrated high-performance kinematics and trajectory planning library.

## Project Structure

```
├── alicia_d_sdk
│   ├── api
│   │   └── synria_robot_api.py      # User-level API
│   ├── execution
│   │   └── hardware_executor.py     # Execution layer
│   ├── hardware
│   │   ├── serial_comm.py           # Serial communication
│   │   ├── data_parser.py           # Data parser
│   │   └── servo_driver.py          # Servo driver
│   ├── __init__.py
│   └── utils
│       ├── calculate.py             # Calculation utilities
│       └── logger/                  # Logging system
├── docs
│   ├── api_reference.md             # API reference
│   ├── examples.md                  # Examples guide
│   ├── installation.md              # Installation guide
│   └── logger_levels.md             # Logger levels
├── examples
│   ├── 00_demo_read_version.py      # Read firmware version
│   ├── 01_torque_switch.py          # Torque switch
│   ├── 02_demo_zero_calibration.py  # Zero calibration
│   ├── 03_demo_read_state.py        # Read state
│   ├── 04_demo_move_gripper.py      # Gripper control
│   ├── 05_demo_move_joint.py        # Joint motion
│   ├── 06_demo_forward_kinematics.py  # Forward kinematics
│   ├── 07_demo_inverse_kinematics.py  # Inverse kinematics
│   ├── 08_demo_drag_teaching.py     # Drag teaching
│   ├── 09_demo_joint_traj.py        # Joint space trajectory planning
│   ├── 10_demo_cartesian_traj.py    # Cartesian space trajectory planning
│   ├── 11_demo_sparkvis.py          # SparkVis UI bidirectional sync
│   ├── 12_benchmark_read_joints.py  # Joint reading performance benchmark
│   └── 13_utmostFPS.py              # Maximum FPS test
```

## Installation

```bash
pip install alicia_d_sdk
```

## Quick Start

1.  Installation: Use `pip install alicia_d_sdk` or refer to the [Installation Guide](docs/installation.md)
2.  Run examples:
```bash
cd examples
python3 00_demo_read_version.py    # Read firmware version
python3 03_demo_read_state.py      # Read status
python3 04_demo_move_gripper.py    # Gripper control
python3 05_demo_move_joint.py      # Joint movement
```

## Documentation

**English Documentation:**
*   [Installation Guide](docs/installation_en.md)
*   [Examples Guide](docs/examples_en.md)
*   [API Reference](docs/api_reference_en.md)
*   [Migration Guide (v6.0.0 → v6.1.0)](docs/migration_guide_en.md)
*   [Logger Levels](docs/logger_levels_en.md)

**中文文档：**
*   [安装指南](docs/installation.md)
*   [示例说明](docs/examples.md)
*   [API 参考](docs/api_reference.md)
*   [迁移指南 (v6.0.0 → v6.1.0)](docs/migration_guide.md)
*   [日志级别](docs/logger_levels.md)