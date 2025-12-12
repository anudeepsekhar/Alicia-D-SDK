# 安装指南

本指南将引导您完成 Alicia D SDK 的安装与运行环境配置。

---

##  环境要求

- Python 3.6 及以上版本（推荐 Python 3.8）
- 支持串口的计算机（USB 转串口芯片已集成在机械臂）

---

##  安装步骤

### 1. 克隆或下载项目
```bash
git clone https://github.com/Synria-Robotics/Alicia-D-SDK.git -b v6.1.0-dev
cd Alicia-D-SDK
```

### 2. 创建 Python 环境并安装依赖
使用 Conda 环境（推荐）：
```bash
conda create -n alicia python=3.8
conda activate alicia
```

安装依赖与 SDK：
```bash
# pip install -r requirements.txt
pip install -e .
```

---

##  快速开始

### 基本使用示例

```python
from alicia_d_sdk import create_robot

# 创建机器人实例（自动搜索串口）
robot = create_robot()

# 连接机械臂
if robot.connect():
    print("连接成功！")
    
    # 打印当前状态
    robot.print_state()
    
    # 移动到初始位置
    robot.set_home()
    
    # 断开连接
    robot.disconnect()
else:
    print("连接失败，请检查串口")
```

### 手动指定串口

如果自动连接失败，可手动指定串口：

```python
# Linux
robot = create_robot(port="/dev/ttyACM0")

# Windows
robot = create_robot(port="COM3")
```

---

##  连接硬件

- 将机械臂通过 USB 连接到计算机
- 确保电源打开
- 系统应自动识别串口设备，如：
  - Linux: `/dev/ttyACM0`, `/dev/ttyACM1` ...
  - Windows: `COM3`, `COM4` ...

---

##  示例验证

执行以下命令测试连接和读取状态：
```bash
cd examples
python3 00_demo_read_version.py   # 读取固件版本
python3 03_demo_read_state.py     # 读取机械臂状态
```

若连接成功，终端将输出固件版本、当前关节角度、末端位姿与夹爪状态。

---

## ⚠️ 故障排查

### 找不到串口/连接失败
- 检查 USB 线与电源
- Linux 用户需确保在 `dialout` 用户组中：
  ```bash
  sudo usermod -a -G dialout $USER
  # 然后重新登录
  ```
- 运行 `00_demo_read_version.py` 检测固件版本

### 波特率问题
- 新固件（6.x.x）：默认波特率 1000000
- 旧固件（<6.x.x）：可能需要使用波特率 921600

手动指定波特率：
```python
robot = create_robot(port="/dev/ttyACM0")
```

### 权限错误 (Permission denied)
- 可尝试以 sudo 运行或检查用户串口权限
- Linux: 确保用户在 dialout 组中
- 检查串口是否被其他程序占用

### 固件版本检测失败
- 多次运行 `00_demo_read_version.py`
- 检查串口连接是否稳定

---

## 📦 依赖包说明

主要依赖：
- `pyserial`: 串口通信
- `numpy`: 数值计算
- `pycrc`: CRC校验
- `robocore`: 运动学和轨迹规划（自动安装）

---

安装成功后，即可通过 `SynriaRobotAPI` 接口控制机械臂完成各种动作。