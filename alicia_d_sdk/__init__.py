"""
Alicia-D SDK v6.0.0 - 与 RoboCore 桥接

架构层次：
- 用户层: SynriaRobotAPI (统一用户接口)
- 规划层: 使用 RoboCore 轨迹规划功能
- 控制层: MotionController (运动控制)
- 执行层: HardwareExecutor (硬件执行)
- 硬件层: ServoDriver (底层驱动)
- 运动学层: 使用 RoboCore 运动学功能 (FK/IK/Jacobian)

Bridge with RoboCore:
- robocore.kinematics: 提供 FK/IK/Jacobian 计算
- robocore.planning: 提供轨迹规划功能
- robocore.modeling: 提供 RobotModel
"""

from alicia_d_sdk.api import SynriaRobotAPI
from alicia_d_sdk.hardware import ServoDriver
from alicia_d_sdk.execution import HardwareExecutor

# Import from RoboCore for kinematics and modeling
from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from robocore.planning import (
    cubic_polynomial_trajectory,
    quintic_polynomial_trajectory,
    linear_joint_trajectory,
    linear_cartesian_trajectory,
    trapezoidal_velocity_profile
)

__version__ = "6.1.0"
__author__ = "Synria Robotics"
__description__ = "Alicia-D机械臂SDK v6.1.0 - Bridged with RoboCore"

# Re-export RoboCore components for convenience
__all__ = [
    # Core API
    "SynriaRobotAPI",
    "create_robot",
    "create_session",

    # Hardware Layer
    "ServoDriver",

    # Execution Layer
    "HardwareExecutor",

    # RoboCore - Modeling
    "RobotModel",

    # RoboCore - Kinematics
    "forward_kinematics",
    "inverse_kinematics",
    "jacobian",

    # RoboCore - Planning
    "cubic_polynomial_trajectory",
    "quintic_polynomial_trajectory",
    "linear_joint_trajectory",
    "linear_cartesian_trajectory",
    "trapezoidal_velocity_profile",
]


def create_robot(
    port: str = "",
    robot_version: str = "v5_6",
    gripper_type: str = "50mm",
    debug_mode: bool = False,
) -> SynriaRobotAPI:
    """
    Create robot instance.

    :param port: Serial port
    :param robot_version: Robot version (e.g., "v5_6", "v5_4")
    :param gripper_type: Gripper type (e.g., "50mm", "30mm")
    :param debug_mode: Debug mode
    :return: SynriaRobotAPI instance
    """
    # 创建硬件层
    servo_driver = ServoDriver(port=port, debug_mode=debug_mode)

    # 创建运动学层 (使用 RoboCore)
    try:
        from synriard import get_model_path
        urdf_path = get_model_path("Alicia_D", version=robot_version, variant=f"gripper_{gripper_type}")
        end_link = 'tool0'
        robot_model = RobotModel(str(urdf_path), end_link=end_link)
    except ImportError:
        print("Warning: synriard not found, using default URDF path")
        # Fallback to default path
        from pathlib import Path
        default_urdf = Path(__file__).parent.parent / "assets" / "robot" / "urdf" / f"Alicia-D_{robot_version}" / "alicia_duo_with_gripper.urdf"
        if default_urdf.exists():
            robot_model = RobotModel(str(default_urdf), end_link='tool0')
        else:
            raise FileNotFoundError(f"Cannot find URDF file for {robot_version}")

    # 创建用户层 (不再需要 ik_controller，直接使用 robocore.kinematics 函数)
    robot = SynriaRobotAPI(
        servo_driver=servo_driver,
        robot_model=robot_model
    )

    return robot
