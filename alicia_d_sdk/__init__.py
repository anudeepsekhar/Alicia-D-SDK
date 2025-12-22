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

# Import from RoboCore for kinematics and modeling
from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from synriard import get_model_path


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
    gripper_type: str = None,
    debug_mode: bool = False,
    base_link: str = "base_link",
    end_link: str = "tool0",
) -> SynriaRobotAPI:
    """
    Create robot instance.

    :param port: Serial port
    :param gripper_type: Gripper type:
        - explicit value such as "50mm" / "100mm" for user-defined configuration
        - None to auto-select from saved JSON (if available) or default to "50mm"
    :param debug_mode: Debug mode
    :param base_link: Base link name in the robot model (default 'base_link')
    :param end_link: End link name in the robot model (default 'tool0')
    :return: SynriaRobotAPI instance
    """
    servo_driver = ServoDriver(port=port, debug_mode=debug_mode)


    if gripper_type is not None:
        effective_gripper_type = gripper_type
    else:
        import json
        from pathlib import Path

        effective_gripper_type = "50mm"
        json_path = Path(__file__).parent / "api" / "gripper_type.json"
        if json_path.exists():
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                cached_type = data.get("type_name")
                if isinstance(cached_type, str) and cached_type:
                    effective_gripper_type = cached_type
            except Exception:
                pass


    urdf_path = get_model_path(
        "Alicia_D",
        version="v5_6",
        variant=f"gripper_{effective_gripper_type}",
    )
    robot_model = RobotModel(str(urdf_path), base_link=base_link, end_link=end_link)

    robot = SynriaRobotAPI(
        servo_driver=servo_driver,
        robot_model=robot_model,
    )

    return robot
