import time
from typing import List, Optional

from alicia_d_sdk.hardware import ServoDriver
from alicia_d_sdk.utils.logger import logger
from alicia_d_sdk.execution.drag_teaching import record_waypoints_manual


class HardwareExecutor:
    def __init__(self, joint_controller: ServoDriver):
        self.joint_controller = joint_controller
        self.delay = 0.02

    def execute(self,
                joint_traj: List[List[float]],
                interaction: bool = False,
                speed_deg_s: int = 10):
        """
        Execute joint trajectory.

        :param joint_traj: Joint trajectory as a list of joint angle lists
        :param interaction: Whether to wait for user confirmation
        :param speed_deg_s: Joint speed in degrees per second (5-400, default 10)
        """

        if interaction:
            logger.module("[executor]按下回车执行轨迹，按下 q 取消：")
            usr_input = input()

            if usr_input.lower() == 'q':
                logger.info("[executor]取消执行轨迹")
                return False

        for idx, point in enumerate(joint_traj):
            # 仅根据关节轨迹执行运动，不再复现夹爪轨迹
            self.joint_controller.set_joint_and_gripper(
                joint_angles=point,
                gripper_value=None,
                speed_deg_s=speed_deg_s,
            )
            time.sleep(self.delay)


class CartesianWaypointPlanner:
    """Cartesian waypoint planner for recording and executing paths"""

    def __init__(self, robot):
        """
        Initialize Cartesian waypoint planner.

        :param robot: Robot API instance
        """
        self.robot = robot

    def get_current_waypoint(self) -> Optional[List[float]]:
        """
        Get current waypoint.

        :return: [x, y, z, qx, qy, qz, qw] or None
        """
        pose = self.robot.get_pose()
        if pose is None:
            return None

        pos = pose['position'].tolist()
        quat = pose['quaternion_xyzw'].tolist()
        return pos + quat

    def record_teaching_waypoints(self) -> List[List[float]]:
        """
        Record teaching waypoints.

        :return: Waypoints as [x, y, z, qx, qy, qz, qw]
        """
        logger.info("=== 教学模式：手动记录路径点 ===")

        # 定义状态获取函数（获取笛卡尔位姿）
        def get_cartesian_state(_):
            return self.get_current_waypoint()

        # 定义日志格式化函数
        def format_waypoint(count, waypoint):
            if waypoint and len(waypoint) >= 3:
                return (f"[记录] 路径点 {count}: "
                        f"位置={[round(p, 4) for p in waypoint[:3]]}")
            return f"[记录] 路径点 {count}"

        # 使用共享函数记录路径点
        waypoints = record_waypoints_manual(
            controller=self.robot,
            get_state_fn=get_cartesian_state,
            format_fn=format_waypoint
        )

        return waypoints

    def execute_trajectory(self,
                           waypoints: List[List[float]],
                           speed_deg_s: int = 10,
                           move_duration: float = 3.0,
                           num_points: int = 150,
                           ik_method: str = 'dls',
                           step_by_step: bool = False,
                           step_delay: float = 0.2):
        """
        Execute trajectory from waypoints.

        :param waypoints: Waypoints as [x, y, z, qx, qy, qz, qw]
        :param speed_deg_s: Motion speed in degrees per second (5-400, default 10)
        :param move_duration: Movement time per waypoint in seconds
        :param num_points: Interpolation points per segment
        :param ik_method: IK method
        :param step_by_step: Whether to execute step by step
        :param step_delay: Delay between steps in seconds
        """
        if not waypoints:
            logger.error("没有路径点可执行")
            return

        logger.info("\n=== 执行笛卡尔轨迹 ===")
        logger.info(f"共 {len(waypoints)} 个路径点")
        logger.info(f"{'逐步' if step_by_step else '连续'}执行模式...")

        for i, waypoint in enumerate(waypoints):
            logger.info(f"执行路径点 {i+1}/{len(waypoints)}...")

            # 分离位姿和夹爪
            pose = waypoint[:7]  # [x, y, z, qx, qy, qz, qw]

            # 执行笛卡尔运动
            self.robot.move_cartesian_linear(
                target_pose=pose,
                speed_deg_s=speed_deg_s,
                duration=move_duration,
                num_points=num_points,
                ik_method=ik_method
            )

            # 设置夹爪（使用统一接口，仅控制夹爪）
            # self.robot.set_robot_target(gripper_value=gripper, speed_deg_s=speed_deg_s, wait_for_completion=False)
            # time.sleep(step_delay)

            # 逐步执行模式下等待用户确认
            if step_by_step and i < len(waypoints) - 1:
                input("按 Enter 继续下一个路径点...")

        logger.info("✓ 轨迹执行完成!")
