import time
from typing import List, Optional
import numpy as np

from alicia_d_sdk.hardware import ServoDriver
from alicia_d_sdk.utils.logger import logger
from alicia_d_sdk.utils.vislab import plot_joint_angles
from .drag_teaching import record_waypoints_manual

class HardwareExecutor:
    def __init__(self, joint_controller: ServoDriver):
        self.joint_controller = joint_controller
        self.delay = 0.02

    def execute(self, 
                joint_traj: List[List[float]], 
                visualize: bool = False,
                gripper_traj: List[float] = None,
                interaction: bool = False,
                firmware_new: bool = True
                ):
        """
        :param joint_traj: Joint trajectory as a list of joint angle lists
        :param visualize: Whether to plot before execution
        :param gripper_traj: Optional gripper values aligned with trajectory
        :param interaction: Whether to wait for user confirmation
        :return: True if executed, False if cancelled
        """
        traj_np = np.array(joint_traj)

        if visualize:
            plot_joint_angles(traj_np)

        if interaction:
            logger.module("[executor]按下回车执行轨迹，按下 q 取消：")
            usr_input = input()

            if usr_input.lower() == 'q':
                logger.info("[executor]取消执行轨迹")
                return False
        
        for idx, point in enumerate(joint_traj):
            self.joint_controller.set_joint_angles(point)
            if gripper_traj is not None and idx < len(gripper_traj):
                g = gripper_traj[idx]
                if g is not None:
                    self.joint_controller.set_gripper(g)
            time.sleep(self.delay)
        

class JointPlanner:
    def plan(
        self,
        start_angles: List[float],
        target_angles: List[float],
        steps: int
        ) -> List[List[float]]:
        """
        :param start_angles: Start joint angles
        :param target_angles: Target joint angles
        :param steps: Number of interpolation steps
        :return: Interpolated joint trajectory
        """
        traj = []
        for step in range(1, steps + 1):
            ratio = self.ease_in_out_cubic(step / steps)
            interp = [
                s + (t - s) * ratio
                for s, t in zip(start_angles, target_angles)
            ]
            traj.append(interp)

        return traj

    @staticmethod
    def ease_in_out_cubic(x: float) -> float:
        """
        :param x: Normalized time in [0, 1]
        :return: Interpolation coefficient
        """
        return 4 * x**3 if x < 0.5 else 1 - pow(-2 * x + 2, 3) / 2


class CartesianWaypointController:
    """Cartesian waypoint controller for recording and executing paths"""
    
    def __init__(self, robot):
        """
        :param robot: Robot API instance
        """
        self.robot = robot
    
    def get_current_waypoint(self) -> Optional[List[float]]:
        """
        :return: [x, y, z, qx, qy, qz, qw, gripper] or None
        """
        pose = self.robot.get_pose()
        if pose is None:
            return None
        
        pos = pose['position'].tolist()
        quat = pose['quaternion_xyzw'].tolist()
        gripper = self.robot.get_gripper() or 0.0
        
        return pos + quat + [gripper]
    
    def record_teaching_waypoints(self) -> List[List[float]]:
        """
        :return: Waypoints as [x, y, z, qx, qy, qz, qw, gripper]
        """
        logger.info("=== 教学模式：手动记录路径点 ===")
        
        # 定义状态获取函数（获取笛卡尔位姿）
        def get_cartesian_state(_):
            return self.get_current_waypoint()
        
        # 定义日志格式化函数
        def format_waypoint(count, waypoint):
            if waypoint and len(waypoint) >= 8:
                return (f"[记录] 路径点 {count}: "
                       f"位置={[round(p, 4) for p in waypoint[:3]]}, "
                       f"夹爪={waypoint[7]:.3f}")
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
                          move_duration: float = 3.0,
                          num_points: int = 150,
                          ik_method: str = 'dls',
                          visualize: bool = False,
                          step_by_step: bool = False,
                          step_delay: float = 0.2):
        """
        :param waypoints: Waypoints as [x, y, z, qx, qy, qz, qw, gripper]
        :param move_duration: Movement time per waypoint in seconds
        :param num_points: Interpolation points per segment
        :param ik_method: IK method
        :param visualize: Whether to visualize
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
            gripper = waypoint[7] if len(waypoint) > 7 else 0.0
            
            # 执行笛卡尔运动
            self.robot.move_cartesian_linear(
                target_pose=pose,
                duration=move_duration,
                num_points=num_points,
                ik_method=ik_method,
                visualize=visualize
            )
            
            # 设置夹爪
            self.robot.set_gripper_target(value=gripper, wait_for_completion=False)
            time.sleep(step_delay)
            
            # 逐步执行模式下等待用户确认
            if step_by_step and i < len(waypoints) - 1:
                input("按 Enter 继续下一个路径点...")
        
        logger.info("✓ 轨迹执行完成!")