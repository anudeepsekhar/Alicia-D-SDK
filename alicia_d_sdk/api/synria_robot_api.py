"""
SynriaRobotAPI - 用户层API

职责：
- 提供简洁统一的用户接口
- 高级运动命令封装
- 状态查询接口
- 系统控制功能
- 参数验证和错误处理
"""

import time
from typing import List, Optional, Dict, Union
import numpy as np

# Import from robocore for kinematics and planning
from robocore.kinematics import inverse_kinematics
from robocore.modeling import RobotModel
from robocore.transform import make_transform, quaternion_to_matrix
from robocore.kinematics import forward_kinematics
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from robocore.planning.trajectory import (
    cubic_polynomial_trajectory,
    quintic_polynomial_trajectory,
    linear_joint_trajectory,
    linear_cartesian_trajectory,
    circular_cartesian_trajectory,
    cartesian_waypoint_trajectory
)
from ..hardware import ServoDriver
from ..execution import HardwareExecutor, JointPlanner
from ..utils.logger import logger
from robocore.utils.control_utils import compute_steps_and_delay, validate_joint_list, check_and_clip_joint_limits


class SynriaRobotAPI:
    """Synria机械臂API - 提供统一的用户接口"""
    
    def __init__(self,
                 servo_driver: ServoDriver,
                 robot_model: RobotModel,
                 acceleration: int = 1):
        """
        :param servo_driver: Servo driver
        :param robot_model: Robot model from RoboCore
        :param acceleration: Acceleration
        """
        # 核心组件
        self.servo_driver = servo_driver
        self.robot_model = robot_model
        self.acceleration = acceleration
        
        # 创建各层组件
        self.hardware_executor = HardwareExecutor(servo_driver)
        
        # 默认参数
        self.home_angles = [0.0] * 6


    
    # ==================== 连接管理 ====================
    
    def connect(self) -> bool:
        """
        :return: True if connected
        """
        result = self.servo_driver.connect()
        if result:
            # Set acceleration to be maximum value
            self.set_acceleration(self.acceleration)
        return result
    
    def disconnect(self):
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()
    
    def is_connected(self) -> bool:
        """
        :return: True if connection is active
        """
        return self.servo_driver.serial_comm.is_connected()
    
    # ==================== Robot Control ====================

    def set_joint_target(self,
              target_joints: List[float],
              joint_format: str = 'rad',
              speed_factor: float = 1.0,
              T_default: float = 4.0,
              n_steps_ref: int = 200,
              visualize: bool = False) -> bool:
        """
        :param target_joints: Target joint angles
        :param joint_format: 'rad' or 'deg'
        :param speed_factor: Speed multiplier
        :param T_default: Default interpolation duration in seconds
        :param n_steps_ref: Reference interpolation steps
        :param visualize: Whether to visualize trajectory
        :return: True if motion started
        """
        logger.info("[moveJ] 开始执行关节空间插值移动")
        
        if target_joints is None:
            logger.error("[moveJ] 请提供 target_joints 参数")
            return False
        
        # 参数验证和转换
        joint_format = joint_format.lower()
        if joint_format not in ['rad', 'deg']:
            logger.error(f"[moveJ] 不支持的 joint_format: '{joint_format}'，应为 'rad' 或 'deg'")
            return False
        
        # 支持角度制输入
        if joint_format == 'deg':
            logger.info("[moveJ] 输入角度单位为 degree，将转换为 rad")
            target_joints = [a * np.pi / 180.0 for a in target_joints]
        
        # 验证关节列表
        validate_joint_list(target_joints)
        
        # 检查关节限位并修正
        target_joints, violations = check_and_clip_joint_limits(
            joints=target_joints,
            joint_limits=self.robot_model.joint_limits
        )
        
        for joint_name, original, clipped in violations:
            logger.warning(
                f"[moveJ] {joint_name} 超出限制：{original:.2f} -> 已截断为 {clipped:.2f}"
            )
        
        # 获取当前状态（直接从servo_driver获取，带重试机制）
        cur_angles = self.servo_driver.get_joint_angles()
        
        # 插值步数与延迟
        steps, delay = compute_steps_and_delay(
            speed_factor=speed_factor,
            T_default=T_default,
            n_steps_ref=n_steps_ref
        )
        planner = JointPlanner()
        # 规划轨迹
        joint_traj = planner.plan(
            start_angles=cur_angles,
            target_angles=target_joints,
            steps=steps
        )
        
        
        # 显示起始和目标角度
        if joint_format == 'deg':
            display_cur = [round(a * 180.0 / np.pi, 1) for a in cur_angles]
            display_target = [round(a * 180.0 / np.pi, 1) for a in target_joints]
            unit = "°"
        else:
            display_cur = [round(a, 3) for a in cur_angles]
            display_target = [round(a, 3) for a in target_joints]
            unit = "rad"
        
        # 日志输出
        logger.info(f"[moveJ] 起始角度 ({unit}): {display_cur}")
        logger.info(f"[moveJ] 目标角度 ({unit}): {display_target}")
        logger.info(f"[moveJ] 插值步数: {steps}，单步延迟: {delay:.3f}s，预计总耗时: {steps * delay:.1f}s")
        
        # 执行轨迹
        self.hardware_executor.delay = delay
        result = self.hardware_executor.execute(
            joint_traj=joint_traj,
            visualize=visualize
        )
        
        return result if result is not None else True
    

    # ==================== 夹爪控制 ====================
    
    def set_gripper_target(self,
                       command: Optional[str] = None,
                       value: Optional[float] = None,
                       wait_for_completion: bool = True,
                       timeout: float = 5.0,
                       tolerance: float = 1.0) -> bool:
        """
        :param command: 'open' or 'close'
        :param value: Gripper value in 0-100
        :param wait_for_completion: Whether to wait until reached
        :param timeout: Maximum wait time in seconds
        :param tolerance: Acceptable difference to target
        :return: True if successful
        """
        if command is not None and value is not None:
            logger.error("command 与 value 参数不可同时指定")
            return False
        
        if command is not None:
            if command == "open":
                value = 100.0  # 打开对应100
            elif command == "close":
                value = 0.0    # 关闭对应0
            else:
                logger.error("command 参数必须是 'open' 或 'close'")
                return False
        
        if value is None:
            logger.error("必须提供 command 或 value 参数")
            return False
        
        # Convert 0-100 value to radians for hardware layer
        # Hardware layer will multiply by RAD_TO_DEG to get back 0-100 range
        # value_rad = value * np.pi / 180.0
        
        # 发送夹爪命令
        success = self.servo_driver.set_gripper(value)
        if not success:
            logger.error("夹爪命令发送失败")
            return False
        
        if wait_for_completion:
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_gripper = self.get_gripper()
                if current_gripper is not None:
                    if abs(current_gripper - value) <= tolerance:
                        logger.info(f"夹爪已到达目标开合度: {value:.1f}")
                        return True
                time.sleep(0.1)
            
            logger.warning("夹爪运动等待超时")
            return False
        
        return True

    def set_home(self, speed_factor: float = 1):
        """
        :param speed_factor: Speed multiplier
        """
        self.set_joint_target(self.home_angles, joint_format='rad', speed_factor=speed_factor)



    def set_pose_target(self, 
                       target_pose: List[float], 
                       backend: str = 'numpy', 
                       method: str = 'dls', 
                       display: bool = True, 
                       tolerance: float = 1e-4, 
                       max_iters: int = 100,
                       multi_start: int = 0,
                       use_random_init: bool = False,
                       speed_factor: float = 1.0,
                       execute: bool = True) -> Dict:
        """
        :param target_pose: [x, y, z, qx, qy, qz, qw]
        :param backend: 'numpy' or 'torch'
        :param method: IK method: 'dls', 'pinv', 'transpose'
        :param display: Whether to display details
        :param tolerance: Position and orientation tolerance
        :param max_iters: Max number of iterations
        :param multi_start: Multi-start attempts (0 to disable)
        :param use_random_init: Use random initial guess
        :param speed_factor: Motion speed multiplier
        :param execute: Execute motion if True
        :return: Dict with success, q, iters, pos_err, ori_err, message
        """
        # Convert pose to transformation matrix
        position = np.array(target_pose[:3])
        quaternion = np.array(target_pose[3:])
        rotation_matrix = quaternion_to_matrix(quaternion)
        pose_matrix = make_transform(rotation_matrix, position)
        
        # Get initial guess
        if use_random_init:
            # Generate random initial guess within joint limits
            q_init = self._generate_random_q(scale=0.5)
            if display:
                logger.info("使用随机初始值")
        else:
            q_init = self.get_joints()
            if q_init is None:
                return {
                    'success': False,
                    'message': '无法获取当前关节角度',
                    'q': None
                }
        
        if display:
            logger.info(f"初始关节角度 (rad): {[f'{q:+.4f}' for q in q_init]}")
            logger.info(f"初始关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in q_init]}")
            logger.info(f"正在求解IK (方法: {method}, 最大迭代: {max_iters})...")
        
        # Solve inverse kinematics
        ik_result = inverse_kinematics(
            self.robot_model,
            pose_matrix,
            q_init,
            backend=backend,
            method=method,
            max_iters=max_iters,
            pos_tol=tolerance,
            ori_tol=tolerance,
            multi_start=multi_start,
            multi_noise=0.3,
            use_analytic_jacobian=True
        )
        
        if ik_result['success']:
            if display:
                logger.info("✓ IK 求解成功!")
                logger.info(f"  迭代次数: {ik_result['iters']}")
                logger.info(f"  位置误差: {ik_result['pos_err']:.6e} m")
                logger.info(f"  姿态误差: {ik_result['ori_err']:.6e} rad")
                logger.info(f"  关节角度 (rad): {[f'{q:+.4f}' for q in ik_result['q']]}")
                logger.info(f"  关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result['q']]}")
            
            # Execute motion if requested
            if execute:
                result = self.set_joint_target(
                    ik_result['q'], 
                    joint_format='rad',
                    speed_factor=speed_factor
                )
                ik_result['motion_executed'] = result
            else:
                ik_result['motion_executed'] = False
                if display:
                    logger.info("  (未执行运动，execute=False)")
            
            return ik_result
        else:
            error_msg = ik_result.get('message', '未知错误')
            if display:
                logger.error(f"✗ IK 求解失败: {error_msg}")
                logger.error(f"  迭代次数: {ik_result.get('iters', 'N/A')}")
                logger.error(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m")
                logger.error(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad")
            
            return ik_result
    


    def get_joints(self) -> Optional[List[float]]:
        """
        :return: Current joint angles in radians
        """
        joint_state = self.servo_driver.get_joint_state()
        if joint_state:
            return joint_state.angles
        else:
            return self.servo_driver.get_joint_angles()
        
    def get_pose(self) -> Optional[Union[List[float], Dict]]:
        """
        :return: Dict with position, rotation, euler_xyz, quaternion_xyzw, transform
        """

        joint_angles = self.get_joints()
        if joint_angles is None:
            logger.error("无法获取关节角度")
            return None

        T_fk = forward_kinematics(
            self.robot_model, 
            joint_angles, 
            backend='numpy', 
            return_end=True
        )

        position_fk = T_fk[:3, 3]
        rotation_fk = T_fk[:3, :3]
        euler_fk = matrix_to_euler(rotation_fk, seq='xyz')
        quat_fk = matrix_to_quaternion(rotation_fk)
            
        return {
            'transform': T_fk,
            'position': position_fk,
            'rotation': rotation_fk,
            'euler_xyz': euler_fk,
            'quaternion_xyzw': quat_fk
        }

    
    def get_gripper(self) -> Optional[float]:
        """
        :return: Gripper value in 0-100
        """
        return self.servo_driver.get_gripper_data()
        # gripper_data = self.servo_driver.get_gripper_data()
        # if gripper_data:
        #     # get_gripper_data returns (gripper_rad, button1, button2)
        #     # Convert from radians to 0-100 range
        #     gripper_rad = gripper_data[0]
        #     gripper_value = gripper_rad * 180.0 / np.pi
        #     return gripper_value
        # return None
    
    def get_firmware_version(self) -> Optional[str]:
        """
        :return: Firmware version string
        """
        # print("get_firmware_version")
        # print(self.servo_driver.get_firmware_version())
        return self.servo_driver.get_firmware_version()
    

    
    # ==================== Advanced Trajectory Methods ====================
    
    def move_joint_trajectory(self,
                             q_end: List[float],
                             duration: float = 2.0,
                             method: str = 'cubic',
                             num_points: int = 100,
                             visualize: bool = False) -> bool:
        """
        :param q_end: Target joint angles in radians
        :param duration: Duration in seconds
        :param method: 'linear', 'cubic', or 'quintic'
        :param num_points: Number of trajectory points
        :param visualize: Whether to visualize
        :return: True if successful
        """
        q_start = self.get_joints()
        if q_start is None:
            logger.error("无法获取当前关节角度")
            return False
        
        q_start = np.array(q_start)
        q_end = np.array(q_end)
        
        # 检查关节限位
        q_end, violations = check_and_clip_joint_limits(
            joints=q_end.tolist(),
            joint_limits=self.robot_model.joint_limits
        )
        q_end = np.array(q_end)
        
        for joint_name, original, clipped in violations:
            logger.warning(f"{joint_name} 超出限制：{original:.2f} -> {clipped:.2f}")
        
        # 生成轨迹
        logger.info(f"使用 {method} 插值生成关节轨迹 (时长: {duration}s, 点数: {num_points})")
        
        if method == 'linear':
            _, q, _, _ = linear_joint_trajectory(q_start, q_end, duration, num_points)
        elif method == 'cubic':
            _, q, _, _ = cubic_polynomial_trajectory(q_start, q_end, duration, num_points)
        elif method == 'quintic':
            _, q, _, _ = quintic_polynomial_trajectory(q_start, q_end, duration, num_points)
        else:
            logger.error(f"不支持的插值方法: {method}")
            return False
        
        # 执行轨迹
        delay = duration / num_points
        self.hardware_executor.delay = delay
        
        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize
        )
        
        return result if result is not None else True
    
    def move_cartesian_linear(self,
                             target_pose: List[float],
                             duration: float = 2.0,
                             num_points: int = 50,
                             ik_method: str = 'dls',
                             visualize: bool = False) -> bool:
        """
        :param target_pose: [x, y, z, qx, qy, qz, qw]
        :param duration: Duration in seconds
        :param num_points: Number of trajectory points
        :param ik_method: IK method
        :param visualize: Whether to visualize
        :return: True if successful
        """
        # 获取当前位姿
        current_pose_dict = self.get_pose()
        if current_pose_dict is None:
            logger.error("无法获取当前位姿")
            return False
        
        pose_start = current_pose_dict['transform']
        
        # 构建目标位姿矩阵
        position = np.array(target_pose[:3])
        quaternion = np.array(target_pose[3:])
        rotation_matrix = quaternion_to_matrix(quaternion)
        pose_end = make_transform(rotation_matrix, position)
        
        # 获取当前关节角度作为IK初始猜测
        q_init = self.get_joints()
        if q_init is None:
            logger.error("无法获取当前关节角度")
            return False
        q_init = np.array(q_init)
        
        logger.info(f"生成笛卡尔直线轨迹 (时长: {duration}s, 点数: {num_points})")
        
        # 生成轨迹
        try:
            _, _, q = linear_cartesian_trajectory(
                self.robot_model,
                pose_start,
                pose_end,
                duration,
                num_points=num_points,
                q_init=q_init,
                ik_backend='numpy',
                ik_method=ik_method,
                max_iters=100,
                pos_tol=1e-3,
                ori_tol=1e-3
            )
        except Exception as e:
            logger.error(f"轨迹规划失败: {e}")
            return False
        
        # 执行轨迹
        delay = duration / num_points
        self.hardware_executor.delay = delay
        
        logger.info(f"执行笛卡尔轨迹 (总点数: {len(q)})")

        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize
        )
        
        return result if result is not None else True
    
    def move_cartesian_circular(self,
                               center: List[float],
                               normal: List[float],
                               radius: float,
                               start_angle: float,
                               end_angle: float,
                               duration: float = 3.0,
                               num_points: int = 50,
                               orientation: str = 'constant',
                               ik_method: str = 'dls',
                               visualize: bool = False) -> bool:
        """
        :param center: [x, y, z]
        :param normal: [nx, ny, nz]
        :param radius: Radius in meters
        :param start_angle: Start angle in radians
        :param end_angle: End angle in radians
        :param duration: Duration in seconds
        :param num_points: Number of trajectory points
        :param orientation: 'constant' or 'tangent'
        :param ik_method: IK method
        :param visualize: Whether to visualize
        :return: True if successful
        """
        # 获取当前关节角度作为IK初始猜测
        q_init = self.get_joints()
        if q_init is None:
            logger.error("无法获取当前关节角度")
            return False
        q_init = np.array(q_init)
        
        center_array = np.array(center)
        normal_array = np.array(normal)
        
        logger.info(f"生成笛卡尔圆弧轨迹 (半径: {radius}m, 角度: {np.rad2deg(start_angle):.1f}° -> {np.rad2deg(end_angle):.1f}°)")
        
        # 生成轨迹
        try:
            _, _, q = circular_cartesian_trajectory(
                self.robot_model,
                center_array,
                normal_array,
                radius,
                start_angle,
                end_angle,
                duration,
                num_points=num_points,
                orientation=orientation,
                q_init=q_init,
                ik_backend='numpy',
                ik_method=ik_method,
                max_iters=100,
                pos_tol=1e-2,
                ori_tol=1e-2
            )
        except Exception as e:
            logger.error(f"圆弧轨迹规划失败: {e}")
            return False
        
        # 执行轨迹
        delay = duration / num_points
        self.hardware_executor.delay = delay
        
        logger.info(f"执行圆弧轨迹 (总点数: {len(q)})")
        print("trajectory:", q)
        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize
        )
        
        return result if result is not None else True
    
    def move_cartesian_waypoints(self,
                                waypoint_poses: List[List[float]],
                                durations: Union[float, List[float]] = 2.0,
                                num_points_per_segment: int = 50,
                                ik_method: str = 'dls',
                                visualize: bool = False) -> bool:
        """
        :param waypoint_poses: List of poses [x, y, z, qx, qy, qz, qw]
        :param durations: Segment duration(s) in seconds
        :param num_points_per_segment: Points per segment
        :param ik_method: IK method
        :param visualize: Whether to visualize
        :return: True if successful
        """
        if len(waypoint_poses) < 2:
            logger.error("至少需要2个路径点")
            return False
        
        # 转换路径点为变换矩阵
        pose_matrices = []
        for pose in waypoint_poses:
            position = np.array(pose[:3])
            quaternion = np.array(pose[3:])
            rotation_matrix = quaternion_to_matrix(quaternion)
            T = make_transform(rotation_matrix, position)
            pose_matrices.append(T)
        
        pose_matrices = np.array(pose_matrices)
        
        # 获取当前关节角度作为IK初始猜测
        q_init = self.get_joints()
        if q_init is None:
            logger.error("无法获取当前关节角度")
            return False
        q_init = np.array(q_init)
        
        logger.info(f"生成笛卡尔多路径点轨迹 ({len(waypoint_poses)}个路径点)")
        
        # 生成轨迹
        try:
            t, _, q = cartesian_waypoint_trajectory(
                self.robot_model,
                pose_matrices,
                durations,
                num_points_per_segment=num_points_per_segment,
                q_init=q_init,
                ik_backend='numpy',
                ik_method=ik_method,
                max_iters=100,
                pos_tol=1e-3,
                ori_tol=1e-3
            )
        except Exception as e:
            logger.error(f"多路径点轨迹规划失败: {e}")
            return False
        
        # 执行轨迹
        total_duration = t[-1]
        delay = total_duration / len(q)
        self.hardware_executor.delay = delay
        
        logger.info(f"执行笛卡尔轨迹 (总时长: {total_duration:.2f}s, 总点数: {len(q)})")
        
        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize
        )
        
        return result if result is not None else True
    
    # ==================== 系统控制 ====================
    def set_acceleration(self, acceleration: int = 1) -> bool:
        """
        :param acceleration: 加速度
        :return: True if successful
        """
        return self.servo_driver.set_acceleration(acceleration)

    def torque_control(self, command: str) -> bool:
        """
        :param command: 'on' or 'off'
        :return: True if successful
        """
        if command == "on":
            logger.info("开启扭矩")
            return self.servo_driver.enable_torque()
        elif command == "off":
            logger.info("关闭扭矩")
            return self.servo_driver.disable_torque()
        else:
            logger.error("command 参数必须是 'on' 或 'off'")
            return False
    
    def zero_calibration(self) -> bool:
        """
        :return: True if successful
        """
        logger.info("开始归零校准,机械臂将失去扭矩")
        logger.info("按下回车继续")
        input()
        # 关闭扭矩
        if not self.torque_control('off'):
            logger.error("扭矩关闭失败")
            return False
        logger.info("请手动拖动机械臂到零点位置，然后按回车继续...")
        input()
        
        # 重新开启扭矩
        if not self.torque_control('on'):
            logger.error("扭矩开启失败")
            return False
        
        # 执行零点校准
        result = self.servo_driver.set_zero_position()
        if result:
            logger.info("归零校准成功")
        else:
            logger.error("归零校准失败")
        
        return result
    
    def emergency_stop(self) -> bool:
        """
        :return: True if successful
        """
        logger.warning("执行紧急停止")
        return self.motion_controller.emergency_stop()
    
    def clear_emergency_stop(self) -> bool:
        """
        :return: True if successful
        """
        logger.info("清除紧急停止状态")
        return self.motion_controller.clear_emergency_stop()
    


    
    def print_state(self, continuous: bool = False, output_format: str = "deg"):
        """
        :param continuous: Whether to print continuously
        :param output_format: 'deg' or 'rad'
        """
        def _print_once():
            joints = self.get_joints()
            pose = self.get_pose()
            gripper = self.get_gripper()
            
            if joints is None:
                logger.warning("无法获取关节状态")
                return

            if output_format == 'deg':
                joint_deg = [angle * 180.0 / np.pi for angle in joints]
                unit = "°"
            else:
                joint_deg = joints
                unit = "rad"
            
            gripper_value = gripper if gripper is not None else 0.0
            
            logger.info(f"关节角度 ({unit}): {[round(a, 2) for a in joint_deg]}, 夹爪开合度: {round(gripper_value, 1)} ")
            if pose:
                position = pose['position']
                quaternion = pose['quaternion_xyzw']
                logger.info(f"位置(xyz /m): {[round(p, 3) for p in position]}, 四元数(qx, qy, qz, qw): {[round(q, 3) for q in quaternion]}")
                print("\n")
        if continuous:
            logger.info("开始连续状态打印，按 Ctrl+C 停止")
            try:
                while True:
                    _print_once()
                    time.sleep(0.5)
            except KeyboardInterrupt:
                logger.info("停止连续状态打印")
        else:
            _print_once()
    

    def _generate_random_q(self, scale: float = 0.5) -> List[float]:
        """
        :param scale: Range scale within joint limits
        :return: Random joint configuration
        """
        rng = np.random.default_rng()
        q = [0.0] * self.robot_model.num_dof()
        
        for js in self.robot_model._actuated:
            lo, hi = -1.0, 1.0
            if js.limit:
                if js.limit[0] is not None:
                    lo = js.limit[0]
                if js.limit[1] is not None:
                    hi = js.limit[1]
            mid = 0.5 * (lo + hi)
            span = 0.5 * (hi - lo) * scale
            q[js.index] = float(rng.uniform(mid - span, mid + span))
        
        return q


    def __del__(self):
        try:
            self.disconnect()
        except Exception as e:
            logger.error(f"SynriaRobotAPI析构异常: {e}")