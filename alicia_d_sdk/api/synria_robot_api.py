"""
SynriaRobotAPI - User-level API

Responsibilities:
- Provide concise unified user interface
- High-level motion command encapsulation
- State query interface
- System control functions
- Parameter validation and error handling
"""

import time
from typing import List, Optional, Dict, Union, Tuple
import numpy as np

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
)
from alicia_d_sdk.hardware import ServoDriver
from alicia_d_sdk.execution import HardwareExecutor
from alicia_d_sdk.utils.logger import logger
from robocore.utils.control_utils import compute_steps_and_delay, validate_joint_list, check_and_clip_joint_limits
from alicia_d_sdk.utils.calculate import calculate_movement_duration


class SynriaRobotAPI:
    """Synria robot arm API - provides unified user interface"""
    
    def __init__(self,
                 servo_driver: ServoDriver,
                 robot_model: RobotModel):
        """Initialize robot API.

        :param servo_driver: Servo driver instance
        :param robot_model: Robot model from RoboCore
        """
        self.servo_driver = servo_driver
        self.data_parser = servo_driver.data_parser  # Direct access to data parser
        self.robot_model = robot_model

        self.hardware_executor = HardwareExecutor(servo_driver)
        self.robot_type = None
    
    # ==================== Connection Management ===================

    def connect(self) -> bool:
        """Connect to robot and detect firmware version.
        """
        result = self.servo_driver.connect()
        if result:
            self.get_robot_state()
        return result
    
    def disconnect(self):
        """Disconnect from robot and stop update threads."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()
    
    def is_connected(self) -> bool:
        """Check if robot is connected.
        """
        return self.servo_driver.serial_comm.is_connected()
    

    # ==================== Get Robot Information ====================
    def get_version(self, timeout: float = 1.0, log:bool = True) -> Optional[Dict[str, str]]:
        """
        Get full version information (serial number, hardware version, firmware version).
        
        :param timeout: Maximum time to wait for response in seconds
        :param log: Whether to log version info
        :return: Dictionary with version info or None if failed
        """
        # Request version info and wait for response
        if not self.servo_driver.acquire_info("version", wait=True, timeout=timeout):
            logger.error("Failed to get version info within timeout period")
            return None
        
        version_info = self.data_parser.get_version_info()
        if version_info is None:
            logger.error("Version info not available after successful acquisition")
            return None

        if log:
            logger.info(
            "Version info: "
            f"Unique ID = {version_info.get('serial_number')}, "
            f"Hardware Version = {version_info.get('hardware_version')}, "
            f"Firmware Version = {version_info.get('firmware_version')}"
        )
        return version_info


    
    def get_robot_state(self, robot_type = None) -> Optional[Union[List[float], Tuple[List[float], bool, bool]]]:
        """Get current joint angles.
        
        :param robot_type: 'follower' or 'leader' or none for auto detection
        :return: Joint state or None if failed
        """
        if robot_type is None:
            robot_type = self._robot_type()
        self.servo_driver.acquire_info("joint", wait=True, timeout=1.0)
        joint_state = self.data_parser.get_joint_state()
        return joint_state
    
    def get_joints(self, robot_type = None) -> Optional[List[float]]:
        """Get current joint angles.
        
        :param robot_type: 'follower' or 'leader' or none for auto detection
        :return: List of joint angles in radians or None if failed
        """
        joint_state = self.get_robot_state(robot_type)
        if joint_state:
            return joint_state.angles
        return None
        
    
    def get_gripper(self) -> Optional[float]:
        """Get current gripper position.

        :return: Gripper value from 0 (closed) to 100 (open) or None if failed
        """
        joint_state = self.get_robot_state()
        if joint_state:
            return joint_state.gripper
        return None
    
    def get_temperature(self, timeout: float = 5.0) -> Optional[List[float]]:
        """Get current servo temperatures.
        
        :param timeout: Maximum time to wait for response in seconds
        :return: List of temperatures in Celsius for each servo, or None if failed
        """
        if not self.servo_driver.acquire_info("temperature", wait=True, timeout=timeout):
            logger.error("Failed to get temperature data within timeout period")
            return None
        
        temp_data = self.data_parser.get_temperature_data()
        if temp_data is None:
            logger.error("Temperature data not available after successful acquisition")
            return None
        
        return temp_data['temperatures']
    
    def get_velocity(self, timeout: float = 1.0) -> Optional[List[float]]:
        """Get current servo velocities.
        
        :param timeout: Maximum time to wait for response in seconds
        :return: List of velocities in degrees per second for each servo, or None if failed
        """
        if not self.servo_driver.acquire_info("velocity", wait=True, timeout=timeout):
            logger.error("Failed to get velocity data within timeout period")
            return None
        
        vel_data = self.data_parser.get_velocity_data()
        if vel_data is None:
            logger.error("Velocity data not available after successful acquisition")
            return None
        
        return vel_data['velocities']

    def get_self_check(self, timeout: float = 1.0):
        """
        Execute machine self-check (servo health) and return detailed status.

        :param timeout: Maximum time to wait for response in seconds
        :return: Dict with keys: robot_type, raw_mask, servo_status_bits, all_ok, faulty_ids, timestamp, or None if failed
        """
        if not self.servo_driver.acquire_info("self_check", wait=True, timeout=timeout):
            logger.error("Failed to get self-check data within timeout period")
            return None

        sc_data = self.data_parser.get_self_check_data()

        robot_type = self._robot_type()
        if robot_type == "leader":
            bits_full = sc_data["bits"][:6]
        elif robot_type == "follower":
            bits_full = sc_data["bits"][:10]
        else:
            logger.error("Invalid robot type")
            return None
        if sc_data is None:
            logger.error("Self-check data not available after successful acquisition")
            return None

        # check if the bits_full is all True
        if all(bits_full):
            logger.info("All servos are OK")
        else:
            # print the faulty servo ids
            faulty_ids = [i + 1 for i, bit in enumerate(bits_full) if not bit]
            logger.warning(f"Broken servo IDs: {faulty_ids}")
            return None




    def get_pose(self) -> Optional[Union[List[float], Dict]]:
        """Get current end-effector pose.

        :return: Dictionary with position, rotation, euler_xyz, quaternion_xyzw, transform, or None if failed
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


    # ==================== Robot Control ====================                         
    
    def set_home(self, speed_deg_s: int = 10):
        """Move robot to home position and wait until near zero.

        :param speed_deg_s: Speed in degrees per second (0-360, required range)
        """
        # time.sleep(0.1)
        home_joints = [0.0] * 6
        self.set_robot_target(target_joints=home_joints, gripper_value=1000, speed_deg_s=speed_deg_s, wait_for_completion=True)

    
    def set_robot_target(self,
                            target_joints: Optional[List[float]] = None,
                            gripper_value: Optional[int] = None,
                            joint_format: str = 'rad',
                            speed_deg_s: int = 10,
                            tolerance: float = 0.05,
                            timeout: float = 10.0,
                            wait_for_completion: bool = True) -> bool:
        """Set joint angles and/or gripper in a single combined command.
        
        :param target_joints: Optional target joint angles. If None, keeps current
        :param gripper_value: Optional gripper value (0-100). If None, keeps current
        :param joint_format: Unit format for joints, 'rad' or 'deg'
        :param speed_deg_s: Speed in degrees per second (0-360, required range)
        :param tolerance: Rad, acceptable abs distance to target for joints
        :param timeout: Seconds, maximum wait time
        :param wait_for_completion: If True, wait until target reached
        :return: True if successful, False otherwise
        """
        # Convert joint format if needed
        if target_joints is not None:
            if joint_format == 'deg':
                target_joints = [a * np.pi / 180.0 for a in target_joints]
        
        # Use unified method
        success = self.servo_driver.set_joint_and_gripper(joint_angles=target_joints, gripper_value=gripper_value, speed_deg_s=speed_deg_s)
        
        if not success:
            logger.error("Failed to set robot target")
            return False
        
        # If no joint target is provided, there is nothing to wait for on joints.
        if wait_for_completion and target_joints is not None:
            return self._wait_for_joint_target(
                target_joints=target_joints,
                tolerance=tolerance,
                timeout=timeout,
                log_prefix="等待关节接近目标"
            )
        
        # Either waiting was not requested, or only gripper was commanded.
        return True
    

    def set_pose_target(self, 
                       target_pose: List[float], 
                       backend: str = 'numpy', 
                       method: str = 'dls', 
                       display: bool = True, 
                       tolerance: float = 1e-4, 
                       max_iters: int = 100,
                       multi_start: int = 0,
                       use_random_init: bool = False,
                       speed_deg_s: int = 10,
                       execute: bool = True) -> Dict:
        """Move end-effector to target pose using inverse kinematics.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param backend: Computation backend, 'numpy' or 'torch'
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param display: Display solution details
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param multi_start: Number of multi-start attempts, 0 to disable
        :param use_random_init: Use random initial guess instead of current pose
        :param speed_deg_s: Motion speed in degrees per second
        :param execute: Execute motion if True
        :return: Dictionary with success, q, iters, pos_err, ori_err, message
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
                result = self.set_robot_target(target_joints=ik_result['q'], joint_format='rad', speed_deg_s=speed_deg_s, wait_for_completion=True)
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
    

    # ==================== Advanced Trajectory Methods ====================
    
    def move_joint_trajectory(self,
                             q_end: List[float],
                             duration: float = 2.0,
                             method: str = 'cubic',
                             num_points: int = 100,
                             visualize: bool = False) -> bool:
        """Execute smooth joint trajectory to target position.

        :param q_end: Target joint angles in radians
        :param duration: Trajectory duration in seconds
        :param method: Interpolation method, 'linear', 'cubic', or 'quintic'
        :param num_points: Number of trajectory waypoints
        :param visualize: Enable trajectory visualization
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
                             speed_deg_s: int = 10,
                             duration: float = 2.0,
                             num_points: int = 50,
                             ik_method: str = 'dls'
                             ) -> bool:
        """Execute linear Cartesian trajectory to target pose.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param speed_deg_s: Motion speed in degrees per second
        :param duration: Trajectory duration in seconds
        :param num_points: Number of trajectory waypoints
        :param ik_method: IK solver method
        :return: True if successful
        """
        # 获取当前位姿
        current_pose_dict = self.get_pose()
        if current_pose_dict is None:
            logger.error("无法获取当前位姿")
            return False
        
        pose_start = current_pose_dict['transform']
        
        # 构建目标位姿矩阵（仅基于末端位姿，不包含夹爪信息）
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
                q_init=None,
                ik_backend='numpy',
                ik_method=ik_method,
                max_iters=500,
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
            speed_deg_s=speed_deg_s,
        )
        
        return result if result is not None else True
    

    
    
    def torque_control(self, command: str, timeout: float = 1.0) -> bool:
        """Enable or disable robot torque.
        
        :param command: 'on' or 'off'
        :param timeout: Maximum time to wait for response in seconds
        :return: True if successful
        """
        if command == "on":
            return self.servo_driver.acquire_info("torque_on", wait=True, timeout=timeout)
        elif command == "off":
            return self.servo_driver.acquire_info("torque_off", wait=True, timeout=timeout)
        else:
            logger.error("command 参数必须是 'on' 或 'off'")
            return False

    
    def zero_calibration(self) -> bool:
        """Execute zero position calibration procedure.

        :return: True if calibration successful
        """
        logger.warning("此操作不可逆，将更改出厂零点位置，请谨慎操作")
        logger.info("开始归零校准,机械臂将失去扭矩")
        logger.info("按下回车继续, Ctrl+C 取消...")
        input()
        if not self.torque_control('off'):
            logger.error("扭矩关闭失败")
            return False
        logger.info("请手动拖动机械臂到零点位置，然后按回车继续...")
        input()

        if not self.servo_driver.acquire_info("zero_cali", wait=True, timeout=2.0):
            logger.error("零点校准失败")
            return False
        time.sleep(0.1)
        self.servo_driver.acquire_info("torque_on", wait=True, timeout=1.0)
        return True
    
    
    def print_state(self, continuous: bool = False, output_format: str = "deg"):
        """Print current robot state.

        :param continuous: Print continuously if True, once if False
        :param output_format: Angle format, 'deg' or 'rad'
        """
        robot_type = self._robot_type()

        def _print_once(robot_type):
            pose = None
            state = self.get_robot_state(robot_type)
            # get temperature and velocity as well
            temperature = self.get_temperature()
            velocity = self.get_velocity()
            self.get_self_check()
            if state is None:
                logger.warning("无法获取关节状态")
                return

            joints = state.angles
            gripper = state.gripper
            status = state.run_status_text

            # Extract pose only for follower
            if robot_type == "follower":
                pose = self.get_pose()

            # Format joints for printing
            if output_format == 'deg':
                joint_out = np.round(np.array(joints) * 180.0 / np.pi, 2)
                unit = "°"
            else:
                joint_out = np.round(joints, 3)
                unit = "rad"
            logger.info(f"关节角度（{unit}): {joint_out.tolist()}, 夹爪(0-1000): {gripper}")
            if status != "idle":
                logger.info(f"按键状态：{status}")

            if pose is not None:
                quaternion = pose['quaternion_xyzw']
                position = pose['position']
                logger.info(f"位置(xyz /m): {np.round(position, 3).tolist()}, 四元数(qx, qy, qz, qw): {np.round(quaternion, 3).tolist()}")

            if temperature is not None:
                logger.info(f"舵机温度（°C): {np.round(temperature, 1).tolist()}")
            if velocity is not None:
                logger.info(f"舵机速度(deg/s) : {np.round(velocity, 1).tolist()}")
            

            print("\n")
        if continuous:
            logger.info("开始连续状态打印，按 Ctrl+C 停止")
            try:
                while True:
                    _print_once(robot_type)
                    time.sleep(0.03)
            except KeyboardInterrupt:
                logger.info("停止连续状态打印")
        else:
            _print_once(robot_type)
    

    def _generate_random_q(self, scale: float = 0.5) -> List[float]:
        """Generate random joint configuration within limits.

        :param scale: Range scale factor within joint limits
        :return: Random joint angles in radians
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


    def _wait_for_joint_target(self,
                               target_joints: Optional[List[float]],
                               tolerance: float,
                               timeout: float,
                               log_prefix: str = "等待关节接近目标") -> bool:
        """Wait until all joints reach target angles.

        :param target_joints: Target joint angles in radians. If None, returns True immediately
        :param tolerance: Rad, acceptable abs distance to target for all joints
        :param timeout: Seconds, maximum wait time
        :param log_prefix: Log message prefix
        :return: True if target reached, False if timeout
        """
        # If no joint target is specified, there is nothing to wait for.
        if target_joints is None:
            logger.debug("No joint target specified, skip joint waiting.")
            return True

        start_time = time.time()
        # logger.info(f"{log_prefix}...")

        while time.time() - start_time < timeout:
            current_joints = self.get_joints()
            if current_joints is not None:
                if all(abs(a - b) <= tolerance for a, b in zip(current_joints, target_joints)):
                    # logger.info("已到达目标位置")
                    return True
            time.sleep(0.05)
        logger.warning("等待关节到目标附近超时")
        # time.sleep(10)
        joints = self.get_joints()
        logger.warning(f"目标关节角度: {target_joints}")
        logger.warning(f"关节角度: {joints}")
        
        return False
    

    def __del__(self):
        try:
            self.disconnect()
        except Exception as e:
            logger.error(f"SynriaRobotAPI destructor exception: {e}")


    def _robot_type(self) -> str:
        if self.robot_type is not None:
            return self.robot_type

        version = self.get_version(log=False)
        if version is None:
            return None
            
        serial_number = version.get("serial_number")
        # ADFS for follower, ADLS for leader
        if serial_number.startswith("ADF"):
            self.robot_type = "follower"
        elif serial_number.startswith("ADL"):
            self.robot_type = "leader"
        
        return self.robot_type