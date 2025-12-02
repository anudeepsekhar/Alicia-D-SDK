import math
import time
import logging
import threading
from typing import List, Optional, Union, Tuple, Dict
import numpy as np

from alicia_d_sdk.hardware.serial_comm import SerialComm
from alicia_d_sdk.hardware.data_parser import DataParser, JointState
from alicia_d_sdk.utils.logger import logger


class ServoDriver:
    """Robot arm control module"""
    
    # Constant definitions
    
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
    # Frame constants
    FRAME_HEADER = 0xAA
    FRAME_FOOTER = 0xFF

    # Command IDs
    CMD_JOINT = 0x06       # Arm joint angle feedback and control
    
    # Gripper type configuration
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3590
    
    # Raw instruction mapping table for information retrieval and control
    INFO_COMMAND_MAP: Dict[str, List[int]] = {
        # Get firmware version
        "version": [0xAA, 0x01, 0x00, 0x01, 0xFE, 0x23, 0xFF],
        # Set current position as zero
        "zero_cali": [0xAA, 0x03, 0x00, 0x01, 0xFE, 0xA8, 0xFF],
        # Torque on/off
        "torque_on": [0xAA, 0x05, 0x00, 0x01, 0x01, 0xF9, 0xFF],
        "torque_off": [0xAA, 0x05, 0x00, 0x01, 0x00, 0x6F, 0xFF],
        # Joint information acquisition (position and status)
        "joint": [0xAA, 0x06, 0x00, 0x01, 0xFE, 0x9A, 0xFF],
        # Temperature information acquisition
        "temperature": [0xAA, 0x06, 0x01, 0x01, 0xFE, 0xAD, 0xFF],
        # Velocity information acquisition
        "velocity": [0xAA, 0x06, 0x02, 0x01, 0xFE, 0xF4, 0xFF],
        "self_check": [0xAA, 0xFE, 0x00, 0x00, 0xFE, 0x93, 0xFF],
    }

    def __init__(self, port: str = "", baudrate: int = 1000000, debug_mode: bool = False, gripper_type: str = "50mm"):
        """
        Initialize robot arm controller
        
        :param port: Serial port name, leave empty to auto search
        :param baudrate: Baud rate
        :param debug_mode: Whether to enable debug mode
        """
        self.debug_mode = debug_mode
        self._lock = threading.Lock()
        self.gripper_type = gripper_type



        # Create serial communication module and data parser
        self.serial_comm = SerialComm(lock=self._lock, port=port, baudrate=baudrate, debug_mode=debug_mode)
        self.data_parser = DataParser(lock=self._lock, debug_mode=debug_mode)
        
        # Number of servos
        self.servo_count = 9
        self.joint_count = 6
        
        # Servo mapping table: joint index -> servo index
        # 6 joints of the arm are mapped to 9 servos
        # [joint1, joint1(duplicate), joint2, joint2(reversed), joint3, joint3(reversed), joint4, joint5, joint6]
        self.joint_to_servo_map = [
            (0, 1.0),    # joint 1 -> servo 1 (normal)
            (0, 1.0),    # joint 1 -> servo 2 (duplicate)
            (1, 1.0),    # joint 2 -> servo 3 (normal)
            (1, -1.0),   # joint 2 -> servo 4 (reversed)
            (2, 1.0),    # joint 3 -> servo 5 (normal)
            (2, -1.0),   # joint 3 -> servo 6 (reversed)
            (3, 1.0),    # joint 4 -> servo 7 (normal)
            (4, 1.0),    # joint 5 -> servo 8 (normal)
            (5, 1.0),    # joint 6 -> servo 9 (normal)
        ]
        
        # State update thread related
        self._update_thread = None
        self.thread_update_interval = 0.005  # Update interval in seconds
        self._stop_thread = threading.Event()
        self._thread_running = False
        
        self.disconnect()
    
    def wait_for_valid_state(self, timeout: float = 1.5) -> bool:
        """
        Wait for robot arm state to become valid

        :param timeout: Maximum waiting time (seconds)
        :return: Whether a valid state was received within timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            js = self.data_parser.get_joint_state()
            if js and max(abs(a) for a in js.angles) > 1e-3:
                return True
            time.sleep(0.05)
        # print(f"[Timeout] No valid joint state received")
        return False

    def __del__(self):
        """Destructor to ensure threads and connections are properly cleaned up"""
        try:
            # Stop state update thread
            self.stop_update_thread()
            # Disconnect
            self.disconnect()
        except Exception as e:
            if hasattr(logger, 'error'):  # logger may be destroyed in some cases
                logger.error(f"Exception in destructor: {str(e)}")
    
    def connect(self) -> bool:
        """
        Connect to the robot arm
        
        :return: Whether connection is successful
        """
        result = self.serial_comm.connect()
        if result:
            # Start state update thread after successful connection
            self.start_update_thread()
            self.wait_for_valid_state()
        return result
    
    def disconnect(self):
        """Disconnect from the robot arm"""
        # Stop state update thread first
        self.stop_update_thread()
        self.serial_comm.disconnect()
    
    def start_update_thread(self):
        """Start state update thread"""
        if self._update_thread is not None and self._thread_running:
            logger.info("State update thread is already running")
            return
        
        # Reset stop flag
        self._stop_thread.clear()
        self._thread_running = True
        
        # Create and start thread
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
    
    def stop_update_thread(self):
        """Stop state update thread"""
        if self._update_thread is None or not self._thread_running:
            return
        
        # Set stop flag
        self._stop_thread.set()
        self._thread_running = False
        
        # Wait for thread to finish
        if self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
        
        self._update_thread = None
    
    def is_update_thread_running(self) -> bool:
        """
        Check whether state update thread is running
        """
        return self._thread_running and self._update_thread is not None and self._update_thread.is_alive()
    
    def get_update_thread_status(self) -> Dict:
        """
        Get detailed status of the state update thread
        
        Returns:
            Dict: Dictionary containing thread status
        """
        return {
            "running": self.is_update_thread_running(),
            "enabled": self._thread_running,
            "thread_exists": self._update_thread is not None,
            "thread_alive": self._update_thread.is_alive() if self._update_thread else False,
            "stop_flag_set": self._stop_thread.is_set()
        }
    
    def _update_loop(self):
        """Main loop of state update thread"""
        
        while not self._stop_thread.is_set():
            time.sleep(self.thread_update_interval)
            try:
                with self._lock:
                    # Read one frame
                    frame = self.serial_comm.read_frame()
                if frame == 9999999:
                    logger.error("Severe serial communication error detected, robot arm may be disconnected", raise_exception=False)
                    break
                if frame:
                    self.data_parser.parse_frame(frame)
        
            except Exception as e:
                logger.error(f"State update thread exception: {str(e)}", raise_exception=False)
                
                
                break
        self._thread_running = False


    
    def acquire_info(self, info_type: str, wait: bool = False, timeout: float = 2.0) -> bool:
        """
        General information acquisition interface, selecting different commands by type.
        Args:
            info_type: Type of information to acquire. Supported types:
                - "version"
                - "zero_cali"
                - "torque_on"
                - "torque_off"
                - "joint"
            wait: If True, wait for the response to be received and parsed
            timeout: Maximum time to wait in seconds (only used if wait=True)
        """
        if info_type not in self.INFO_COMMAND_MAP:
            raise ValueError(f"Unsupported info type: {info_type}")

        # Clear the corresponding event before sending request (if applicable)
        if info_type in self.data_parser._info_event_map:
            event = self.data_parser._info_event_map[info_type]
            event.clear()

        command = self.INFO_COMMAND_MAP[info_type]
        success = self.serial_comm.send_data(command)

        if not success:
            return False
        
        if wait:
            if info_type in self.data_parser._info_event_map:
                return self.data_parser.wait_for_info(info_type, timeout)
            return True
        
        return True

    

    def set_joint_and_gripper(self, 
                              joint_angles: Optional[List[float]] = None,
                              gripper_value: Optional[float] = None,
                              speed_deg_s: float = 20.0) -> bool:
        """
        Unified method to set joints, gripper, or both in a single combined frame.
        Args:
            joint_angles: Optional angle list (radians) for 6 joints. 
                          If None, keeps current joints (or zeros if state unavailable).
            gripper_value: Optional gripper value (0-100). 
                           If None, keeps current gripper (or 50.0 if state unavailable).
            speed_deg_s: Speed in degrees per second (0-360, required range), default 20.0
        """

        if speed_deg_s < 0 or speed_deg_s > 360:
            logger.error(f"Speed out of range: {speed_deg_s} deg/s (valid range: 0-360)")
            return False
        
        frame = self._build_joint_frame(
            joint_angles=joint_angles,
            gripper_value=gripper_value,
            speed_deg_s=speed_deg_s
        )

        # self.serial_comm._hex_print("Send combined control", frame)
        result = self.serial_comm.send_data(frame)
        
        if self.debug_mode:
            self.serial_comm._hex_print("Send combined control", frame)
        
        return result
    
    
    def _build_joint_frame(self, 
                           joint_angles: Optional[List[float]] = None, 
                           gripper_value: Optional[float] = None, 
                           speed_deg_s: float = 10.0) -> List[int]:
        """
        Build combined joint + gripper + speed control frame (CMD=0x06, FUNC=0x03)

        Args:
            joint_angles: Optional angle list (radians) for 6 joints. 
                          If None, keeps current joints (or zeros if state unavailable).
            gripper_value: Optional gripper value (0-100). 
                           If None, keeps current gripper (or 50.0 if state unavailable).
            speed_deg_s: Speed in degrees per second (0-360, required range, maps to hardware 0-5500).
                         The same speed is applied to all joints. The gripper is fixed at 5500.

        """
        # 6 joints * 4 bytes (value + speed) + 1 gripper * 4 bytes (value + speed) = 28 bytes
        DATA_LENGTH = 0x1C
        FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1  # header + cmd + func + len + data + checksum + footer
        
        # Create frame
        frame = [0] * FRAME_SIZE
        frame[0] = self.FRAME_HEADER  # 0xAA
        frame[1] = self.CMD_JOINT     # 0x06
        frame[2] = 0x03               # Function code
        frame[3] = DATA_LENGTH        # Data length: 0x1C (28 bytes)
        frame[-1] = self.FRAME_FOOTER  # 0xFF
        
        data_start = 4

        # Get current state for optional values
        current_state = self.data_parser.get_joint_state()

        if joint_angles is None:
            if current_state and current_state.angles:
                effective_joints = current_state.angles
            else:
                # Default to zero if no current state available
                effective_joints = [0.0] * self.joint_count
        else:
            if len(joint_angles) != self.joint_count:
                logger.error(f"Incorrect joint count: need {self.joint_count}, got {len(joint_angles)}")
                # Fall back to current or zeros to avoid crashing
                if current_state and current_state.angles:
                    effective_joints = current_state.angles
                else:
                    effective_joints = [0.0] * self.joint_count
            else:
                effective_joints = joint_angles

        # Convert common speed value to hardware units (used for all joints and gripper)
        speed_hw_value = self._value_to_hardware_value_speed(speed_deg_s)
        
        for joint_idx in range(6):
            angle_rad = effective_joints[joint_idx]
            hardware_value = self._rad_to_hardware_value(angle_rad)
            # print(f"hardware_value_target: {hardware_value}")
            offset = data_start + joint_idx * 4
            frame[offset] = hardware_value & 0xFF              # low byte
            frame[offset + 1] = (hardware_value >> 8) & 0xFF   # high byte
            frame[offset + 2] = speed_hw_value & 0xFF
            frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
        
        # Gripper value and speed (4 bytes: 2 bytes value, 2 bytes speed)
        gripper_offset = data_start + 6 * 4
        if gripper_value is not None:
            gripper_hw_value = int(max(0, min(1000, gripper_value)))
        else:
            if current_state and current_state.gripper is not None:
                gripper_hw_value = int(max(0, min(1000, current_state.gripper)))
            else:
                gripper_hw_value = 1000  # Default middle position

        gripper_speed_hw_value = 5500
        frame[gripper_offset] = gripper_hw_value & 0xFF
        frame[gripper_offset + 1] = (gripper_hw_value >> 8) & 0xFF
        frame[gripper_offset + 2] = gripper_speed_hw_value & 0xFF
        frame[gripper_offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        if self.debug_mode:
            angle_deg = [round(angle * self.RAD_TO_DEG, 2) for angle in joint_angles]
            logger.debug(f"Send combined frame - joints (deg): {angle_deg}, gripper: {gripper_value}, speed: {speed_deg_s} deg/s")
            
        return frame

    
    def _rad_to_hardware_value(self, angle_rad: float) -> int:
        """
        Convert radians to hardware value (0-4095)
        Args:
            angle_rad: Angle (radians)
        """
        # Range check in radians
        if angle_rad < -math.pi or angle_rad > math.pi:
            logger.warning(f"Angle out of range: {angle_rad:.2f} rad, will be clipped")
            angle_rad = max(-math.pi, min(math.pi, angle_rad))
        
        value = int((angle_rad + math.pi) / (2 * math.pi) * 4096)
        
        return max(0, min(4095, value))
    

    
    def _value_to_hardware_value_speed(self, speed_deg_s: float) -> int:
        """
        Converts speed from degrees per second to hardware value (0-5500).
        
        Speed range: 0 to 360 degrees per second (required, not recommended).
        
        :param speed_deg_s: The desired speed in degrees per second (0-360, required range).
        :return: A corresponding raw integer speed value (0-5500).
        """
        MIN_SPEED_DEG_S = 0.0
        MAX_SPEED_DEG_S = 360.0
        MAX_HARDWARE_VALUE = 5500
        
        # Validate and clip speed to required range
        if speed_deg_s < MIN_SPEED_DEG_S:
            logger.warning(f"Speed below range: {speed_deg_s} deg/s (min {MIN_SPEED_DEG_S}), will be clipped to {MIN_SPEED_DEG_S}")
            speed_deg_s = MIN_SPEED_DEG_S
        elif speed_deg_s > MAX_SPEED_DEG_S:
            logger.warning(f"Speed above range: {speed_deg_s} deg/s (max {MAX_SPEED_DEG_S}), will be clipped to {MAX_SPEED_DEG_S}")
            speed_deg_s = MAX_SPEED_DEG_S
        
        hardware_value = int((speed_deg_s / MAX_SPEED_DEG_S) * MAX_HARDWARE_VALUE)
        
        return max(0, min(MAX_HARDWARE_VALUE, hardware_value))


    