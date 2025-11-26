import math
import time
from typing import List, Dict, Optional, NamedTuple
import threading
import copy
from alicia_d_sdk.utils.logger import logger

# 使用统一的日志器

class JointState(NamedTuple):
    """关节状态数据结构"""
    angles: List[float]  # 六个关节角度(弧度)
    gripper: float       # 夹爪角度(弧度)
    timestamp: float     # 时间戳(秒)
    button1: bool        # 按钮1状态
    button2: bool        # 按钮2状态

class DataParser:
    """机械臂数据解析模块"""
    # 常量定义
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    
    # 指令ID
    CMD_GRIPPER_V5 = 0x02     # 夹爪控制与行程反馈
    CMD_GRIPPER_V6 = 0x12     # 夹爪控制与行程反馈
    CMD_GRIPPER = 0x12     # 夹爪控制与行程反馈
    CMD_ZERO_POS = 0x03    # 机械臂以当前位置为零点
    CMD_JOINT_V5 = 0x04       # 机械臂角度反馈与控制  
    CMD_JOINT_V6 = 0x14       # 机械臂角度反馈与控制
    CMD_JOINT = 0x14       # 机械臂角度反馈与控制
    CMD_MULTI_ARM = 0x06   # 四机械臂角度反馈与控制
    CMD_VERSION = 0x0A     # 机械臂固件版本反馈
    CMD_TORQUE = 0x13      # 机械臂力矩控制
    CMD_ERROR = 0xEE       # 错误反馈
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3600
    # 数据长度
    JOINT_DATA_SIZE = 18
    
    def __init__(self, lock: threading.Lock, debug_mode: bool = False, gripper_type: str = "50mm", robot_type: str = "follower"):
        """
        初始化数据解析器
        
        Args:
            debug_mode: 是否启用调试模式
        """
        self.debug_mode = debug_mode
        if gripper_type == "50mm":
            self.servo_value_limit = self.GRI_MAX_50MM
        else:
            self.servo_value_limit = self.GRI_MAX_100MM 
        # 存储最新数据
        self._joint_states = JointState([0.0]*6, 0.0, 0.0,
                                        False, False)  # 六个关节角度(弧度)

        self.robot_type = robot_type
        self._firmware_version: Optional[str] = None
        self._lock = lock
        logger.info("初始化数据解析模块")
        if debug_mode:
            logger.info("调试模式: 启用")
    
    def parse_frame(self, frame: List[int]) -> Optional[Dict]:
        """
        解析数据帧
        
        Args:
            frame: 完整的数据帧(字节列表)
            
        Returns:
            Dict: 解析结果，如果解析失败则返回None
        """
        cmd_id = frame[1]
        if cmd_id == self.CMD_JOINT_V5 or cmd_id == self.CMD_JOINT_V6:
            return self._parse_joint_data(frame)
        elif cmd_id == self.CMD_GRIPPER_V5:
            return self._parse_gripper_data_old(frame)
        elif cmd_id == self.CMD_GRIPPER_V6:
            return self._parse_gripper_data(frame, robot_type=self.robot_type)
        elif cmd_id == self.CMD_ERROR:
            return self._parse_error_data(frame)
        elif cmd_id == self.CMD_VERSION:
            return self._parse_version_data(frame)
        else:
            if self.debug_mode:
                logger.debug(f"未处理的指令ID: 0x{cmd_id:02X}")
            return None
    
    def get_joint_state(self) -> Optional[JointState]:
        """
        获取关节状态
        
        Returns:
            Optional[JointState]: 当前关节状态，如果尚未更新则返回None
        """
        with self._lock:
            js = self._joint_states
            if js.angles is None or js.timestamp is None:
                logger.warning("机械臂状态尚未更新")
                return None
            return copy.deepcopy(self._joint_states)
        
    def get_firmware_version(self) -> Optional[str]:
        """
        获取固件版本信息
        
        Returns:
            Optional[str]: 固件版本字符串，如果尚未获取则返回None
        """
        with self._lock:
            # print("firmware in data_parser:", self._firmware_version)
            return self._firmware_version
        


    def _update_joint_state(self,
                        angles: Optional[List[float]] = None,
                        gripper: Optional[float] = None,
                        button1: Optional[bool] = None,
                        button2: Optional[bool] = None):
        with self._lock:
            prev = self._joint_states
            self._joint_states = JointState(
                angles=angles if angles is not None else prev.angles,
                gripper=gripper if gripper is not None else prev.gripper,
                timestamp=time.time(),
                button1=button1 if button1 is not None else prev.button1,
                button2=button2 if button2 is not None else prev.button2,
            )


    def _parse_joint_data(self, frame: List[int]) -> Dict:
        """
        解析关节数据帧 (0x04)
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 检查数据长度
        # print(f"帧长度: {len(frame)}")
        # # print frame in hex format:
        # frame_hex = " ".join([f"{b:02X}" for b in frame])
        # logger.info(f"接收到的帧: {frame_hex}")
        if frame[2] != self.JOINT_DATA_SIZE:  # 0x12对应十进制18 (9个舵机 * 2字节)
            logger.warning(f"关节数据长度错误: {frame[2]}")
            return None
        
        # 舵机到关节的映射表 - 与ROS一致
        servo_to_joint_map = {
            0: (0, 1.0),    # 舵机1 -> 关节1 (正向)
            1: None,        # 舵机2 -> 忽略(重复)
            2: (1, 1.0),    # 舵机3 -> 关节2 (正向)
            3: None,        # 舵机4 -> 忽略(重复反向)
            4: (2, 1.0),    # 舵机5 -> 关节3 (正向)
            5: None,        # 舵机6 -> 忽略(重复反向)
            6: (3, 1.0),    # 舵机7 -> 关节4 (正向)
            7: (4, 1.0),    # 舵机8 -> 关节5 (正向)
            8: (5, 1.0),    # 舵机9 -> 关节6 (正向)
        }
        
        # 初始化关节角度数组
        joint_values = [0.0] * 6
        servo_values = []
        
        # 处理9个舵机数据
        for i in range(9):
            # 数据索引计算
            byte_idx = 3 + i * 2
            if byte_idx + 1 >= len(frame):
                logger.warning(f"舵机数据越界: 索引{byte_idx}超出范围")
                continue
            
            # 解析舵机原始值
            low_byte = frame[byte_idx]
            high_byte = frame[byte_idx + 1]
            servo_value = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8)
            servo_values.append(servo_value)
            
            # 映射到关节
            mapping = servo_to_joint_map.get(i)
            if mapping is not None:
                joint_idx, direction = mapping
                # 转换为弧度并应用方向系数
                angle_rad = self._value_to_radians(servo_value) * direction
                joint_values[joint_idx] = angle_rad
        
        # 更新存储的数据
        self._update_joint_state(angles=joint_values)
        
        if self.debug_mode:
            degrees = [round(rad * self.RAD_TO_DEG, 2) for rad in joint_values]
            logger.debug(f"关节角度(度): {degrees}")
        
        return {
            "type": "joint_data",
            "angles": self._joint_states.angles,
            "servo_values": servo_values,
            "timestamp": self._joint_states.timestamp
        }

    def _value_to_radians(self, value: int) -> float:
        """
        将舵机值转换为弧度值 - 与ROS代码保持一致
        
        Args:
            value: 舵机值(0-4095)
            
        Returns:
            float: 弧度值
        """
        try:
            # 值范围检查
            if value < 0 or value > 4095:
                logger.warning(f"舵机值超出范围: {value} (有效范围0-4095)")
                value = max(0, min(value, 4095))
            
            # 转换为角度: -180到+180度
            # 使用与ROS代码一致的转换公式
            angle_deg = -180.0 + (value / 2048.0) * 180.0
            
            # 转换为弧度并返回
            return angle_deg * self.DEG_TO_RAD
                
        except Exception as e:
            logger.error(f"值转换异常: {str(e)}")
            return 0.0
    
    def _parse_gripper_data_old(self, frame: List[int]) -> Dict:
        """
        解析夹爪数据帧 (0x02)
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 解析按钮状态 (如果数据帧中包含)
        button1 = False
        button2 = False
        if len(frame) >= 10:  # 确保有足够的数据
            # 假设第8、9个字节包含按钮状态信息
            button1 = (frame[8] & 0x01) != 0
            button2 = (frame[9] & 0x01) != 0

        # 检查最小长度
        if len(frame) < 8:
            logger.warning("夹爪数据帧长度不足")
            return None
        
        if button1:
             gripper_raw = frame[6] | (frame[7] << 8)
        # 从字节4-5提取夹爪角度
        else:
            gripper_raw = frame[4] | (frame[5] << 8)

        if gripper_raw < 2048 or gripper_raw > self.servo_value_limit:
            gripper_raw = max(2048, min(gripper_raw, self.servo_value_limit))
        
        # # 转换为0-100范围
        # # 反向映射：2048(硬件打开) → 100, servo_value_limit(硬件关闭) → 0
        ratio = (self.servo_value_limit - 2048) / 100
        gripper_value = 100 - ((gripper_raw - 2048) / ratio)

        # 更新存储的数据
        self._update_joint_state(gripper=gripper_value, button1=button1,
                                 button2=button2)
        

        return {
            "type": "gripper_data",
            "gripper_angle": self._joint_states.gripper,
            "button1": self._joint_states.button1,
            "button2": self._joint_states.button2,
            "timestamp": self._joint_states.timestamp
        }
    



    def _parse_gripper_data(self, frame: List[int], robot_type: str = "follower") -> Dict:
        """
        解析夹爪数据帧 (0x12)
        
        协议格式:
        | 0xAA | 0x12 | 0x07 | 0x01 | range_low | range_high | pot_low | pot_high | sync_btn | attitude_btn | checksum | 0xFF |
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 检查最小长度 (需要至少12字节: header + cmd + len + suite + 4 data + 2 buttons + checksum + footer)
        if len(frame) < 12:
            logger.warning(f"夹爪数据帧长度不足: 需要至少12字节，实际{len(frame)}字节")
            return None
        
        # 验证帧头
        if frame[0] != 0xAA:
            logger.warning(f"夹爪数据帧头错误: 期望0xAA，实际0x{frame[0]:02X}")
            return None
        
        # 验证命令ID
        if frame[1] != self.CMD_GRIPPER_V6:
            logger.warning(f"夹爪数据帧命令ID错误: 期望0x{self.CMD_GRIPPER_V6:02X}，实际0x{frame[1]:02X}")
            return None
        
        # 验证帧尾
        if frame[-1] != 0xFF:
            logger.warning(f"夹爪数据帧尾错误: 期望0xFF，实际0x{frame[-1]:02X}")
            return None
        
        # 解析夹爪范围值 (字节4-5，用于验证，应与发送值相同)
        gripper_range_low = frame[4]
        gripper_range_high = frame[5]
        gripper_raw = gripper_range_low | (gripper_range_high << 8)
        
        # 解析电位计值 (字节6-7，这是实际的夹爪位置值)
        potentiometer_low = frame[6]
        potentiometer_high = frame[7]
        potentiomete_raw = potentiometer_low | (potentiometer_high << 8)
        
        button1 = frame[8]  # 同步按键状态
        button2 = frame[9]  # 姿态按键状态
        if robot_type == "leader" or button1:
            gripper_raw = potentiomete_raw

        ratio = (self.servo_value_limit - 2048) / 100
        gripper_value = 100 - ((gripper_raw - 2048) / ratio)
        # clip gripper_value to 0-100, round to 2 decimal places
        gripper_value = round(max(0, min(gripper_value, 100)), 2)
        self._update_joint_state(gripper=gripper_value, button1=button1,
                                 button2=button2)
        
        if self.debug_mode:
            logger.debug(f"夹爪数据: 范围值={gripper_raw}, 位置值={gripper_raw}, "
                        f"同步按键={'按下' if button1 else '未按下'}, "
                        f"姿态按键={'按下' if button2 else '未按下'}")
        
        return {
            "type": "gripper_data",
            "gripper_angle": self._joint_states.gripper,
            "gripper_raw": gripper_raw,
            "button1": self._joint_states.button1,
            "button2": self._joint_states.button2,
            "timestamp": self._joint_states.timestamp
        }
    



    def _parse_error_data(self, frame: List[int]) -> Dict:
        """
        解析错误数据帧 (0xEE)
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 检查最小长度
        if len(frame) < 7:
            logger.warning("错误数据帧长度不足")
            return None
        
        # 提取错误码和附加信息
        error_code = frame[3]
        error_param = frame[4]
        
        error_types = {
            0x00: "包头/包尾或长度错误",
            0x01: "校验错误",
            0x02: "模式错误",
            0x03: "ID无效",
        }
        
        error_message = error_types.get(error_code, f"未知错误(0x{error_code:02X})")
        
        logger.warning(f"设备错误: {error_message}, 参数: 0x{error_param:02X}")
        
        return {
            "type": "error_data",
            "error_code": error_code,
            "error_param": error_param,
            "error_message": error_message,
            "timestamp": time.time()
        }
    
    def _parse_version_data(self, frame: List[int]) -> Dict:
        """
        解析版本数据帧 (0x12)
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 提取版本信息
        version_str = f"{frame[3]}.{frame[4]}.{frame[5]}"
        
        # 存储版本信息
        with self._lock:
            self._firmware_version = version_str
        
        return {
            "type": "version_data",
            "version": version_str,
            "timestamp": time.time()
        }
    
    def _bytes_to_radians(self, byte_array: List[int]) -> float:
        """
        将字节数组转换为弧度值
        
        Args:
            byte_array: 2字节数组
            
        Returns:
            float: 弧度值
        """
        try:
            if len(byte_array) != 2:
                logger.warning(f"数据长度错误：需要2个字节，实际{len(byte_array)}个字节")
                return 0.0
            
            # 构造16位整数
            hex_value = (byte_array[0] & 0xFF) | ((byte_array[1] & 0xFF) << 8)
            
            # 值范围检查
            if hex_value < 0 or hex_value > 4095:
                logger.warning(f"舵机值超出范围: {hex_value} (有效范围0-4095)")
                hex_value = max(0, min(hex_value, 4095))
            
            # 转换为角度: -180到+180度
            angle_deg = -180.0 + (hex_value / 2048.0) * 180.0
            
            # 转换为弧度并返回
            return angle_deg * self.DEG_TO_RAD
                
        except Exception as e:
            logger.error(f"字节转换异常: {str(e)}")
            return 0.0
    
    def _verify_checksum(self, frame: List[int]) -> bool:
        """
        验证帧的校验和
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            bool: 校验是否通过
        """
        if len(frame) < 4:
            return False
        
        # 计算从第3个字节到倒数第3个字节的所有元素之和
        checksum = 0
        for i in range(3, len(frame) - 2):
            checksum += frame[i]
        
        checksum %= 2
        received_checksum = frame[-2]
        
        return checksum == received_checksum
    
    def _bytes_to_hex(self, data: List[int]) -> str:
        """
        将字节数组转换为十六进制字符串
        
        Args:
            data: 字节数组
            
        Returns:
            str: 十六进制字符串
        """
        return " ".join([f"{b:02X}" for b in data])