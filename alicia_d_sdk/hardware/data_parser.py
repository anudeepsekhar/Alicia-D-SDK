import math
import time
from typing import List, Dict, Optional, NamedTuple, Union
import threading
import copy
from alicia_d_sdk.utils.logger import logger


class JointState(NamedTuple):
    """Joint state data structure."""
    angles: List[float]  # Six joint angles (radians)
    gripper: float       # Gripper value
    timestamp: float     # Timestamp (seconds)
    run_status_text: str # Run status text


class DataParser:
    """Robot arm data parsing module."""
    # Constants
    DEG_TO_RAD = math.pi / 180.0  # Degrees to radians
    RAD_TO_DEG = 180.0 / math.pi  # Radians to degrees
    
    # Command IDs
    CMD_GRIPPER = 0x04     # Gripper control and travel feedback
    CMD_GRIPPER_V6 = 0x12  # Gripper control for V6 firmware
    CMD_ZERO_POS = 0x03    # Set current position as zero
    CMD_JOINT = 0x06       # Joint angle feedback and control
    CMD_VERSION = 0x01     # Firmware version feedback
    CMD_TORQUE = 0x05      # Torque control
    CMD_ERROR = 0xEE       # Error feedback
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3600
    # Data sizes
    JOINT_DATA_SIZE = 18
    
    def __init__(self, lock: threading.Lock, debug_mode: bool = False, gripper_type: str = "50mm"):
        """
        Initialize data parser.
        
        Args:
            lock: Shared threading lock for concurrent access.
            debug_mode: Whether to enable debug logging.
            gripper_type: Gripper type, e.g. "50mm" or "100mm".
            robot_type: Robot role, e.g. "follower" or "leader".
        """
        self.debug_mode = debug_mode
        if gripper_type == "50mm":
            self.servo_value_limit = self.GRI_MAX_50MM
        else:
            self.servo_value_limit = self.GRI_MAX_100MM 
        # Store latest joint state
        self._joint_states = JointState([0.0]*6, 0.0, 0.0, "idle")

        self._firmware_version: Optional[str] = None
        # Full version information dict: serial, hardware, firmware
        self._version_info: Optional[Dict[str, str]] = None
        self._lock = lock
        
        # Store run status from joint data
        self._run_status: Optional[int] = None
        self._run_status_text: Optional[str] = None
        
        # Event-based synchronization for async data acquisition
        # Events are set when corresponding data is received and parsed
        self._version_event = threading.Event()
        self._joint_event = threading.Event()
        self._gripper_event = threading.Event()
        
        # Mapping from info type to corresponding event
        self._info_event_map = {
            "version": self._version_event,
            "joint": self._joint_event,
            "gripper": self._gripper_event,
        }
        
    
    def parse_frame(self, frame: List[int]) -> Optional[Dict]:
        """
        Parse a full data frame.
        
        Args:
            frame: Complete data frame (byte list)
        """
        cmd_id = frame[1]
        if cmd_id == self.CMD_VERSION:
            return self._parse_version_data(frame)
        elif cmd_id == self.CMD_JOINT:
            return self._parse_joint_data(frame)

        elif cmd_id == self.CMD_ERROR:
            return self._parse_error_data(frame)
        else:
            if self.debug_mode:
                logger.debug(f"Unhandled command ID: 0x{cmd_id:02X}")
            return None
    
    def get_joint_state(self) -> Optional[JointState]:
        """
        Get current joint state.
        """
        with self._lock:
            js = self._joint_states
            # print("self joint states:", js)
            if js.angles is None or js.timestamp is None:
                logger.warning("Robot state has not been updated yet")
                return None
            return copy.deepcopy(self._joint_states)
        

    
    def get_version_info(self) -> Optional[Dict[str, str]]:
        """
        Get full version information.
        
        Returns:
            Optional[Dict[str, str]]: Dictionary with keys:
                - 'serial_number'
                - 'hardware_version'
                - 'firmware_version'
            or None if not available.
        """
        with self._lock:
            if self._version_info is None:
                return None
           
            return dict(self._version_info)
    
    def wait_for_info(self, info_type: str, timeout: float = 2.0) -> bool:
        """
        Wait for specified info type to be received and parsed.
        
        Args:
            info_type: Type of information to wait for. Supported types:
                - "version": Wait for version info
                - "joint": Wait for joint state
                - "gripper": Wait for gripper state
            timeout: Maximum time to wait in seconds

        """
        if info_type not in self._info_event_map:
            raise ValueError(f"Unsupported info type: {info_type}. Supported types: {list(self._info_event_map.keys())}")
        
        event = self._info_event_map[info_type]
        return event.wait(timeout)
        


    def _update_joint_state(self,
                        angles: Optional[List[float]] = None,
                        gripper: Optional[float] = None,
                        run_status_text: Optional[str] = None):
        with self._lock:
            prev = self._joint_states
            self._joint_states = JointState(
                angles=angles if angles is not None else prev.angles,
                gripper=gripper if gripper is not None else prev.gripper,
                timestamp=time.time(),
                run_status_text=run_status_text if run_status_text is not None else prev.run_status_text
            )


    def _parse_joint_data(self, frame: List[int]) -> Dict:
        """
        Parse joint data frame.
        
        Args:
            frame: Complete data frame
        """

        # 灵动系列 DATA layout:
        #   - 6 joint angles (each 2 bytes, low byte first)
        #   - 1 gripper position (2 bytes)
        #   - 1 run status (1 byte)
        #
        # Total DATA length = 6*2 + 2 + 1 = 15 bytes (LEN should be >= 0x0F, example shows 0x10).

        # Basic length check: header(1)+CMD(1)+func(1)+LEN(1)+DATA(LEN)+checksum(1)+footer(1)

        data_len = frame[3]
        expected_min_len = 4 + data_len + 2  # header+cmd+func+LEN + DATA + checksum+footer
        if len(frame) < expected_min_len:
            logger.warning(f"Joint frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if len(data_bytes) < 15:
            logger.warning(f"Joint DATA too short: expect ≥15 bytes, got {len(data_bytes)}")
            return None

        # Parse 6 joint angles
        joint_values: List[float] = [0.0] * 6
        for i in range(6):
            start = i * 2
            joint_bytes = data_bytes[start:start + 2]
            angle_rad = self._bytes_to_radians(joint_bytes)
            joint_values[i] = angle_rad
            
        gripper_low = data_bytes[12]
        gripper_high = data_bytes[13]
        gripper_raw = (gripper_low & 0xFF) | ((gripper_high & 0xFF) << 8)

        # Map raw gripper value to 0–100 (same logic as V6 gripper parsing)
        ratio = (self.servo_value_limit - 2048) / 100
        gripper_value = 100 - ((gripper_raw - 2048) / ratio)
        gripper_value = round(max(0, min(gripper_value, 100)), 2)

        # Parse run status (last mandatory byte)
        run_status = data_bytes[14]
        run_status_map = {
            0x00: "idle",
            0x01: "locked",
            0x10: "sync",
            0x11: "sync_locked",
            0xE1: "overheat",
            0xE2: "overheat_protect",
        }
        run_status_text = run_status_map.get(run_status, "unknown")
        # print("run_status_text:", run_status_text)
        # Store run status
        with self._lock:
            self._run_status = run_status
            self._run_status_text = run_status_text

        # Update stored joint & gripper state
        self._update_joint_state(angles=joint_values, gripper=gripper_value, run_status_text=run_status_text)

        # Signal that joint state has been updated
        self._joint_event.set()

        if self.debug_mode:
            degrees = [round(rad * self.RAD_TO_DEG, 2) for rad in joint_values]
            logger.debug(
                f"Joint angles (deg): {degrees}, "
                f"gripper={gripper_value}, "
                f"run_status=0x{run_status:02X}({run_status_text})"
            )

        return {
            "type": "joint_data",
            "angles": self._joint_states.angles,
            "gripper": self._joint_states.gripper,
            "run_status": run_status,
            "run_status_text": run_status_text,
            "timestamp": self._joint_states.timestamp,
        }




    def _parse_error_data(self, frame: List[int]) -> Dict:
        """
        Parse error data frame (0xEE).
        
        Args:
            frame: Complete data frame
        """
        # Minimal length check
        if len(frame) < 7:
            logger.warning("Error frame too short")
            return None
        
        # Extract error code and parameter
        error_code = frame[3]
        error_param = frame[4]
        
        error_types = {
            0x00: "Header/footer or length error",
            0x01: "Checksum error",
            0x02: "Mode error",
            0x03: "Invalid ID",
        }
        
        error_message = error_types.get(error_code, f"Unknown error (0x{error_code:02X})")
        
        logger.warning(f"Device error: {error_message}, param: 0x{error_param:02X}")
        
        return {
            "type": "error_data",
            "error_code": error_code,
            "error_param": error_param,
            "error_message": error_message,
            "timestamp": time.time()
        }
    
    def _parse_version_data(self, frame: List[int]) -> Dict:
        """
        Parse version data frame (CMD=0x01).
        
        Protocol:
        | 0xAA | 0x01 | 0x00 | LEN | DATA... | CHECKSUM | 0xFF |
        
        DATA layout (LEN = 0x18 = 24 bytes):
          - Serial number: 16 ASCII bytes
          - Hardware version: 4 ASCII bytes
          - Firmware version: 4 ASCII bytes
        """
        # Basic length check: header(1)+CMD(1)+func(1)+LEN(1)+DATA(LEN)+checksum(1)+footer(1)
        if len(frame) < 4 + frame[3] + 2:
            logger.warning(f"Version frame too short: expect ≥{4 + frame[3] + 2}, got {len(frame)}")
            return None

        data_len = frame[3]
        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if data_len < 24:
            logger.warning(f"Version data length too short: expect 24, got {data_len}")
            return None

        # Split fields according to protocol
        serial_bytes = data_bytes[0:16]
        hardware_bytes = data_bytes[16:20]
        firmware_bytes = data_bytes[20:24]

        def _bytes_to_ascii(b: List[int]) -> str:
            try:
                return "".join(chr(x) for x in b).strip()
            except Exception as e:
                logger.error(f"Version ASCII parse exception: {e}")
                return ""

        serial_number = _bytes_to_ascii(serial_bytes)
        hardware_raw = _bytes_to_ascii(hardware_bytes)
        firmware_raw = _bytes_to_ascii(firmware_bytes)

        # Example: firmware_raw = "0610" -> "610" -> "6.1.0"
        compact = firmware_raw.lstrip("0") or "0"
        if len(compact) == 3:
            version_str = f"{compact[0]}.{compact[1]}.{compact[2]}"
        elif len(compact) == 2:
            version_str = f"{compact[0]}.{compact[1]}.0"
        elif len(compact) == 1:
            version_str = f"{compact[0]}.0.0"
        else:
            # Fallback: use raw string
            version_str = firmware_raw or "unknown"

        # Store firmware version (for upper-level API)
        with self._lock:
            self._firmware_version = version_str
            self._version_info = {
                "serial_number": serial_number,
                "hardware_version": hardware_raw,
                "firmware_version": version_str,
            }
        
        # Signal that version info has been received and parsed
        self._version_event.set()
        
        if self.debug_mode:
            logger.debug(
                f"Version parsed: SN='{serial_number}', HW='{hardware_raw}', FW_raw='{firmware_raw}', FW='{version_str}'"
            )

        return {
            "type": "version_data",
            "serial_number": serial_number,
            "hardware_version": hardware_raw,
            "firmware_version_raw": firmware_raw,
            "version": version_str,
            "timestamp": time.time(),
        }
    
    def _bytes_to_radians(self, byte_array: List[int]) -> float:
        """
        Convert 2-byte array (little endian) to radians directly.
        Mapping: 0 -> -π, 2048 -> 0, 4095 -> +π
        """
        if len(byte_array) != 2:
            logger.warning(f"Data length error: need 2 bytes, got {len(byte_array)}")
            return 0.0
        
        # Build 16-bit integer
        hex_value = (byte_array[0] & 0xFF) | ((byte_array[1] & 0xFF) << 8)
        
        # Range check
        if hex_value < 0 or hex_value > 4095:
            logger.warning(f"Servo value out of range: {hex_value} (valid 0–4095)")
            hex_value = max(0, min(hex_value, 4095))
        
        # Directly map raw value to radians: 0–4095 -> [-π, π]
        return (hex_value / 4096.0) * (2 * math.pi) - math.pi

    
    def _value_to_radians(self, value: int) -> float:
        """
        Convert servo raw value to radians directly.
        Mapping: 0 -> -π, 2048 -> 0, 4095 -> +π
        """
        if value < 0 or value > 4095:
            logger.warning(f"Servo value out of range: {value} (valid 0–4095)")
            value = max(0, min(value, 4095))
        
        # Directly map raw value to radians: 0–4095 -> [-π, π]
        return (value / 4096.0) * (2 * math.pi) - math.pi
                

