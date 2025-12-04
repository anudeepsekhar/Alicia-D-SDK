"""
Hardware Layer - 硬件层

提供底层硬件驱动功能，包括串口通信、数据解析、硬件协议处理等。
"""

from alicia_d_sdk.hardware.servo_driver import ServoDriver
from alicia_d_sdk.hardware.serial_comm import SerialComm
from alicia_d_sdk.hardware.data_parser import DataParser, JointState

ArmController = ServoDriver

__all__ = [
    "ServoDriver",
    "SerialComm", 
    "DataParser",
    "JointState",
    "ArmController"  # Backward compatibility
]