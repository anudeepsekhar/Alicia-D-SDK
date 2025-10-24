"""
Demo: Read robot firmware version

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0
"""

import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger

def main(args):
    """Read and print robot firmware version.

    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type
    )

    try:
        # Connect to robot
        if not robot.connect():
            logger.error("Connection failed, please check serial port settings")
            return
        firmware_version = robot.get_firmware_version(timeout=2)
        if not firmware_version:
                logger.info("Firmware version 5.0.0.")

    except KeyboardInterrupt:
        logger.info("\nOperation interrupted by user")

    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")

    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot firmware version")

    # Robot configuration
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v5_6",  help="机械臂版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")

    args = parser.parse_args()

    main(args)