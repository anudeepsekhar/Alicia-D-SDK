"""
Demo: Read robot firmware version

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0
"""

import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger

def main(args):
    """Read and print robot firmware version.

    :param args: Command line arguments containing port
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(port=args.port)

    try:
        # Connect to robot
        if not robot.connect():
            logger.error("Connection failed, please check serial port settings")
            return
        firmware_version = robot.get_version()

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
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    args = parser.parse_args()

    main(args)