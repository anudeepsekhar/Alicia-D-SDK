# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
Demo: Robot torque control

Warning: 
- Ensure no obstacles around the robot arm before calibration
- When torque is disabled, manually support the robot arm
"""

import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger

def main(args):
    """Execute robot torque control.
    
    :param args: Command line arguments containing port
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(
        port=args.port,
    )
    
    try:
        logger.info("Please manually hold the robot arm.")
        logger.info("请托住机械臂以免其突然掉落。")
        input("Press Enter to disable torque...")
        robot.torque_control('off')
        
        input("Press Enter to re-enable torque...")
        robot.torque_control('on')
        logger.info("Torque re-enabled.")
        
    
    except Exception as e:
        print(f"✗ Error: {e}")
        
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot zero calibration program")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    args = parser.parse_args()

    main(args)
