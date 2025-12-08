"""
Demo: Gripper control

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Open/close gripper
- Control gripper to specific angle
- Wait for gripper motion completion
"""

import alicia_d_sdk
import time
from alicia_d_sdk.utils.logger import logger

def main(args):
    """Demonstrate gripper control.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(port=args.port)
    
    try:
        # Connect to robot
        if not robot.connect():
            return
        
        # Get gripper value (0-1000)
        gripper_value = robot.get_gripper()
        if gripper_value is not None:
            logger.info(f"Gripper value: {gripper_value:.1f}")
        else:
            logger.warning("Failed to read gripper value")
        
        # Test 1: Open gripper
        robot.set_robot_target(gripper_value=1000)
        time.sleep(1)
        # Test 2: Close gripper
        robot.set_robot_target(gripper_value=0)
        time.sleep(1)
        # # Test 3: Partially open
        robot.set_robot_target(gripper_value=500)
        time.sleep(1)

        
    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    
    except Exception as e:
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Gripper Control Demo")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    args = parser.parse_args()
    
    
    main(args)
