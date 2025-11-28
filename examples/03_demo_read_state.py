"""
Demo: Read and print robot state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read joint angles (radians or degrees)
- Read end-effector pose
- Read gripper state
- Support single or continuous printing
"""

import alicia_d_sdk


def main(args):
    """Read and print robot state.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type
    )
    
    try:
        # Connect to robot
        if not robot.connect():
            return
        
        # Print robot state once
        if args.single:
            robot.print_state(continuous=False, output_format=args.format)
        else:
            # Print robot state continuously
            robot.print_state(continuous=True, output_format=args.format)
        
    except KeyboardInterrupt:
        print("\n✗ Reading interrupted")
    
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot state")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    # Display settings
    parser.add_argument('--format', type=str, default='deg', choices=['rad', 'deg'], help="Angle display format: rad(radians) or deg(degrees)")
    parser.add_argument('--single', action='store_true',  help="Print state once (default: continuous print)")         
    
    args = parser.parse_args()
    
    main(args)
