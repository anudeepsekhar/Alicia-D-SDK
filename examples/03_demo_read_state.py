"""
Demo: 读取并打印机械臂状态

Updated for Alicia-D SDK v6.0.0 with RoboCore bridge

功能:
- 读取关节角度（弧度或角度）
- 读取末端位姿
- 读取夹爪状态
- 支持单次或连续打印
"""

import alicia_d_sdk


def main(args):
    """
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.version,
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
        print("Disconnected from robot")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot state")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")
    
    # Display settings
    parser.add_argument('--format', type=str, default='deg', choices=['rad', 'deg'], help="Angle display format: rad(radians) or deg(degrees)")
    parser.add_argument('--single', action='store_true',  help="Print state once (default: continuous print)")         
    
    args = parser.parse_args()
    
    main(args)
