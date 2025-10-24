"""
Demo: 机械臂零点校准程序

Updated for Alicia-D SDK v6.0.0 with RoboCore bridge

警告: 
- 校准前请确保机械臂周围无障碍物
- 校准时会关闭扭矩，请手动支撑机械臂
"""

import alicia_d_sdk


def main(args):
    """Execute robot zero calibration.
    
    :param args: Command line arguments containing port, baudrate, version, and gripper_type
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
            print("✗ Connection failed, please check serial port settings")
            return

        robot.zero_calibration()
        
    
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
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")
    
    args = parser.parse_args()

    
    main(args)
