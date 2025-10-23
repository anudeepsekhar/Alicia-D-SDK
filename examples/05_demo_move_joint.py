"""
Demo: 使用 move_to_joint_state 控制机械臂移动到目标关节位置

功能:
- 支持角度和弧度输入
- 自动关节角度插值
- 可调节运动速度
"""

import alicia_d_sdk
import time

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
            print("✗ 连接失败，请检查串口设置")
            return
                
        # Set target joint positions in degrees
        target_joints_deg = [-30, 30.0, 30.0, 20.0, -20.0, 10.0]
        robot.set_home()
        robot.set_joint_target(
            target_joints=target_joints_deg,
            joint_format='deg',
            speed_factor=args.speed
        )
        # time.sleep(2)
        robot.set_home()

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="机械臂运动控制示例")
    
    parser.add_argument('--port', type=str, default="/dev/ttyCH343USB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")
    
    args = parser.parse_args()
    main(args)