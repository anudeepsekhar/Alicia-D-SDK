"""
Demo: Control robot to move to target joint positions using move_to_joint_state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed
"""

import alicia_d_sdk
import time

def main(args):
    """Control robot joint movements.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(port=args.port)


    try:
        # Set target joint positions in degrees
        # target_joints_deg = [-30, 30.0, 30.0, 20.0, -20.0, 10.0]
        # target_joints_deg = [0.06577542, -0.64249217,  1.53825531,  0.00158657,  0.26565895, -1.26124511]
        # target_joints_deg = [-0.01132878, -0.52490978,  1.3053775,   1.02923886,  0.8045441,  -0.17470895]
        # target_joints_deg = [0.468795, 0.20364996, 2.23934348, 0.87907791, -0.14206953, 0.91907001]
        # target_joints_deg = [-0.93070936, -0.3446542,   1.20170948, -1.63267547,  0.85681103,  0.41842812]
        target_joints_deg = [-0.46670565, -0.73013471,  1.22830511,  0.20290779,  0.60581784,  0.90357157]
        robot.set_home(speed_deg_s=args.speed_deg_s)
        time.sleep(1)
        # Use unified joint and gripper target interface
        # robot.set_robot_target(
        #     target_joints=target_joints_deg,
        #     joint_format='rad',
        #     # joint_format='deg',
        #     speed_deg_s=args.speed_deg_s,
        #     wait_for_completion=True
        # )
        # time.sleep(1)
        # robot.set_home(speed_deg_s=args.speed_deg_s)

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="机械臂运动控制示例")
    
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--speed_deg_s', type=int, default=20,  help="关节运动速度 (单位: 度/秒，默认: 10，范围: 10-400度/秒)")
    args = parser.parse_args()
    main(args)