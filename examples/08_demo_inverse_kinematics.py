"""
Demo: Inverse kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Specify target end-effector pose
- Solve for joint angles
- Move to target position
"""

import numpy as np
import argparse
import alicia_d_sdk
from robocore.utils.beauty_logger import beauty_print_array, beauty_print

def main(args):
    """
    :param args: Command line arguments
    """
    robot = alicia_d_sdk.create_robot(port=args.port)

    if not robot.connect():
        print("✗ 连接失败，请检查串口设置")
        return
    
    if args.multi_start > 0:
        print(f"多起点尝试: {args.multi_start}")    

    ik_result = robot.set_pose_target(
        target_pose=args.end_pose,
        backend='numpy',
        method=args.method,
        display=True,
        tolerance=1e-4,
        max_iters=args.max_iters,
        multi_start=args.multi_start,
        use_random_init=args.use_random_init,
        speed_deg_s=args.speed_deg_s,
        execute=args.execute
    )
    
    print("\n" + "=" * 60)
    beauty_print("详细求解结果:")
    print(f"  成功: {ik_result['success']}")
    if ik_result['success']:
        print(f"   迭代次数: {ik_result['iters']}")
        print(f"  位置误差: {ik_result['pos_err']:.6e} m")
        print(f"  姿态误差: {ik_result['ori_err']:.6e} rad")
        beauty_print("关节角度 (弧度):")
        print(f"  q_ik = {beauty_print_array(ik_result['q'])}")
        beauty_print("关节角度 (角度):")
        print(f"  q_ik = {beauty_print_array(np.rad2deg(ik_result['q']))}")
        if ik_result['motion_executed']:
            beauty_print("✓ 机械臂已移动到目标位置")
        else:
            beauty_print("(未执行移动)")
    else:
        print(f"  错误信息: {ik_result.get('message', '未知错误')}")
    print("=" * 60 + "\n")
    
    # 如果执行了移动，回到初始位置
    if args.execute and ik_result['success']:
        print("\n返回初始位置...")
        robot.set_home(speed_deg_s=args.speed_deg_s)
    
    robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="逆向运动学示例")
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--speed_deg_s', type=int, default=10,  help="关节运动速度 (单位: 度/秒，默认: 10，范围: 5-400度/秒)")

    # IK Configuration
    parser.add_argument('--end-pose', type=float, nargs=7, 
                       default=[+0.28778, -0.16163, +0.36082, +0.037734, +0.781503, -0.303739, +0.543665], 
                       help='目标位姿 (7个浮点数: px py pz qx qy qz qw)')
    parser.add_argument('--method', type=str, default='dls', 
                       choices=['dls', 'pinv', 'transpose'],
                       help='IK方法: dls(阻尼最小二乘), pinv(伪逆), transpose(雅可比转置)')
    parser.add_argument('--max-iters', type=int, default=100,  help='最大迭代次数 (默认: 100)')
    parser.add_argument('--multi-start', type=int, default=0,  help='多起点尝试次数 (0=禁用, 建议: 5-10)')
    parser.add_argument('--use-random-init', action='store_true', help='使用随机初始值（默认使用当前关节角度）')
    parser.add_argument('--execute', action='store_true', help='执行移动到求解的位置')
    
    args = parser.parse_args()
    
    main(args)
