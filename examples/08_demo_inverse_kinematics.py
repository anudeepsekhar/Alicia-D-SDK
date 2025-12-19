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
import time
import alicia_d_sdk
import robocore as rc
from robocore.kinematics.ik import inverse_kinematics
from robocore.transform.conversions import quaternion_to_matrix
from robocore.utils.beauty_logger import beauty_print_array, beauty_print
from robocore.utils.backend import to_numpy


def main(args):
    robot = alicia_d_sdk.create_robot(port=args.port,
                                      robot_version=args.robot_version,
                                      gripper_type=args.gripper_type,
                                      base_link=args.base_link,
                                      end_link=args.end_link)

    # Set backend
    rc.set_backend(args.backend)

    # Get robot model
    robot_model = robot.robot_model

    # Get initial guess
    if args.init_strategy == 'current':
        q0 = robot.get_joints()
        if q0 is None:
            print("✗ 无法获取当前关节角度")
            robot.disconnect()
            return
        # Use 'random' strategy with current joints as base
        actual_strategy = 'random'
        print(f"当前关节角度 (弧度): {beauty_print_array(q0)}")
        print(f"当前关节角度 (角度): {beauty_print_array(np.rad2deg(q0))}")
    else:
        q0 = None
        actual_strategy = args.init_strategy

    # Convert pose to transformation matrix
    T_target = np.zeros((4, 4))
    T_target[:3, 3] = args.end_pose[:3]
    T_target[3, 3] = 1.0
    T_target[:3, :3] = quaternion_to_matrix(args.end_pose[3:])

    print(f"\n目标位姿:")
    print(f"  位置: [{args.end_pose[0]:.6f}, {args.end_pose[1]:.6f}, {args.end_pose[2]:.6f}] m")
    print(f"  四元数: [{args.end_pose[3]:.6f}, {args.end_pose[4]:.6f}, {args.end_pose[5]:.6f}, {args.end_pose[6]:.6f}]")

    if args.num_inits > 1:
        print(f"\n多起点尝试: {args.num_inits}, 策略: {actual_strategy}")
    elif args.init_strategy == 'current':
        print(f"\n使用当前关节角度作为初始猜测")

    # Solve inverse kinematics
    start_time = time.time()
    ik_result = inverse_kinematics(
        robot_model,
        T_target,
        q0,
        method=args.method,
        max_iters=args.max_iters,
        pos_tol=args.tolerance,
        ori_tol=args.tolerance,
        num_initial_guesses=args.num_inits,
        initial_guess_strategy=actual_strategy,
        initial_guess_scale=args.init_scale,
        random_seed=args.seed,
        use_analytic_jacobian=True
    )
    elapsed_time = time.time() - start_time

    # Convert to numpy
    q_ik = to_numpy(ik_result['q'])
    
    beauty_print("详细求解结果", type="module")
    print(f"  成功: {ik_result['success']}")

    # Extract error information (handle both single value and list)
    iters = ik_result.get('iters', 0)
    pos_err = ik_result.get('pos_err', float('inf'))
    ori_err = ik_result.get('ori_err', float('inf'))
    err_norm = ik_result.get('err_norm', None)

    # Handle list results (from batch processing)
    if isinstance(iters, list):
        iters = iters[0] if iters else 0
    if isinstance(pos_err, list):
        pos_err = pos_err[0] if pos_err else float('inf')
    if isinstance(ori_err, list):
        ori_err = ori_err[0] if ori_err else float('inf')
    if isinstance(err_norm, list):
        err_norm = err_norm[0] if err_norm else None

    if args.force_execute:
        print(f"  迭代次数: {iters}")
        print(f"  位置误差: {pos_err:.6e} m")
        print(f"  姿态误差: {ori_err:.6e} rad")
        if err_norm is not None:
            print(f"  总误差: {err_norm:.6e}")
        print(f"  计算时间: {elapsed_time * 1000:.4f} ms")
        beauty_print("关节角度 (弧度):")
        print(f"  q_ik = {beauty_print_array(q_ik)}")
        beauty_print("关节角度 (角度):")
        print(f"  q_ik = {beauty_print_array(np.rad2deg(q_ik))}")

        # Execute motion if requested
        if args.execute:
            print("\n执行移动到目标位置...")
            success = robot.set_robot_target(
                target_joints=q_ik,
                joint_format='rad',
                speed_deg_s=args.speed_deg_s,
                wait_for_completion=True
            )
            if success:
                beauty_print("✓ 机械臂已移动到目标位置")
            else:
                beauty_print("✗ 移动失败")
        else:
            beauty_print("(未执行移动)")
    else:
        # Failed case - show detailed error information
        print(f"  迭代次数: {iters}/{args.max_iters}")
        print(f"  位置误差: {pos_err:.6e} m (容差: {args.tolerance:.6e} m)")
        print(f"  姿态误差: {ori_err:.6e} rad (容差: {args.tolerance:.6e} rad)")
        if err_norm is not None:
            print(f"  总误差: {err_norm:.6e}")
        print(f"  计算时间: {elapsed_time * 1000:.4f} ms")

        # Provide suggestions
        print("\n  可能的原因:")
        if iters >= args.max_iters:
            print("    - 达到最大迭代次数，可能目标位姿不可达或初始猜测不佳")
        if pos_err > args.tolerance * 10:
            print(f"    - 位置误差较大 ({pos_err:.6e} m)，目标位置可能超出工作空间")
        if ori_err > args.tolerance * 10:
            print(f"    - 姿态误差较大 ({ori_err:.6e} rad)，目标姿态可能不可达")

        print("\n  建议:")
        print("    - 尝试增加 --num-inits 参数（例如: --num-inits 10）")
        print("    - 尝试不同的 --init-strategy（例如: --init-strategy random）")
        print("    - 尝试增加 --max-iters 参数")
        print("    - 检查目标位姿是否在工作空间内")

        if 'message' in ik_result:
            print(f"\n  错误信息: {ik_result['message']}")
    print("=" * 60 + "\n")
    
    robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="逆向运动学示例")
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--speed_deg_s', type=int, default=20,  help="关节运动速度 (单位: 度/秒，默认: 10，范围: 5-400度/秒)")
    parser.add_argument('--robot_version', type=str, default="v5_6", help="机器人版本")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪类型")
    parser.add_argument('--model_format', type=str, default="urdf", help="模型格式")
    parser.add_argument('--base_link', type=str, default="world", help="基座链路名称, world 或 base_link等")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称, tool0 或 Link6等")

    # IK Configuration
    parser.add_argument('--end-pose', type=float, nargs=7, 
                        default=[-0.2, -0.3, +0.3, +0.707, 0, -0.707, 0],
                       help='目标位姿 (7个浮点数: px py pz qx qy qz qw)')
    parser.add_argument('--method', type=str, default='dls', 
                       choices=['dls', 'pinv', 'transpose'],
                       help='IK方法: dls(阻尼最小二乘), pinv(伪逆), transpose(雅可比转置)')
    parser.add_argument('--max-iters', type=int, default=500,  help='最大迭代次数 (默认: 100)')
    parser.add_argument('--tolerance', type=float, default=1e-4, help='位置和姿态容差 (默认: 1e-4)')
    parser.add_argument('--num-inits', type=int, default=5,  help='初始猜测数量 (默认: 1)')
    parser.add_argument('--init-strategy', type=str, default='current',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform', 'current'],
                        help='初始猜测策略 (默认: current=使用当前关节角度)')
    parser.add_argument('--init-scale', type=float, default=1.0,
                        help='关节限制缩放因子 (0.0到1.0, 默认: 1.0)')
    parser.add_argument('--seed', type=int, default=None,
                        help='随机种子 (默认: None)')
    parser.add_argument('--backend', type=str, default='numpy',
                        choices=['numpy', 'torch'],
                        help='计算后端 (默认: numpy)')
    parser.add_argument('--execute', action='store_true', help='执行移动到求解的位置')
    parser.add_argument('--force-execute', action='store_true', help='强制执行移动到求解的位置，不考虑是否成功')
    
    args = parser.parse_args()
    
    main(args)
