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
Demo: Inverse kinematics

Features:
- Specify target end-effector pose
- Solve for joint angles
- Move to target position
"""
import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import numpy as np
import argparse
import alicia_d_sdk
from robocore.utils.beauty_logger import beauty_print_array, beauty_print


def main(args):
    robot = alicia_d_sdk.create_robot(port=args.port,
                                      gripper_type=args.gripper_type,
                                      base_link=args.base_link,
                                      end_link=args.end_link,
                                      backend=args.backend)

    if not robot.is_connected():
        print("✗ 连接失败，请检查串口设置")
        return

    ik_result = robot.set_pose(
        target_pose=args.end_pose,
        backend=args.backend,
        method=args.method,
        pos_tol=args.pos_tol,
        ori_tol=args.ori_tol,
        max_iters=args.max_iters,
        num_initial_guesses=args.num_inits,
        initial_guess_strategy=args.init_strategy,
        initial_guess_scale=args.init_scale,
        random_seed=args.seed,
        speed_deg_s=args.speed_deg_s,
        execute=args.execute,
        force_execute=args.force_execute
    )
    
    beauty_print("详细求解结果", type="module")
    print(f"  成功: {ik_result['success']}")
    if ik_result['success']:
        print(f"  迭代次数: {ik_result['iters']}")
        print(f"  位置误差: {ik_result['pos_err']:.6e} m")
        print(f"  姿态误差: {ik_result['ori_err']:.6e} rad")
        if 'err_norm' in ik_result and ik_result['err_norm'] is not None:
            print(f"  总误差: {ik_result['err_norm']:.6e}")
        if 'computation_time' in ik_result:
            print(f"  计算时间: {ik_result['computation_time'] * 1000:.4f} ms")
        beauty_print("关节角度 (弧度):")
        print(f"  q_ik = {beauty_print_array(ik_result['q'])}")
        beauty_print("关节角度 (角度):")
        print(f"  q_ik = {beauty_print_array(np.rad2deg(ik_result['q']))}")
        if ik_result.get('motion_executed', False):
            beauty_print("✓ 机械臂已移动到目标位置")
        else:
            beauty_print("(未执行移动)")
    else:
        print(f"  迭代次数: {ik_result.get('iters', 0)}/{args.max_iters}")
        print(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m (容差: {args.pos_tol:.6e} m)")
        print(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad (容差: {args.ori_tol:.6e} rad)")
        print(f"  错误信息: {ik_result.get('message', '未知错误')}")
    print("=" * 60 + "\n")
    
    robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="逆向运动学示例")
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--speed_deg_s', type=int, default=10,  help="关节运动速度 (单位: 度/秒，默认: 10，范围: 5-400度/秒)")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪类型")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称, world 或 base_link等")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称, tool0 或 link6等")

    # IK Configuration
    parser.add_argument('--end-pose', type=float, nargs=7, 
                       default=[0.26336, -0.17054, +0.4051, -0.560276, +0.357632, +0.745837, +0.043783],
                       help='目标位姿 (7个浮点数: px py pz qx qy qz qw)')
    parser.add_argument('--method', type=str, default='dls', 
                       choices=['dls', 'pinv', 'transpose'],
                       help='IK方法: dls(阻尼最小二乘), pinv(伪逆), transpose(雅可比转置)')
    parser.add_argument('--max-iters', type=int, default=500,  help='最大迭代次数 (默认: 500)')
    parser.add_argument('--pos-tol', type=float, default=1e-3, help='位置容差 (默认: 1e-3)')
    parser.add_argument('--ori-tol', type=float, default=1e-3, help='姿态容差 (默认: 1e-3)')
    parser.add_argument('--num-inits', type=int, default=10,  help='初始猜测数量 (默认: 10)')
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
