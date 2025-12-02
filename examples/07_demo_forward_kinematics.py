"""
Demo: Forward kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read current joint angles
- Calculate end-effector pose
- Display position, rotation matrix, Euler angles, quaternion
"""

import numpy as np
import alicia_d_sdk
from robocore.utils.beauty_logger import beauty_print_array, beauty_print


def main(args):
    """Demonstrate forward kinematics.
    
    :param args: Command line arguments containing port, version, and gripper_type
    """
    # 创建机械臂实例
    robot = alicia_d_sdk.create_robot(port=args.port)
    
    robot_model = robot.robot_model
    if not robot.connect():
        return
    
    # 显示机器人模型信息
    robot_model.summary(show_chain=True)
    robot_model.print_tree(show_fixed=True)
    
    # 使用API的详细姿态获取方法
    pose_info = robot.get_pose()
    if pose_info is None:
        beauty_print("获取位姿失败")
        return
    
    # 提取各个组件
    position_fk = pose_info['position']
    rotation_fk = pose_info['rotation']
    euler_fk = pose_info['euler_xyz']
    quat_fk = pose_info['quaternion_xyzw']
    T_fk = pose_info['transform']
    
    # 显示结果
    beauty_print("End-Effector Position (m):")
    print(f"  p = {beauty_print_array(position_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, radians):")
    print(f"  rpy = {beauty_print_array(euler_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, degrees):")
    print(f"  rpy = {beauty_print_array(np.rad2deg(euler_fk))}")
    beauty_print("End-Effector Orientation (Quaternion xyzw):")
    print(f"  quat = {beauty_print_array(quat_fk, precision=6)}")
    # Add note about quaternion sign ambiguity
    quat_neg = -quat_fk
    print("  Note: q and -q represent the same rotation")
    print(f"  -quat = {beauty_print_array(quat_neg, precision=6)} (equivalent)")
    beauty_print("Rotation Matrix:")
    print(beauty_print_array(rotation_fk, precision=6))
    beauty_print("Homogeneous Transformation Matrix:")
    print(beauty_print_array(T_fk, precision=6))
    
    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="正向运动学示例")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="",   help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    
    args = parser.parse_args()
    

    main(args)
