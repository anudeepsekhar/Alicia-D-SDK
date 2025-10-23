#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demo: 拖动示教 - 

功能特点：
- 关闭扭矩后拖动机械臂，直接记录关节角度
- 支持手动、自动和仅回放三种模式
- 根据记录模式使用不同的回放策略

# See what motions are available
python 09_demo_drag_teaching.py --list-motions

# Record new motions
python 09_demo_drag_teaching.py --port /dev/ttyUSB0 --mode auto --save-motion my_demo
python 09_demo_drag_teaching.py --port /dev/ttyUSB0 --mode manual --save-motion key_points

# Replay existing motions
python 09_demo_drag_teaching.py --port /dev/ttyUSB0 --mode replay_only --save-motion my_demo

# Get help
python 09_demo_drag_teaching.py --help
"""

import os
import json
import time
import argparse
import threading
import numpy as np
from typing import List, Dict, Any, Optional
from datetime import datetime

from alicia_d_sdk import create_robot

from alicia_d_sdk.execution.drag_teaching import SimpleDragTeaching
from alicia_d_sdk.execution.drag_teaching import print_available_motions, list_available_motions


def main(args):
    """Demonstrate drag teaching functionality.
    
    :param args: Command line arguments containing port, baudrate, version, gripper_type, mode, sample_hz, save_motion, and list_motions
    """
    
    # 如果用户请求列出动作，直接显示并退出
    if args.list_motions:
        print_available_motions()
        return
    
    # 验证参数
    if args.mode in ['manual', 'auto'] and not args.save_motion:
        print(f"[错误] {args.mode} 模式需要指定 --save-motion 参数")
        print("使用 --help 查看帮助信息")
        return
    
    if args.mode == 'replay_only' and not args.save_motion:
        print("[错误] 回放模式需要指定 --save-motion 参数")
        print("使用 --list-motions 查看可用动作")
        return

    robot = create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.version,
        gripper_type=args.gripper_type
    )

    if not robot.connect():
        print("无法连接到机器人")
        return

    drag_teaching = SimpleDragTeaching(args, robot)
    drag_teaching.run()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="拖动示教", 
                                   formatter_class=argparse.RawDescriptionHelpFormatter,)
    # Robot configuration
    parser.add_argument('--port', type=str, default="/dev/ttyCH343USB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")
    
    parser.add_argument('--mode', choices=['manual', 'auto', 'replay_only'], default='auto', 
                       help="模式: manual(手动插值) 或 auto(自动快速) 或 replay_only(仅回放)")
    parser.add_argument('--sample-hz', type=float, default=50.0, help="自动模式采样频率")
    parser.add_argument('--save-motion', help="动作名称 (录制模式: 新动作名; 回放模式: 已有动作名)")
    parser.add_argument('--list-motions', action='store_true',  help="列出所有可用的动作并退出")
    args = parser.parse_args()
    main(args)
