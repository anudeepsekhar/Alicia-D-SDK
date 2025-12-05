"""
Demo: Multi-point Cartesian trajectory planning

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Demonstrates how to perform multi-point trajectory planning in Cartesian space.
Supports manual drag teaching to record multiple waypoints.
"""

import argparse
import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger
from alicia_d_sdk.execution import CartesianWaypointPlanner


def main(cmd_args):
    """Demonstrate multi-point Cartesian trajectory planning.
    """
    
    # Initialize and connect to the robot
    robot = alicia_d_sdk.create_robot(port=cmd_args.port)
    
    if not robot.connect():
        logger.error("Unable to connect to the robot")
        return
    
    try:
        # Create Cartesian waypoint controller
        planner = CartesianWaypointPlanner(robot)
        
        # Move to initial position
        logger.info("\n1. Moving to initial position...")
        robot.set_home()
        
        # Record waypoints (manual drag mode)
        waypoints = planner.record_teaching_waypoints()
        
        if not waypoints:
            logger.error("No waypoints recorded, exiting demo")
            return
        
        # 选择执行模式
        step_by_step = input("\n选择执行模式:\n"
                            "1. 连续执行 (推荐)\n"
                            "2. 逐步执行\n"
                            "请输入选择 (1/2): ").strip() == "2"
        
        # Execute trajectory
        planner.execute_trajectory(
            waypoints=waypoints,
            speed_deg_s=cmd_args.speed_deg_s,
            move_duration=cmd_args.move_duration,
            num_points=cmd_args.num_points,
            ik_method=cmd_args.ik_method,
            step_by_step=step_by_step,
            step_delay=0.5 if step_by_step else 0.2
        )
        
        logger.info("\n✓ Demo completed!")
        
    except KeyboardInterrupt:
        logger.info("\nUser interrupted")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-Point Cartesian Trajectory Demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyCH343USB0 或 COM3)")
    parser.add_argument('--speed_deg_s', type=int, default=10,  help="关节运动速度 (单位: 度/秒，默认: 10，范围: 5-400度/秒)")
    # Trajectory planning settings
    parser.add_argument('--move_duration', type=float, default=3.0, help="每个路径点的移动时间 (秒, 默认: 3.0)")
    parser.add_argument('--num_points', type=int, default=200, help="轨迹插值点数 (默认: 150)")
    parser.add_argument('--ik_method', type=str, default='dls',
                       choices=['dls', 'pinv', 'lm'], help="逆运动学求解方法 (默认: dls)")
    parser.add_argument('--visualize', action='store_true', help="启用轨迹可视化")
    
    args = parser.parse_args()
    
    main(args)

