"""
Demo: Multi-Point Cartesian Trajectory Planning

展示如何在笛卡尔空间进行多点轨迹规划
支持手动拖动机械臂记录多个路径点
"""

import argparse
import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger
from alicia_d_sdk.execution import CartesianWaypointController


def main(cmd_args):
    """
    :param cmd_args: Command line arguments
    """
    logger.info("=== 多点笛卡尔轨迹规划演示 ===")
    
    # 初始化并连接机器人
    robot = alicia_d_sdk.create_robot(
        port=cmd_args.port,
        baudrate=cmd_args.baudrate,
        robot_version=cmd_args.version,
        gripper_type=cmd_args.gripper_type
    )
    
    if not robot.connect():
        logger.error("无法连接到机器人")
        return
    
    try:
        # 创建笛卡尔路径点控制器
        controller = CartesianWaypointController(robot)
        
        # 移动到初始位置
        logger.info("\n1. 移动到初始位置...")
        robot.set_joint_target([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 记录路径点（手动拖动模式）
        waypoints = controller.record_teaching_waypoints()
        
        if not waypoints:
            logger.error("没有记录到路径点，退出演示")
            return
        
        # 选择执行模式
        step_by_step = input("\n选择执行模式:\n"
                            "1. 连续执行 (推荐)\n"
                            "2. 逐步执行\n"
                            "请输入选择 (1/2): ").strip() == "2"
        
        # 执行轨迹
        controller.execute_trajectory(
            waypoints=waypoints,
            move_duration=cmd_args.move_duration,
            num_points=cmd_args.num_points,
            ik_method=cmd_args.ik_method,
            visualize=cmd_args.visualize,
            step_by_step=step_by_step,
            step_delay=0.5 if step_by_step else 0.2
        )
        
        logger.info("\n✓ 演示完成!")
        
    except KeyboardInterrupt:
        logger.info("\n用户中断")
    except Exception as e:
        logger.error(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-Point Cartesian Trajectory Demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机器人版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")
    
    
    # Trajectory planning settings
    parser.add_argument('--move_duration', type=float, default=3.0, help="每个路径点的移动时间 (秒, 默认: 3.0)")
    parser.add_argument('--num_points', type=int, default=150, help="轨迹插值点数 (默认: 150)")
    parser.add_argument('--ik_method', type=str, default='dls',
                       choices=['dls', 'pinv', 'lm'], help="逆运动学求解方法 (默认: dls)")
    parser.add_argument('--visualize', action='store_true', help="启用轨迹可视化")
    
    args = parser.parse_args()
    
    main(args)

