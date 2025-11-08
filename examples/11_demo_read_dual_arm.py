"""
Demo: Read dual single-arm state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Connect to two single robotic arms via two serial ports
- Continuously read and display joint angles, gripper angles, and button states
- Support for dual-arm reading using multi-threading
"""

import argparse
import time
import threading
import serial
import serial.tools.list_ports
import os
import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger


class DualArmController:
    """双单臂控制器（两个独立的单臂）"""
    
    def __init__(self, debug_mode: bool = False):
        self.debug_mode = debug_mode
        self.robots = []
        self.stop_reading = threading.Event()
        self.print_lock = threading.Lock()  # 用于同步打印输出
    
    def find_ports(self) -> list:
        """查找所有可用的串口设备"""
        available_devices = []
        try:
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                # 支持 Linux (ttyUSB) 和 Windows (COM)
                if "ttyUSB" in port.device or "COM" in port.device:
                    if os.access(port.device, os.R_OK | os.W_OK):
                        available_devices.append(port.device)
                        logger.info(f"找到可用设备: {port.device}")
                else:
                    logger.warning(f"设备 {port.device} 不符合要求，跳过")
        except Exception as e:
            logger.error(f"列出端口时异常: {str(e)}")
            return []
        return available_devices
    
    def connect_all_robots(self, args):
        """连接双单臂（两个端口）"""
        # 如果指定了端口，使用指定的端口；否则自动查找前两个
        if args.port1 and args.port2:
            devices = [args.port1, args.port2]
            logger.info(f"使用指定的端口: {devices}")
        else:
            # 自动查找端口，但只取前两个
            devices = self.find_ports()
            if len(devices) < 2:
                print(f"✗ 需要两个端口，但只找到 {len(devices)} 个可用设备")
                if devices:
                    print(f"找到的设备: {devices}")
                return False
            devices = devices[:2]  # 只取前两个
            logger.info(f"自动找到两个端口: {devices}")
        
        # 获取每个机械臂的类型
        robot_types = [args.robot_type1, args.robot_type2]
        
        # 连接两个机械臂
        for i, device in enumerate(devices):
            try:
                robot_type = robot_types[i]
                robot = alicia_d_sdk.create_robot(
                    port=device,
                    baudrate=args.baudrate,
                    robot_version=args.robot_version,
                    robot_type=robot_type,
                    gripper_type=args.gripper_type,
                    debug_mode=self.debug_mode
                )
                
                if robot.connect():
                    self.robots.append(robot)
                    robot_type_name = "示教臂(leader)" if robot_type == "leader" else "操作臂(follower)"
                    logger.info(f"成功连接到机械臂 {i+1} ({robot_type_name}): {device}")
                else:
                    print(f"✗ 连接设备失败: {device}")
            except Exception as e:
                print(f"✗ 创建机械臂实例时出错 {device}: {e}")
                import traceback
                traceback.print_exc()
        
        # 确保连接了两个机械臂
        if len(self.robots) != 2:
            print(f"✗ 需要连接两个机械臂，但只连接了 {len(self.robots)} 个")
            return False
        
        return True
    
    def read_robot_data(self, robot, robot_id, robot_type_name, robot_type, output_format):
        """在单独的线程中读取单个机械臂的数据"""
        logger.info(f"机械臂 {robot_id} ({robot_type_name}) 开始读取数据")
        try:
            while not self.stop_reading.is_set():
                # 使用锁确保打印输出不会混乱
                with self.print_lock:
                    print(f"\n{'='*60}")
                    print(f"[机械臂 {robot_id} ({robot_type_name})]")
                    print(f"{'='*60}")
                    robot.print_state(continuous=False, output_format=output_format, robot_type=robot_type)
                    print(f"{'='*60}\n")
                time.sleep(0.1)  # 稍微增加间隔，避免输出过快
        except Exception as e:
            with self.print_lock:
                print(f"✗ 机械臂 {robot_id} ({robot_type_name}) 数据读取错误: {e}")
    
    def start_reading(self, args):
        """开始从所有机械臂读取数据"""
        if not self.robots:
            print("✗ 没有已连接的机械臂")
            return
        
        print("开始读取数据...")
        print("按 Ctrl+C 退出")
        print("=" * 50)
        
        # 获取每个机械臂的类型名称
        robot_types = [args.robot_type1, args.robot_type2]
        robot_type_names = []
        for robot_type in robot_types:
            if robot_type == "leader":
                robot_type_names.append("示教臂(leader)")
            else:
                robot_type_names.append("操作臂(follower)")
        
        # 获取输出格式，如果没有指定则默认为 'deg'
        output_format = getattr(args, 'format', 'deg')
        
        threads = []
        for i, robot in enumerate(self.robots):
            thread = threading.Thread(
                target=self.read_robot_data,
                args=(robot, i + 1, robot_type_names[i], robot_types[i], output_format),
                name=f"Robot{i+1}_Reader"
            )
            threads.append(thread)
            thread.start()
        
        try:
            # 等待用户中断
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n\n程序已停止")
        finally:
            # 停止所有线程
            self.stop_reading.set()
            for thread in threads:
                thread.join()
    
    def disconnect_all(self):
        """断开所有连接"""
        for i, robot in enumerate(self.robots):
            robot.disconnect()
            logger.info(f"机械臂 {i+1} 已断开连接")


def main(args):
    """主函数"""
    print("=== 双单臂数据读取示例 ===")
    
    dual_controller = DualArmController(debug_mode=args.debug)
    
    try:
        # 连接所有机械臂
        if not dual_controller.connect_all_robots(args):
            print("无法连接到机械臂，请检查连接")
            return
        
        # 开始读取数据
        dual_controller.start_reading(args)
    
    except Exception as e:
        print(f"✗ 程序执行错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 断开所有连接
        dual_controller.disconnect_all()
        print("所有连接已断开")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="读取双单臂状态（需要两个端口）")
    
    # 串口设置
    parser.add_argument('--port1', type=str, default="", help="第一个机械臂的串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--port2', type=str, default="", help="第二个机械臂的串口端口 (例如: /dev/ttyUSB1 或 COM4)")
    parser.add_argument('--baudrate', type=int, default=1000000, help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v5_6", help="机械臂版本 (默认: v5_6)")
    parser.add_argument('--robot_type1', type=str, default="follower", choices=['follower', 'leader'], 
                        help="第一个机械臂类型: follower(操作臂) 或 leader(示教臂) (默认: follower)")
    parser.add_argument('--robot_type2', type=str, default="follower", choices=['follower', 'leader'], 
                        help="第二个机械臂类型: follower(操作臂) 或 leader(示教臂) (默认: follower)")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--format', type=str, default='deg', choices=['rad', 'deg'], 
                        help="角度显示格式: rad(弧度) 或 deg(度数) (默认: deg)")
    parser.add_argument('--debug', action='store_true', help="启用调试模式")
    
    args = parser.parse_args()
    
    main(args)

