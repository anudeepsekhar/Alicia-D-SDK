import os
import json
import time
import threading
from typing import List, Dict, Any, Optional, Callable
from datetime import datetime
import numpy as np


def record_waypoints_manual(controller, 
                            get_state_fn: Optional[Callable] = None,
                            format_fn: Optional[Callable] = None) -> List[Any]:
    """
    通用手动路径点记录函数
    
    Args:
        controller: 机器人控制器
        get_state_fn: 自定义状态获取函数，返回要记录的数据。如果为None，使用默认的关节+夹爪
        format_fn: 自定义格式化函数，用于日志输出。如果为None，使用默认格式
        
    Returns:
        记录的路径点列表
    """
    print("\n=== 手动记录模式 ===")
    print("关闭扭矩后，拖动到目标位置按回车记录")
    
    input("按回车开始...")
    controller.torque_control('off')
    print("[安全] 扭矩已关闭，可以拖动机械臂")
    
    waypoints = []
    
    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break
            
            # 使用自定义或默认的状态获取函数
            if get_state_fn:
                state = get_state_fn(controller)
            else:
                # 默认：记录关节角度和夹爪状态
                joints = controller.get_joints()
                try:
                    gripper = float(controller.get_gripper())
                except:
                    gripper = 0.0
                state = {"t": time.time(), "q": joints, "grip": gripper}
            
            if state:
                waypoints.append(state)
                
                # 使用自定义或默认的格式化函数输出日志
                if format_fn:
                    print(format_fn(len(waypoints), state))
                else:
                    # 默认格式
                    if isinstance(state, dict) and 'q' in state:
                        print(f"[记录] 第{len(waypoints)}个点: 关节{[round(j, 3) for j in state['q']]}, 夹爪{state.get('grip', 0):.3f}")
                    else:
                        print(f"[记录] 第{len(waypoints)}个点")
                        
    finally:
        controller.torque_control('on')
        print("[安全] 扭矩已重新开启")
        
    return waypoints


class SimpleDragTeaching:
    """简化的拖动示教类 - 直接记录关节状态"""
    
    def __init__(self, args, controller):
        self.args = args
        self.controller = controller

    def setup(self):
        """初始化设置"""
        if self.args.save_motion:
            print(f"动作名: {self.args.save_motion}")
        if self.args.mode == 'auto':
            print(f"采样频率: {self.args.sample_hz} Hz")
        print("=" * 30)
        
    def manual_mode(self) -> List[Dict[str, Any]]:
        """手动模式 - 记录关键点"""
        return record_waypoints_manual(self.controller)
        
    def auto_mode(self) -> List[Dict[str, Any]]:
        """自动模式 - 连续记录"""
        print("\n=== 自动模式 ===")
        print("关闭扭矩后拖动机械臂，系统自动记录轨迹")
        
        input("按回车开始...")
        self.controller.torque_control('off')
        print("[安全] 扭矩已关闭，可以拖动机械臂")
        
        # 记录变量
        trajectory = []
        recording = threading.Event()
        
        def record_loop():
            """后台记录线程"""
            dt = 1.0 / self.args.sample_hz
            start_time = time.time()
            
            while recording.is_set():
                try:
                    current_time = time.time() - start_time
                    joints = self.controller.get_joints()
                    try:
                        gripper = float(self.controller.get_gripper())
                    except:
                        gripper = 0.0
                        
                    point = {
                        "t": current_time,
                        "q": joints,
                        "grip": gripper
                    }
                    trajectory.append(point)
                    
                except Exception as e:
                    print(f"[警告] 记录失败: {e}")
                    
                time.sleep(dt)
        
        try:
            input("开始拖动，按回车开始记录...")
            recording.set()
            thread = threading.Thread(target=record_loop, daemon=True)
            thread.start()
            print(f"[记录中] 频率 {self.args.sample_hz} Hz...")
            
            input("按回车停止记录...")
            recording.clear()
            thread.join(timeout=1.0)
            
        finally:
            self.controller.torque_control('on')
            print("[安全] 扭矩已重新开启")
            
        print(f"[完成] 记录了 {len(trajectory)} 个点")
        return trajectory
    
    def replay_only_mode(self) -> Optional[List[Dict[str, Any]]]:
        """仅回放模式 - 加载已有数据"""
        print("\n=== 仅回放模式 ===")
        
        # 检查动作名是否提供
        if not self.args.save_motion:
            print("[错误] 回放模式需要指定动作名")
            print("请使用 --save-motion <motion_name> 参数")
            print("或使用 --list-motions 查看可用动作")
            return None
        
        # 加载数据
        save_dir = os.path.join("example_motions", self.args.save_motion)
        traj_path = os.path.join(save_dir, "joint_traj.json")
        meta_path = os.path.join(save_dir, "meta.json")
        
        # 检查目录和文件是否存在
        if not os.path.exists(save_dir):
            print(f"[错误] 未找到动作目录: {save_dir}")
            print("\n可用的动作:")
            available = list_available_motions()
            if available:
                for motion in available[:5]:  # 只显示前5个
                    print(f"  - {motion}")
                if len(available) > 5:
                    print(f"  ... 还有 {len(available)-5} 个动作")
                print(f"\n使用 --list-motions 查看完整列表")
            else:
                print("  未找到任何已录制的动作")
                print("  请先使用 auto 或 manual 模式录制动作")
            return None
            
        if not os.path.exists(traj_path):
            print(f"[错误] 未找到轨迹文件: {traj_path}")
            print("该动作可能损坏或不完整")
            return None
            
        if not os.path.exists(meta_path):
            print(f"[警告] 未找到元数据文件: {meta_path}")
            print("将使用默认设置继续加载")
        
        # 读取数据和元信息
        try:
            with open(traj_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"[错误] 读取轨迹文件失败: {e}")
            return None
            
        # 读取元信息
        meta = {}
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
            except Exception as e:
                print(f"[警告] 读取元数据失败: {e}")
        
        # 更新模式为原始记录模式
        original_mode = meta.get('mode', 'auto')
        print(f"[加载] 轨迹: {len(data)}个点")
        print(f"[加载] 原始模式: {original_mode}")
        print(f"[加载] 创建时间: {meta.get('created_at', 'Unknown')}")
        print(f"[加载] 采样频率: {meta.get('sample_hz', 'Unknown')} Hz")
        
        # 临时保存原始参数并设置为原始模式
        self._original_mode = self.args.mode
        self.args.mode = original_mode
        
        return data
        
    def save_data(self, data: List[Dict[str, Any]]) -> Optional[str]:
        """保存数据"""
        if not data:
            print("[保存] 没有数据")
            return None
            
        # 创建保存目录
        save_dir = os.path.join("example_motions", self.args.save_motion)
        os.makedirs(save_dir, exist_ok=True)
        
        # 保存关节轨迹
        traj_path = os.path.join(save_dir, "joint_traj.json")
        with open(traj_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            
        # 保存元信息
        meta = {
            "motion": self.args.save_motion,
            "mode": self.args.mode,  # 保存记录时的模式
            "created_at": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "sample_hz": self.args.sample_hz,
            "count": len(data),
            "description": "拖动示教轨迹"
        }
        
        meta_path = os.path.join(save_dir, "meta.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
            
        print(f"[保存] 轨迹: {traj_path}")
        print(f"[保存] 元数据: {meta_path}")
        
        return save_dir
        
    def replay(self, data: List[Dict[str, Any]]):
        """根据模式回放轨迹"""
        if not data:
            return
            
        print(f"\n=== 轨迹回放 ===")
        print(f"回放模式: {self.args.mode}")
        replay = input(f"是否回放轨迹（{len(data)}个点）？(y/n): ").strip().lower()
        if replay != 'y':
            return
            
        print("[回放] 开始...")
        
        if self.args.mode == 'auto':
            # 自动模式：使用直接设置，快速回放
            print("[回放] 使用直接设置模式（快速）")
            for i, point in enumerate(data):
                try:
                    # 获取夹爪值
                    gripper_value = point.get("grip", 0.0)
                    
                    # 使用combined control直接设置关节和夹爪，无插值
                    self.controller.servo_driver.set_joint_and_gripper(
                        joint_angles=point["q"],
                        gripper_value=gripper_value,
                        speed_deg_s=30.0  # 高速回放
                    )
                    
                    print(f"[回放] {i+1}/{len(data)}")
                    time.sleep(0.02)  # 20ms延时，快速回放
                    
                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")
                    
        elif self.args.mode == 'manual':
            # 手动模式：使用插值运动，平滑回放
            print("[回放] 使用插值运动模式（平滑）")
            for i, point in enumerate(data):
                try:
                    firmware_new = self.controller.firmware_new
                    if firmware_new:
                        self.controller.set_joint_target(point["q"])
                    else:
                        self.controller.set_joint_target_interplotation(point["q"], joint_format='rad', speed_factor=1.0, T_default=0.5, n_steps_ref=50)
                    
                    # 设置夹爪
                    gripper_value = point.get("grip", 0.0)
                    if gripper_value is not None:
                        try:
                            # 夹爪值已经是0-100范围，直接使用
                            self.controller.set_gripper_target(value=gripper_value, wait_for_completion=False)
                        except:
                            pass
                    
                    print(f"[回放] {i+1}/{len(data)}")
                    
                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")
        
        print("[回放] 完成")
    
    
    def run(self):
        """运行主程序"""
        try:
            # 对于replay_only模式，在setup之前先验证动作是否存在
            if self.args.mode == 'replay_only':
                if not self.args.save_motion:
                    print("[错误] 回放模式必须指定 --save-motion 参数")
                    print("使用 --list-motions 查看可用动作")
                    return
                
                # 检查动作是否存在
                save_dir = os.path.join("example_motions", self.args.save_motion)
                if not os.path.exists(save_dir):
                    print(f"[错误] 动作 '{self.args.save_motion}' 不存在")
                    print("\n提示:")
                    available = list_available_motions()
                    if available:
                        print("可用的动作:")
                        for motion in available[:3]:
                            print(f"  {motion}")
                        if len(available) > 3:
                            print(f"  ... 还有 {len(available)-3} 个")
                    else:
                        print("未找到任何已录制的动作，请先录制")
                    return
            
            self.setup()
            
            # 根据模式执行不同操作
            if self.args.mode == 'manual':
                data = self.manual_mode()
            elif self.args.mode == 'auto':
                data = self.auto_mode()
            elif self.args.mode == 'replay_only':
                data = self.replay_only_mode()
            else:
                raise ValueError(f"不支持的模式: {self.args.mode}")
                
            if data:
                # 只有非replay_only模式才保存数据
                if self.args.mode != 'replay_only':
                    save_dir = self.save_data(data)
                    
                    if save_dir:
                        print(f"\n[完成] 拖动示教完成！")
                        print(f"数据已保存到: {save_dir}")
                else:
                    print(f"\n[完成] 加载轨迹完成！")
                    
                # 回放
                self.replay(data)
                
                # 恢复原始模式（对于replay_only）
                if hasattr(self, '_original_mode'):
                    self.args.mode = self._original_mode
                    
            else:
                print("[完成] 未记录数据或加载失败")
                
        except KeyboardInterrupt:
            print("\n[中断] 用户中断")
        except Exception as e:
            print(f"[错误] 运行失败: {e}")




def list_available_motions() -> List[str]:
    """列出所有可用的动作"""
    motions_dir = "example_motions"
    if not os.path.exists(motions_dir):
        return []
    
    motions = []
    for item in os.listdir(motions_dir):
        motion_path = os.path.join(motions_dir, item)
        if os.path.isdir(motion_path):
            # 检查是否有轨迹数据
            traj_file = os.path.join(motion_path, "joint_traj.json")
            if os.path.exists(traj_file):
                motions.append(item)
    
    return sorted(motions)


def print_available_motions():
    """打印所有可用的动作"""
    motions = list_available_motions()
    
    print("=== 可用的动作列表 ===")
    if not motions:
        print("未找到任何已录制的动作")
        print("请先使用 auto 或 manual 模式录制动作")
        return
    
    print(f"在 example_motions/ 目录下找到 {len(motions)} 个动作:")
    
    for i, motion in enumerate(motions, 1):
        motion_dir = os.path.join("example_motions", motion)
        meta_path = os.path.join(motion_dir, "meta.json")
        
        # 读取动作信息
        info = f"{i:2d}. {motion}"
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
                mode = meta.get('mode', 'unknown')
                created = meta.get('created_at', 'unknown')
                count = meta.get('count', 0)
                info += f" (模式: {mode}, 点数: {count}, 创建: {created})"
            except:
                pass
        
        print(info)
    
    print(f"\n使用示例:")
    print(f"python 07_demo_drag_teaching.py --port /dev/ttyUSB0 --mode replay_only --save-motion {motions[0]}")
