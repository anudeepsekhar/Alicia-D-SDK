"""
SparkVis WebSocket Bridge for Real Robot Integration

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

This module provides WebSocket-based bidirectional communication between
SparkVis UI and real Alicia-D robots, enabling real-time synchronization
and data logging capabilities.
"""

import asyncio
import json
import os
import signal
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional

try:
    import websockets  # pip install websockets==13.1
except ImportError:
    print("请先安装 websockets: pip install websockets==13.1")
    raise


class SparkVisBridge:
    """WebSocket bridge for SparkVis UI and robot synchronization."""

    def __init__(
        self,
        robot,
        host: str = "localhost",
        port: int = 8765,
        output_file: Optional[str] = None,
        enable_robot_sync: bool = True,
        robot_sync_rate_hz: float = 50.0,
        log_source: str = "ui"  # ui | robot | both
    ):
        """Initialize SparkVis bridge.

        :param robot: Alicia-D robot instance
        :param host: WebSocket server host
        :param port: WebSocket server port
        :param output_file: CSV output file path (optional)
        :param enable_robot_sync: Enable robot->UI state broadcasting
        :param robot_sync_rate_hz: Robot state broadcast frequency in Hz
        :param log_source: Data logging source ('ui', 'robot', or 'both')
        """
        self.robot = robot
        self.host = host
        self.port = port
        self.enable_robot_sync = enable_robot_sync
        self.robot_sync_interval = 1.0 / max(1e-3, robot_sync_rate_hz)
        self.log_source = log_source.lower()

        # WebSocket clients
        self.websocket_connections = set()
        self._pending_robot_state: Optional[Dict[str, float]] = None

        # CSV logging
        self.output_file = output_file
        self.file_handle = None
        if self.output_file:
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
                self.file_handle = open(self.output_file, 'w')
                self.file_handle.write('timestamp,joint1,joint2,joint3,joint4,joint5,joint6,gripper\n')
                self.file_handle.flush()
                print(f"[Log] 数据记录启用: {self.output_file}")
            except Exception as e:
                print(f"[Log] 打开CSV失败: {e}")
                self.file_handle = None

        # graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"[Exit] 收到信号 {signum}，正在优雅关闭...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources."""
        if self.file_handle:
            try:
                self.file_handle.flush()
                self.file_handle.close()
                print("[Log] CSV 已保存")
            except Exception as e:
                print(f"[Log] 关闭CSV失败: {e}")
            finally:
                self.file_handle = None

    @staticmethod
    def _now_str(ts: Optional[float] = None) -> str:
        """Get formatted timestamp string."""
        return datetime.fromtimestamp(ts or time.time()).strftime('%H:%M:%S.%f')[:-3]

    def read_robot_state(self) -> Optional[Dict[str, float]]:
        """Read current robot joint states and gripper position."""
        try:
            joints = self.robot.get_joints()  # 6 rad
            gripper_rad = self.robot.get_gripper()  # rad
            if joints is None or gripper_rad is None:
                return None

            # Convert gripper to percentage [0..1] for UI
            try:
                gripper_pct = max(0.0, min(1.0, float(gripper_rad) / 100.0))
            except Exception:
                gripper_pct = 0.0

            return {
                "Joint1": float(joints[0]),
                "Joint2": float(joints[1]),
                "Joint3": float(joints[2]),
                "Joint4": float(joints[3]),
                "Joint5": float(joints[4]),
                "Joint6": float(joints[5]),
                "gripper": float(gripper_pct),
            }
        except Exception as e:
            print(f"[Robot] 读取状态失败: {e}")
            return None

    def apply_ui_joint_update(self, joint_values: Dict[str, float]):
        """Apply joint updates received from UI to robot."""
        try:
            joints_rad = [
                float(joint_values.get('Joint1', 0.0)),
                float(joint_values.get('Joint2', 0.0)),
                float(joint_values.get('Joint3', 0.0)),
                float(joint_values.get('Joint4', 0.0)),
                float(joint_values.get('Joint5', 0.0)),
                float(joint_values.get('Joint6', 0.0)),
            ]
            print(f"joints_rad: {joints_rad}")

            if getattr(self.robot, 'firmware_new', False):
                self.robot.set_joint_target(
                    target_joints=joints_rad,
                    joint_format='rad',
                )
            else:
                # Old firmware uses interpolation interface
                self.robot.set_joint_target_interplotation(
                    target_joints=joints_rad,
                    joint_format='rad',
                    speed_factor=1.0,
                )

            # Gripper percentage [0..1] → angle [0..100]
            if 'gripper' in joint_values:
                pct = max(0.0, min(1.0, float(joint_values['gripper'])))
                self.robot.set_gripper_target(value=pct * 100.0, wait_for_completion=False)

            # Log UI commands
            if self.file_handle and self.log_source in ("ui", "both"):
                row = f"{self._now_str()},{','.join(map(str, joints_rad))},{joint_values.get('gripper', 0.0)}\n"
                self.file_handle.write(row)
                self.file_handle.flush()
        except Exception as e:
            print(f"[Robot] 写入失败: {e}")

    async def broadcast_robot_state(self, joint_data: Dict[str, float]):
        """Broadcast robot state to all connected WebSocket clients."""
        if not self.websocket_connections:
            return
        message = {
            'type': 'robot_state_update',
            'joint_values': joint_data,
            'timestamp': time.time()
        }
        disconnected = set()
        for ws in self.websocket_connections.copy():
            try:
                await ws.send(json.dumps(message))
            except Exception:
                disconnected.add(ws)
        self.websocket_connections -= disconnected

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections and messages."""
        print(f"🔗 WebSocket连接: ws://{self.host}:{self.port}")
        self.websocket_connections.add(websocket)

        # Initial sync: push current robot state
        if self.enable_robot_sync:
            robot_state = self.read_robot_state()
            if robot_state:
                await websocket.send(json.dumps({
                    'type': 'robot_state_update',
                    'joint_values': robot_state,
                    'timestamp': time.time()
                }))
                print("📤 已发送当前机器人状态给新连接的客户端")

        # Periodic task: Robot → UI broadcasting
        async def periodic_robot_sender():
            while websocket in self.websocket_connections:
                if self.enable_robot_sync:
                    state = self.read_robot_state()
                    if state:
                        # Log robot state
                        if self.file_handle and self.log_source in ("robot", "both"):
                            row = f"{self._now_str()},{state['Joint1']},{state['Joint2']},{state['Joint3']},{state['Joint4']},{state['Joint5']},{state['Joint6']},{state['gripper']}\n"
                            self.file_handle.write(row)
                            self.file_handle.flush()
                        try:
                            await websocket.send(json.dumps({
                                'type': 'robot_state_update',
                                'joint_values': state,
                                'timestamp': time.time()
                            }))
                        except Exception:
                            break
                await asyncio.sleep(self.robot_sync_interval)

        sender_task = asyncio.create_task(periodic_robot_sender())

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get('type')

                    if msg_type == 'joint_update':
                        joint_values = data.get('joint_values', {})
                        ts = data.get('timestamp', time.time())
                        print(f"[UI→Robot] {self._now_str(ts)} 关节更新: {joint_values}")
                        self.apply_ui_joint_update(joint_values)

                        # Send acknowledgment
                        await websocket.send(json.dumps({
                            'type': 'joint_update_ack',
                            'timestamp': time.time(),
                            'success': True
                        }))

                    elif msg_type == 'request_robot_state':
                        state = self.read_robot_state()
                        if state:
                            await websocket.send(json.dumps({
                                'type': 'robot_state_update',
                                'joint_values': state,
                                'timestamp': time.time()
                            }))
                except json.JSONDecodeError as e:
                    print(f"❌ JSON解析错误: {e}")
                except Exception as e:
                    print(f"❌ 处理消息错误: {e}")
        except Exception:
            pass
        finally:
            if 'sender_task' in locals():
                sender_task.cancel()
            self.websocket_connections.discard(websocket)
            print("🔌 WebSocket断开")

    def start_server(self):
        """Start the WebSocket server."""
        print(f"🚀 启动 WebSocket 服务器: ws://{self.host}:{self.port}")
        print("⏹️  按 Ctrl+C 停止")

        async def server():
            async with websockets.serve(self.websocket_handler, self.host, self.port):
                await asyncio.Future()

        try:
            asyncio.run(server())
        except KeyboardInterrupt:
            print("🛑 服务器已停止")
        finally:
            self.cleanup()
