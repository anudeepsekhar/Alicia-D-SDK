#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
Demo: SparkVis UI bidirectional synchronization and data logging (Real Robot)

Features:
- Start WebSocket server for UI ↔ Robot bidirectional sync
- UI → Robot: Receive joint_update and send directly to real robot
- Robot → UI: Periodically broadcast current robot state to UI (toggleable)
- Data logging: Record joint data from UI commands to CSV (optional)

Usage:
1. Start SparkVis backend server:
   cd SparkVis
   python backend_server.py

2. Start SparkVis web server:
   cd SparkVis
   python -m http.server 8080

3. Run this demo (robot bridge):
   cd Alicia-D-SDK/examples
   python 10_demo_sparkvis.py --port /dev/ttyUSB0

4. Open browser and visit:
   http://localhost:8080

Note: All three components must be running simultaneously for full functionality.
"""

import alicia_d_sdk
from alicia_d_sdk.execution.sparkvis import SparkVisBridge


def main(args):
    """SparkVis WebSocket bridge demonstration.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(port=args.port)

    try:
        try:
            robot.set_home()
        except Exception:
            pass

        # Create and start SparkVis bridge
        bridge = SparkVisBridge(
            robot=robot,
            host=args.host,
            port=args.websocket_port,
            output_file=args.output_file or None,
            enable_robot_sync=args.enable_robot_sync,
            robot_sync_rate_hz=args.robot_sync_rate,
            log_source=args.log_source,
        )
        bridge.start_server()

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='SparkVis ↔ 真实机器人 同步与数据记录 Demo')
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default='', help='串口设备，如 /dev/ttyUSB0 或 COM3')
    # WebSocket settings
    parser.add_argument('--host', type=str, default='localhost', help='WebSocket主机')
    parser.add_argument('--websocket-port', type=int, default=8765, help='WebSocket端口')
    
    # Data logging settings
    parser.add_argument('--output-file', type=str, default='', help='CSV输出路径（留空不记录）')
    parser.add_argument('--log-source', type=str, default='ui', choices=['ui','robot','both'], help='记录UI指令/机器人状态/二者')
    # Robot sync settings
    parser.add_argument('--enable-robot-sync', action='store_true', help='启用 机器人→UI 状态同步')
    parser.add_argument('--robot-sync-rate', type=float, default=50.0, help='机器人状态广播频率 Hz')
    
    args = parser.parse_args()
    main(args)