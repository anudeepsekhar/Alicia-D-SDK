#!/usr/bin/env python3
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

"""Joint Space Trajectory Planning and Execution

This demo demonstrates:
1. Generating smooth joint space trajectories through multiple waypoints
2. Supporting random waypoint generation or loading from file
3. Executing the trajectory on the robot

Usage examples:
# Record waypoints manually by dragging the robot
python 09_demo_joint_traj.py --save-file my_waypoints.json

# Load waypoints from file and execute
python 09_demo_joint_traj.py --waypoints-file my_waypoints.json --execute

# Generate random waypoints and execute
python 09_demo_joint_traj.py  --num-waypoints 6 --execute

# Use B-Spline planner with custom settings
python 09_demo_joint_traj.py  --planner b_spline --duration 3.0 --num-points 1000 --execute

# Use multi-segment planner
python 09_demo_joint_traj.py  --planner multi_segment --duration-per-segment 1.5 --execute

# Get help
python 09_demo_joint_traj.py --help
"""

import numpy as np
import argparse

import alicia_d_sdk
from alicia_d_sdk.execution import JointTrajectoryExecutor
from robocore.utils.beauty_logger import beauty_print
from robocore.utils.backend import to_numpy

from alicia_d_sdk.utils.trajectory_utils import (
    handle_waypoint_recording,
    load_or_generate_joint_waypoints,
    display_joint_waypoints,
    display_joint_trajectory_stats,
    plot_trajectory
)


def main(args):
    """Main function for joint space trajectory planning and execution."""
    # [0] Initialize robot connection
    beauty_print("Joint Space Trajectory Planning", type="module")
    
    robot = alicia_d_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        base_link=args.base_link,
        end_link=args.end_link,
        backend=args.backend,
        device=args.device
    )
    robot_model = robot.robot_model

    # [1] Handle waypoint recording or loading/generation
    waypoints, gripper_waypoints = handle_waypoint_recording(robot, args, waypoint_type='joint')
    if waypoints is None:
        # Load or generate waypoints
        try:
            waypoints, gripper_waypoints = load_or_generate_joint_waypoints(robot, robot_model, args)
        except Exception as e:
            beauty_print(f"Failed to load/generate waypoints: {e}", type="error")
            robot.disconnect()
            return
    
    display_joint_waypoints(waypoints, gripper_waypoints)

    # [2] Generate trajectory
    beauty_print("[2] Generating Joint Space Trajectory", type="module", centered=False)
    
    planner_name = f"B-Spline (degree={args.bspline_degree})" if args.planner == 'b_spline' else f"Multi-Segment (method={args.segment_method})"
    beauty_print(f"Using {planner_name} planner")

    trajectory = robot.plan_joint_trajectory(
        waypoints=waypoints,
        planner_type=args.planner,
        duration=args.duration if args.planner == 'b_spline' else None,
        num_points=args.num_points if args.planner == 'b_spline' else None,
        bspline_degree=args.bspline_degree,
        segment_method=args.segment_method,
        duration_per_segment=args.duration_per_segment if args.planner == 'multi_segment' else None,
        num_points_per_segment=args.num_points_per_segment if args.planner == 'multi_segment' else None,
        gripper_waypoints=gripper_waypoints
    )

    display_joint_trajectory_stats(trajectory)
    gripper_trajectory = trajectory.get('gripper', None)

    # [3] Plot trajectory (optional)
    if args.plot:
        beauty_print("[3] Plotting Trajectory", type="module", centered=False)
        plot_trajectory(trajectory, waypoints, plot_type='joint')
    
    input("\nPress Enter to start trajectory execution...")

    # [4] Execute trajectory
    beauty_print("[4] Executing Trajectory on Robot", type="module", centered=False)
    
    executor = JointTrajectoryExecutor(
        robot=robot,
        speed_deg_s=args.speed_deg_s,
        tolerance=0.5,
        timeout=args.timeout,
        progress_interval=50,
        initial_delay=1.0,
        wait_for_completion=True,
        use_timing=False
    )
    
    executor.execute(
        joint_angles=to_numpy(trajectory['q']),
        trajectory_times=to_numpy(trajectory['t']),
        gripper_values=gripper_trajectory,
        initial_tolerance=0.1,
        initial_wait=True
    )

    robot.disconnect()
    return {'trajectory': trajectory, 'waypoints': waypoints}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Joint Space Trajectory Planning and Execution')
    
    # Robot connection
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪类型")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称")
    
    # Waypoint settings
    parser.add_argument('--no-record', action='store_true', help='Disable recording mode')
    parser.add_argument('--save-file', type=str, default=None, help='Path to save recorded waypoints')
    parser.add_argument('--waypoints-file', type=str, default=None, help='Path to JSON file with waypoints')
    parser.add_argument('--num-waypoints', type=int, default=6, help='Number of waypoints for random generation')
    parser.add_argument('--joint-scale', type=float, default=0.6, help='Scale factor for random joints (0.0-1.0)')
    parser.add_argument('--use-current-joints', action='store_true', help='Use current joints as first waypoint')
    
    # Trajectory planning
    parser.add_argument('--planner', type=str, default='b_spline', choices=['b_spline', 'multi_segment'],
                        help='Planner type (default: b_spline)')
    parser.add_argument('--duration', type=float, default=2.0, help='Trajectory duration (B-Spline)')
    parser.add_argument('--duration-per-segment', type=float, default=1.0, help='Duration per segment (Multi-Segment)')
    parser.add_argument('--num-points', type=int, default=800, help='Number of points (B-Spline)')
    parser.add_argument('--num-points-per-segment', type=int, default=100, help='Points per segment (Multi-Segment)')
    parser.add_argument('--bspline-degree', type=int, default=5, choices=[3, 5], help='B-Spline degree')
    parser.add_argument('--segment-method', type=str, default='quintic', choices=['cubic', 'quintic'],
                        help='Multi-segment method')
    
    # Execution
    parser.add_argument('--speed-deg-s', type=int, default=30, help="关节运动速度 (度/秒)")
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout per command (seconds)')
    
    # Other
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'], help='Backend')
    parser.add_argument('--device', type=str, default='cpu', help='Device')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    main(parser.parse_args())
