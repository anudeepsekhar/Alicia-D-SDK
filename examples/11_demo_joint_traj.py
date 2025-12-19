#!/usr/bin/env python3
"""Joint Space Trajectory Planning and Execution

This demo demonstrates:
1. Generating smooth joint space trajectories through multiple waypoints
2. Supporting random waypoint generation or loading from file
3. Executing the trajectory on the robot

Copyright (c) 2025 Synria Robotics Co., Ltd.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

Author: Synria Robotics Team
Website: https://synriarobotics.ai
"""

import numpy as np
import argparse
import time
import json
import os

import alicia_d_sdk
import robocore as rc
from robocore.planning import BSplinePlanner, MultiSegmentPlanner, plot_joint_trajectory
from robocore.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.utils.backend import to_numpy


def load_joint_waypoints_from_file(file_path):
    """Load joint waypoints from JSON file.
    
    :param file_path: Path to JSON file containing joint waypoints
    :return: Tuple of (waypoints_array, gripper_values) where:
             - waypoints_array: Array of joint angles [n_waypoints, n_dof]
             - gripper_values: Array of gripper values [n_waypoints] or None if not present
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    waypoints = []
    gripper_values = []
    has_gripper = False
    
    # Check if data is a list
    if not isinstance(data, list):
        raise ValueError("JSON file must contain a list of waypoints")
    
    for i, wp in enumerate(data):
        if isinstance(wp, list):
            # Old format: list of joint angles only
            waypoints.append(np.array(wp, dtype=np.float64))
            gripper_values.append(None)
        elif isinstance(wp, dict):
            # New format: dict with 'joints' and optionally 'gripper'
            if 'joints' not in wp:
                raise ValueError(f"Waypoint {i+1}: dict must contain 'joints' key")
            waypoints.append(np.array(wp['joints'], dtype=np.float64))
            if 'gripper' in wp:
                gripper_values.append(float(wp['gripper']))
                has_gripper = True
            else:
                gripper_values.append(None)
        else:
            raise ValueError(f"Waypoint {i+1}: must be a list of joint angles or a dict with 'joints' key")
    
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")
    
    # Convert to numpy array and check consistency
    waypoints_array = np.array(waypoints)
    n_dof = waypoints_array.shape[1]
    
    # Check all waypoints have same DOF
    for i, wp in enumerate(waypoints):
        if len(wp) != n_dof:
            raise ValueError(f"Waypoint {i+1}: has {len(wp)} joints, expected {n_dof}")
    
    # Convert gripper values to array if all waypoints have gripper info
    if has_gripper:
        gripper_array = np.array([g if g is not None else 0.0 for g in gripper_values], dtype=np.float64)
        return waypoints_array, gripper_array
    else:
        return waypoints_array, None


def save_joint_waypoints_to_file(waypoints, file_path, gripper_values=None):
    """Save joint waypoints to JSON file.
    
    :param waypoints: Array of joint angles [n_waypoints, n_dof]
    :param file_path: Path to save JSON file
    :param gripper_values: Optional array of gripper values [n_waypoints]
    """
    # Convert to list format for JSON serialization
    waypoints_list = []
    for i, wp in enumerate(waypoints):
        wp_list = wp.tolist() if isinstance(wp, np.ndarray) else list(wp)
        if gripper_values is not None and i < len(gripper_values):
            # New format: dict with joints and gripper
            waypoints_list.append({
                'joints': wp_list,
                'gripper': float(gripper_values[i])
            })
        else:
            # Old format: list of joint angles only
            waypoints_list.append(wp_list)
    
    with open(file_path, 'w') as f:
        json.dump(waypoints_list, f, indent=2)
    
    gripper_info = f" (with gripper)" if gripper_values is not None else ""
    beauty_print(f"Saved {len(waypoints)} waypoints{gripper_info} to {file_path}", type="success")


def record_joint_waypoints_manual(robot):
    """Record joint waypoints by manually dragging robot.
    
    :param robot: Robot controller instance
    :return: Tuple of (waypoints_array, gripper_values) where:
             - waypoints_array: Array of joint angles [n_waypoints, n_dof]
             - gripper_values: Array of gripper values [n_waypoints]
    """
    beauty_print("\n=== 手动记录模式 ===", type="module", centered=False)
    beauty_print("关闭扭矩后，拖动到目标位置按回车记录")
    
    input("按回车开始...")
    robot.torque_control('off')
    beauty_print("[安全] 扭矩已关闭，可以拖动机械臂", type="warning")
    
    waypoints = []
    gripper_values = []
    
    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break
            
            # Get current joint angles
            joints = robot.get_joints()
            if joints is None:
                beauty_print("✗ 无法获取当前关节角度", type="warning")
                continue
            
            # Get current gripper value
            gripper = robot.get_gripper()
            if gripper is None:
                beauty_print("✗ 无法获取当前夹爪状态", type="warning")
                gripper = 0.0  # Default to closed
            
            joints = to_numpy(joints)
            waypoints.append(joints)
            gripper_values.append(float(gripper))
            
            beauty_print(f"[记录] 第{len(waypoints)}个点:")
            print(f"  关节角度 (rad): {beauty_print_array(joints)}")
            print(f"  关节角度 (deg): {beauty_print_array(np.rad2deg(joints))}")
            print(f"  夹爪状态: {gripper:.1f} (0-1000, 0=闭合, 1000=张开)")
    
    finally:
        robot.torque_control('on')
        beauty_print("[安全] 扭矩已重新开启", type="success")
    
    if len(waypoints) < 2:
        beauty_print("至少需要2个waypoint才能生成轨迹", type="warning")
        return None, None
    
    return np.array(waypoints), np.array(gripper_values)


def main(args):
    # Create robot connection
    beauty_print("Joint Space Trajectory Planning", type="module")
    
    robot = alicia_d_sdk.create_robot(port=args.port,
                                      robot_version=args.robot_version,
                                      gripper_type=args.gripper_type,
                                      base_link=args.base_link,
                                      end_link=args.end_link)

    # Set backend
    rc.set_backend(args.backend, device=args.device)
    robot_model = robot.robot_model

    beauty_print(f"Robot Model: {args.robot_version}")
    beauty_print(f"Base Link: {args.base_link}, End Link: {args.end_link}")
    beauty_print(f"DOF: {len(robot_model._chain_actuated)}")

    # [0] Record mode: manually record waypoints
    waypoints = None
    gripper_waypoints = None
    if args.record:
        beauty_print("[0] Recording Joint Waypoints", type="module", centered=False)
        waypoints, gripper_waypoints = record_joint_waypoints_manual(robot)
        
        if waypoints is None:
            robot.disconnect()
            return
        
        # Save to file if specified
        if args.save_file:
            save_joint_waypoints_to_file(waypoints, args.save_file, gripper_waypoints)
            beauty_print(f"Waypoints saved to {args.save_file}", type="success")
        else:
            beauty_print("No save file specified. Use --save-file to save waypoints.", type="warning")
        
        # Ask if user wants to execute trajectory
        user_input = input("\nExecute trajectory with recorded waypoints? (y/n): ").strip().lower()
        if user_input != 'y':
            beauty_print("Exiting without execution.", type="info")
            robot.disconnect()
            return
        # Continue to trajectory planning and execution
    
    # [1] Load or generate joint waypoints
    beauty_print("[1] Loading/Generating Joint Waypoints", type="module", centered=False)

    if waypoints is None:
        if args.waypoints_file:
            beauty_print(f"Loading waypoints from file: {args.waypoints_file}")
            try:
                waypoints, gripper_waypoints = load_joint_waypoints_from_file(args.waypoints_file)
                beauty_print(f"Successfully loaded {len(waypoints)} waypoints from file")
                if gripper_waypoints is not None:
                    beauty_print(f"Gripper values loaded: {len(gripper_waypoints)} waypoints")
            except Exception as e:
                beauty_print(f"Failed to load waypoints from file: {e}", type="error")
                robot.disconnect()
                return
        else:
            # Generate random waypoints within joint limits
            beauty_print(f"Generating {args.num_waypoints} random waypoints within joint limits...")
            waypoints = []
            gripper_waypoints = []
            
            # Get current joint angles and gripper as starting point (optional)
            if args.use_current_joints:
                q_start = robot.get_joints()
                g_start = robot.get_gripper()
                if q_start is not None:
                    waypoints.append(to_numpy(q_start))
                    gripper_waypoints.append(float(g_start) if g_start is not None else 500.0)
                    beauty_print(f"Using current joint angles and gripper as first waypoint:")
                    print(f"  Current joints (rad): {beauty_print_array(q_start)}")
                    print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q_start))}")
                    print(f"  Current gripper: {g_start:.1f} (0-1000)" if g_start is not None else "  Current gripper: N/A")
                    num_random = args.num_waypoints - 1
                else:
                    beauty_print("✗ 无法获取当前关节角度，使用随机生成", type="warning")
                    q_start = None
                    num_random = args.num_waypoints
            else:
                q_start = None
                num_random = args.num_waypoints
            
            # Generate random waypoints
            for i in range(num_random):
                # Use different seed for each waypoint to ensure variety
                waypoint_seed = args.seed + i if args.seed is not None else None
                q = to_numpy(robot_model.random_q(seed=waypoint_seed, scale=args.joint_scale))
                waypoints.append(q)
                # Random gripper value between 0 and 1000
                np.random.seed(waypoint_seed if waypoint_seed is not None else None)
                gripper_waypoints.append(float(np.random.uniform(0, 1000)))
            
            waypoints = np.array(waypoints)
            gripper_waypoints = np.array(gripper_waypoints) if gripper_waypoints else None

    beauty_print(f"Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i+1}: {beauty_print_array(wp)} (rad)")
        print(f"              {beauty_print_array(np.rad2deg(wp))} (deg)")
        if gripper_waypoints is not None and i < len(gripper_waypoints):
            print(f"              夹爪: {gripper_waypoints[i]:.1f} (0-1000)")

    # [2] Generate trajectory using planner
    beauty_print("[2] Generating Joint Space Trajectory", type="module", centered=False)

    if args.planner == 'b_spline':
        planner = BSplinePlanner(degree=args.bspline_degree)
        beauty_print(f"Using B-Spline planner (degree={args.bspline_degree})")
    else:  # multi_segment
        planner = MultiSegmentPlanner(method=args.segment_method)
        beauty_print(f"Using Multi-Segment planner (method={args.segment_method})")

    if args.planner == 'b_spline':
        trajectory = planner.plan(
            waypoints=waypoints,
            duration=args.duration,
            num_points=args.num_points
        )
    else:  # multi_segment
        trajectory = planner.plan(
            waypoints=waypoints,
            durations=args.duration_per_segment,
            num_points_per_segment=args.num_points_per_segment
        )

    beauty_print(f"Trajectory generated:")
    print(f"  Duration: {trajectory['t'][-1]:.3f} s")
    print(f"  Points: {len(trajectory['t'])}")
    print(f"  Max velocity: {np.max(np.abs(trajectory['qd'])):.3f} rad/s")
    print(f"  Max acceleration: {np.max(np.abs(trajectory['qdd'])):.3f} rad/s²")
    if 'qddd' in trajectory:
        print(f"  Max jerk: {np.max(np.abs(trajectory['qddd'])):.3f} rad/s³")

    # Interpolate gripper values if available
    gripper_trajectory = None
    if gripper_waypoints is not None:
        t_waypoints = np.linspace(0, trajectory['t'][-1], len(gripper_waypoints))
        t_traj = to_numpy(trajectory['t'])
        # Use linear interpolation for gripper using numpy
        gripper_trajectory = np.interp(t_traj, t_waypoints, gripper_waypoints)
        # Clip to valid range [0, 1000]
        gripper_trajectory = np.clip(gripper_trajectory, 0, 1000)
        beauty_print(f"Gripper trajectory interpolated: {len(gripper_trajectory)} points")
        print(f"  Gripper range: [{np.min(gripper_trajectory):.1f}, {np.max(gripper_trajectory):.1f}]")

    # Add waypoints to trajectory for plotting
    trajectory['waypoints'] = waypoints

    # [3] Plot trajectory first
    if args.plot:
        beauty_print("[3] Plotting Trajectory", type="module", centered=False)
        try:
            import matplotlib.pyplot as plt
            plot_joint_trajectory(trajectory, waypoints)
            plt.show(block=False)  # Non-blocking show
            plt.pause(0.1)  # Small pause to ensure plot is displayed
        except ImportError:
            beauty_print("matplotlib not installed. Skipping plots.", type="warning")
    
    # Wait for user to press Enter before execution
    input("\nPress Enter to start trajectory execution...")

    # [4] Execute trajectory on robot
    beauty_print("[4] Executing Trajectory on Robot", type="module", centered=False)
    
    beauty_print(f"Executing trajectory with {len(trajectory['q'])} points...")
    beauty_print(f"Speed: {args.speed_deg_s} deg/s")
    beauty_print(f"Trajectory duration: {trajectory['t'][-1]:.3f} s")
    beauty_print(f"Control frequency: {len(trajectory['q']) / trajectory['t'][-1]:.1f} Hz")
    
    # Get trajectory time array and joint angles
    t_trajectory = to_numpy(trajectory['t'])
    joint_angles = to_numpy(trajectory['q'])
    
    # Move to first point with wait
    beauty_print("Moving to starting position...")
    first_gripper = int(gripper_trajectory[0]) if gripper_trajectory is not None else None
    robot.set_robot_target(
        target_joints=joint_angles[0],
        gripper_value=first_gripper,
        joint_format='rad',
        speed_deg_s=args.speed_deg_s,
        tolerance=0.1,
        wait_for_completion=True,
        timeout=args.timeout
    )
    time.sleep(1)  # Small delay before starting trajectory
    
    start_exec_time = time.time()
    executed_count = 0
    trajectory_start_time = time.time()
    
    for i, q in enumerate(joint_angles):
        # # Calculate when this point should be executed based on trajectory timing
        # if i == 0:
        #     # First point already reached, just mark time
        #     point_target_time = trajectory_start_time
        # else:
        #     # Calculate time since trajectory start
        #     point_target_time = trajectory_start_time + t_trajectory[i]
        
        # Get gripper value for this point
        gripper_val = int(gripper_trajectory[i]) if gripper_trajectory is not None else None
        
        # Send command with both joints and gripper
        success = robot.set_robot_target(
            target_joints=q,
            gripper_value=gripper_val,
            joint_format='rad',
            speed_deg_s=args.speed_deg_s,
            tolerance=0.5,
            wait_for_completion=True,
            timeout=args.timeout
        )
        
        if success:
            executed_count += 1
            if (i + 1) % 50 == 0 or i == 0:
                print(f"  Executed point {i+1}/{len(joint_angles)} (t={t_trajectory[i]:.3f}s)")
        else:
            beauty_print(f"  Failed to execute point {i+1}", type="warning")
            user_input = input("  Continue execution? (y/n): ").strip().lower()
            if user_input != 'y':
                beauty_print("  Stopping execution.", type="warning")
                break
        
        # # Wait until next point's target time (except for last point)
        # if i < len(joint_angles) - 1:
        #     current_time = time.time()
        #     next_target_time = trajectory_start_time + t_trajectory[i + 1]
        #     wait_time = next_target_time - current_time
            
        #     if wait_time > 0:
        #         time.sleep(wait_time)
        #     elif wait_time < -0.01:  # If we're more than 10ms behind, warn
        #         if (i + 1) % 100 == 0:
        #             beauty_print(f"  Warning: Running {abs(wait_time)*1000:.1f}ms behind schedule at point {i+1}", type="warning")
    
    exec_time = time.time() - start_exec_time
    beauty_print(f"Trajectory execution completed:")
    print(f"  Executed: {executed_count}/{len(joint_angles)} points")
    print(f"  Total time: {exec_time:.4f} s (target: {trajectory['t'][-1]:.3f} s)")
    print(f"  Average time per point: {exec_time/executed_count*1000:.4f} ms" if executed_count > 0 else "")

    robot.disconnect()

    return {
        'trajectory': trajectory,
        'waypoints': waypoints
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Joint Space Trajectory Planning and Execution')
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v5_6", help="机器人版本")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪类型")
    parser.add_argument('--base_link', type=str, default="world", help="基座链路名称, world 或 base_link等")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称, tool0 或 Link6等")
    
    # Recording settings
    parser.add_argument('--record', action='store_true',
                        help='Record waypoints by manually dragging robot (torque will be disabled)')
    parser.add_argument('--save-file', type=str, default=None,
                        help='Path to save recorded waypoints (JSON format, used with --record)')
    
    # Trajectory planning settings
    parser.add_argument('--duration', type=float, default=2.0, help='Trajectory duration in seconds (for B-Spline)')
    parser.add_argument('--duration-per-segment', type=float, default=1.0,
                        help='Duration per segment in seconds (for Multi-Segment, default: 1.0)')
    parser.add_argument('--num-points', type=int, default=800, help='Number of points in trajectory (for B-Spline, should be 200hz)')
    parser.add_argument('--num-points-per-segment', type=int, default=100,
                        help='Number of points per segment (for Multi-Segment, default: 100)')
    parser.add_argument('--waypoints-file', type=str, default=None,
                        help='Path to JSON file containing joint waypoints. If provided, overrides --num-waypoints. '
                             'Format: list of waypoints, each is a list of joint angles [q1, q2, ..., qn] in radians')
    parser.add_argument('--num-waypoints', type=int, default=5,
                        help='Number of waypoints (default: 5, ignored if --waypoints-file or --record is provided)')
    parser.add_argument('--joint-scale', type=float, default=0.6,
                        help='Scale factor for random joint generation (0.0 to 1.0, default: 0.6)')
    parser.add_argument('--use-current-joints', action='store_true',
                        help='Use current joint angles as first waypoint')
    
    # Planner settings
    parser.add_argument('--planner', type=str, default='b_spline', choices=['b_spline', 'multi_segment'],
                        help='Trajectory planner type (default: b_spline)')
    parser.add_argument('--bspline-degree', type=int, default=5, choices=[3, 5],
                        help='B-Spline degree (3 for cubic, 5 for quintic, default: 3)')
    parser.add_argument('--segment-method', type=str, default='quintic', choices=['cubic', 'quintic'],
                        help='Multi-segment planner method (default: quintic)')
    
    # Execution settings
    parser.add_argument('--speed-deg-s', type=int, default=20,
                        help="关节运动速度 (单位: 度/秒，默认: 20，范围: 10-400度/秒)")
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Timeout for each motion command (seconds)')
    
    # Other settings
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'],
                        help='Backend (default: numpy)')
    parser.add_argument('--device', type=str, default='cpu', help='Device (cpu/cuda)')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    args = parser.parse_args()

    main(args)

