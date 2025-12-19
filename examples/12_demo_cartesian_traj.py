#!/usr/bin/env python3
"""Cartesian Spline Trajectory Planning with Inverse Kinematics

This demo demonstrates:
1. Generating a smooth spline trajectory in Cartesian space through multiple waypoints
2. Solving inverse kinematics for all poses in the trajectory (batch IK)
3. Executing the trajectory on the robot (optional)

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
from robocore.planning import SplineCurvePlanner, plot_cartesian_with_ik
from robocore.kinematics.ik import inverse_kinematics
from robocore.transform import make_transform, quaternion_to_matrix, rpy_to_matrix
from robocore.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.utils.backend import to_numpy


def load_waypoints_from_file(file_path):
    """Load waypoints from JSON file.
    
    :param file_path: Path to JSON file containing waypoints
    :return: List of 4x4 transformation matrices
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    waypoints = []
    
    # Check if data is a list
    if not isinstance(data, list):
        raise ValueError("JSON file must contain a list of waypoints")
    
    for i, wp in enumerate(data):
        if isinstance(wp, list) and len(wp) == 4 and all(isinstance(row, list) and len(row) == 4 for row in wp):
            # Direct 4x4 matrix format
            waypoints.append(np.array(wp, dtype=np.float64))
        elif isinstance(wp, dict):
            # Position + orientation format
            if 'position' not in wp:
                raise ValueError(f"Waypoint {i+1}: missing 'position' field")
            
            pos = np.array(wp['position'], dtype=np.float64)
            if len(pos) != 3:
                raise ValueError(f"Waypoint {i+1}: position must be [x, y, z]")
            
            # Handle orientation
            if 'orientation' in wp:
                ori = wp['orientation']
                if isinstance(ori, list):
                    if len(ori) == 4:
                        # Quaternion [x, y, z, w]
                        R = quaternion_to_matrix(np.array(ori, dtype=np.float64))
                    elif len(ori) == 3:
                        # RPY [roll, pitch, yaw]
                        R = rpy_to_matrix(ori[0], ori[1], ori[2])
                    else:
                        raise ValueError(f"Waypoint {i+1}: orientation must be quaternion [x,y,z,w] or RPY [roll,pitch,yaw]")
                else:
                    raise ValueError(f"Waypoint {i+1}: orientation must be a list")
            else:
                # Default orientation: identity rotation
                R = np.eye(3)
            
            # Create 4x4 transform matrix
            T = make_transform(R, pos)
            waypoints.append(T)
        else:
            raise ValueError(f"Waypoint {i+1}: invalid format. Must be 4x4 matrix or dict with 'position' and optional 'orientation'")
    
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")
    
    return waypoints


def normalize_joint_angles(q_new, q_prev, robot_model):
    """Normalize joint angles to minimize discontinuity with previous configuration.
    
    For revolute joints, add/subtract 2π to keep angles close to previous configuration.
    This helps maintain continuity when joints wrap around.
    
    :param q_new: New joint configuration
    :param q_prev: Previous joint configuration (can be None)
    :param robot_model: RobotModel instance
    :return: Normalized joint configuration
    """
    q_new = np.asarray(q_new)
    if q_prev is None:
        return q_new
    
    q_prev = np.asarray(q_prev)
    q_normalized = q_new.copy()
    
    for js in robot_model._chain_actuated:
        if js.joint_type == 'revolute':
            # For revolute joints, try to minimize the difference
            diff = q_new[js.index] - q_prev[js.index]
            
            # If difference is large, try adding/subtracting 2π
            if abs(diff) > np.pi:
                # Try subtracting 2π
                q_candidate = q_new[js.index] - 2 * np.pi
                if js.limit_lower is not None and js.limit_upper is not None:
                    if js.limit_lower <= q_candidate <= js.limit_upper:
                        if abs(q_candidate - q_prev[js.index]) < abs(diff):
                            q_normalized[js.index] = q_candidate
                            continue
                
                # Try adding 2π
                q_candidate = q_new[js.index] + 2 * np.pi
                if js.limit_lower is not None and js.limit_upper is not None:
                    if js.limit_lower <= q_candidate <= js.limit_upper:
                        if abs(q_candidate - q_prev[js.index]) < abs(diff):
                            q_normalized[js.index] = q_candidate
    
    return q_normalized


def solve_ik_simple(robot_model, target_pose, q0, args, actual_strategy):
    """Solve IK using previous solution as initial guess (simple approach like InteractiveDualArmIK).
    
    :param robot_model: RobotModel instance
    :param target_pose: Target pose (4x4 matrix)
    :param q0: Previous joint configuration (for continuity)
    :param args: Command line arguments
    :param actual_strategy: Actual initial guess strategy to use
    :return: IK result dictionary
    """
    if q0 is None:
        # First pose, use normal IK with specified strategy
        result = inverse_kinematics(
            robot_model,
            target_pose,
            q0=None,
            method=args.method,
            max_iters=args.max_iters,
            pos_tol=args.pos_tol,
            ori_tol=args.ori_tol,
            num_initial_guesses=args.num_inits,
            initial_guess_strategy=actual_strategy,
            initial_guess_scale=args.init_scale,
            random_seed=args.seed,
        )
        return result
    
    # Always use previous solution as initial guess (ensures continuity)
    # For subsequent poses, use smaller num_initial_guesses since we have a good initial guess
    num_inits_subsequent = min(5, args.num_inits) if args.num_inits > 1 else 1
    result = inverse_kinematics(
        robot_model,
        target_pose,
        q0=q0,
        method=args.method,
        max_iters=args.max_iters,
        pos_tol=args.pos_tol,
        ori_tol=args.ori_tol,
        num_initial_guesses=num_inits_subsequent,
        initial_guess_strategy='random',
        initial_guess_scale=args.init_scale,
        random_seed=args.seed,
    )
    
    # Normalize joint angles for revolute joints to maintain continuity
    if result['success']:
        q_normalized = normalize_joint_angles(result['q'], q0, robot_model)
        result['q'] = q_normalized.tolist()

    return result


def main(args):
    # Create robot connection
    beauty_print("Cartesian Spline Planning with IK Batch Solver", type="module")
    
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

    # [1] Generate spline trajectory in Cartesian space
    beauty_print("[1] Generating Spline Trajectory", type="module", centered=False)

    # Use numpy backend for smooth cubic spline interpolation
    # (torch backend uses linear interpolation which is not smooth)
    rc.set_backend('numpy')
    planner = SplineCurvePlanner()

    # Load waypoints from file or generate random waypoints
    if args.waypoints_file:
        beauty_print(f"Loading waypoints from file: {args.waypoints_file}")
        try:
            waypoints_list = load_waypoints_from_file(args.waypoints_file)
            waypoints = np.array([to_numpy(wp) for wp in waypoints_list])
            beauty_print(f"Successfully loaded {len(waypoints)} waypoints from file")
        except Exception as e:
            beauty_print(f"Failed to load waypoints from file: {e}", type="error")
            robot.disconnect()
            return
    else:
        # Generate random poses within robot workspace using random_pose_batch
        beauty_print(f"Generating {args.num_waypoints} random waypoints within workspace...")
        waypoints_list = robot_model.random_pose_batch(
            batch_size=args.num_waypoints,
            seed=args.seed,
            scale=args.workspace_scale
        )
        # Convert to numpy array if needed
        waypoints = np.array([to_numpy(wp) for wp in waypoints_list])

    beauty_print(f"Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        pos = wp[:3, 3]
        print(f"  Waypoint {i+1}: position = {beauty_print_array(pos)}")

    # Generate spline trajectory
    trajectory = planner.plan(
        waypoints=waypoints,
        duration=args.duration,
        num_points=args.num_points
    )

    beauty_print(f"Trajectory generated:")
    print(f"  Duration: {trajectory['t'][-1]:.3f} s")
    print(f"  Points: {len(trajectory['t'])}")
    print(f"  Max linear velocity: {np.max(np.linalg.norm(trajectory['velocities'][:, :3], axis=1)):.3f} m/s")
    print(f"  Max angular velocity: {np.max(np.linalg.norm(trajectory['velocities'][:, 3:], axis=1)):.3f} rad/s")

    # Extract waypoint positions for plotting
    waypoint_positions = np.array([wp[:3, 3] for wp in waypoints])
    trajectory['waypoints'] = waypoint_positions

    # Extract poses from trajectory
    if 'poses' in trajectory:
        target_poses = trajectory['poses']  # Shape: (num_points, 4, 4)
    else:
        # Build poses from positions and orientations
        positions = trajectory['positions']
        orientations = trajectory['orientations']
        num_points = len(positions)
        target_poses = np.zeros((num_points, 4, 4))
        for i in range(num_points):
            target_poses[i] = make_transform(orientations[i], positions[i])

    # Switch to specified backend for IK computation
    if args.backend != 'numpy':
        beauty_print(f"Switching to {args.backend} backend for IK computation...")
        rc.set_backend(args.backend, device=args.device)

    # [2] Solve IK sequentially using previous solution as initial guess
    # This ensures joint angle continuity (standard approach for Cartesian trajectories)
    beauty_print("[2] Solving Inverse Kinematics (Sequential)", type="module", centered=False)

    # Display sample target pose for debugging
    beauty_print(f"Sample target pose (first point):")
    sample_pose = target_poses[0]
    print(f"  Position: {beauty_print_array(sample_pose[:3, 3])}")
    print(f"  Rotation matrix:\n{sample_pose[:3, :3]}")

    beauty_print(f"Solving IK sequentially for {len(target_poses)} poses...")
    beauty_print(f"  Using previous solution as initial guess (ensures continuity)")

    # Get initial joint configuration for first pose
    if args.init_strategy == 'current':
        q0 = robot.get_joints()
        if q0 is None:
            beauty_print("✗ 无法获取当前关节角度，使用零初始配置", type="warning")
            q0 = None
            actual_strategy = 'zero'
        else:
            beauty_print(f"Using current joint angles as initial guess:")
            print(f"  Current joints (rad): {beauty_print_array(q0)}")
            print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q0))}")
            # Use 'random' strategy with current joints as base
            actual_strategy = 'random'
    else:
        q0 = None
        actual_strategy = args.init_strategy

    start_time = time.time()
    ik_results = []

    for i, target_pose in enumerate(target_poses):
        # Use simple IK solver (always uses previous solution as initial guess)
        result = solve_ik_simple(
            robot_model,
            target_pose,
            q0,
            args,
            actual_strategy if i == 0 else 'random'
        )

        ik_results.append(result)
        if q0 is not None:
            q_diff = np.linalg.norm(np.array(result['q']) - np.array(q0))
            print(f"Point {i+1}: Success={result['success']}, Joint angle change: {q_diff:.6f}")
        else:
            print(f"Point {i+1}: Success={result['success']}")
        print("q: ", result['q'])
        print("pos_err: ", result['pos_err'])
        print("ori_err: ", result['ori_err'])
        print("iters: ", result['iters'])
        print("--------------------------------")

        # This maintains continuity even if some poses fail
        q0 = to_numpy(result['q'])

    ik_time = time.time() - start_time

    beauty_print(f"IK sequential computation completed:")
    print(f"  Total time: {ik_time:.4f} s")
    print(f"  Average time per pose: {ik_time/len(target_poses)*1000:.4f} ms")

    # Check first few results for debugging
    beauty_print(f"First 3 IK results:")
    for i in range(min(3, len(ik_results))):
        result = ik_results[i]
        print(f"  Point {i+1}: success={result['success']}, "
              f"pos_err={result.get('pos_err', 0.0):.6e}, "
              f"ori_err={result.get('ori_err', 0.0):.6e}, "
              f"iters={result.get('iters', 0)}")

    # [3] Extract and display joint angles
    beauty_print("[3] Joint Angle Trajectory", type="module", centered=False)

    # Extract joint angles from IK results
    joint_angles = []
    success_count = 0

    for i, result in enumerate(ik_results):
        if result['success']:
            joint_angles.append(result['q'])
            success_count += 1
        else:
            # Use previous solution if available, otherwise use zeros
            if len(joint_angles) > 0:
                joint_angles.append(joint_angles[-1])
            else:
                joint_angles.append(np.zeros(len(robot_model._chain_actuated)))

    joint_angles = np.array(joint_angles)  # Shape: (num_points, n_dof)

    beauty_print(f"IK Success Rate: {success_count}/{len(ik_results)} ({success_count/len(ik_results)*100:.1f}%)")

    # Display joint angle statistics
    beauty_print(f"Joint Angle Statistics:")
    print(f"  Shape: {joint_angles.shape}")
    print(f"  Min values: {beauty_print_array(np.min(joint_angles, axis=0))}")
    print(f"  Max values: {beauty_print_array(np.max(joint_angles, axis=0))}")
    print(f"  Mean values: {beauty_print_array(np.mean(joint_angles, axis=0))}")

    # Display sample joint angles
    beauty_print(f"Sample Joint Angles (first 5 points):")
    for i in range(min(5, len(joint_angles))):
        print(f"  Point {i+1} (t={trajectory['t'][i]:.3f}s): {beauty_print_array(joint_angles[i])}")

    if len(joint_angles) > 5:
        beauty_print(f"Sample Joint Angles (last 5 points):")
        for i in range(max(0, len(joint_angles)-5), len(joint_angles)):
            print(f"  Point {i+1} (t={trajectory['t'][i]:.3f}s): {beauty_print_array(joint_angles[i])}")

    # Display error statistics for successful IK solutions
    if success_count > 0:
        pos_errors = [r['pos_err'] for r in ik_results if r['success']]
        ori_errors = [r['ori_err'] for r in ik_results if r['success']]

        beauty_print(f"Error Statistics (successful cases):")
        print(f"  Position error - Mean: {np.mean(pos_errors):.6e} m, Max: {np.max(pos_errors):.6e} m")
        print(f"  Orientation error - Mean: {np.mean(ori_errors):.6e} rad, Max: {np.max(ori_errors):.6e} rad")

    beauty_print("✓ Cartesian spline planning with IK batch solver completed!", type="success")

    # [4] Plot trajectory first
    if args.plot:
        beauty_print("[4] Plotting Trajectory", type="module", centered=False)
        try:
            import matplotlib.pyplot as plt
            plot_cartesian_with_ik(trajectory, waypoints, joint_angles, ik_results)
            plt.show(block=False)  # Non-blocking show
            plt.pause(0.1)  # Small pause to ensure plot is displayed
        except ImportError:
            beauty_print("matplotlib not installed. Skipping plots.", type="warning")
    
    # Wait for user to press Enter before execution
    input("\nPress Enter to start trajectory execution...")
    
    # [5] Execute trajectory on robot (default behavior)
    beauty_print("[5] Executing Trajectory on Robot", type="module", centered=False)
    
    if success_count < len(ik_results) * 0.8:  # Less than 80% success
        beauty_print(f"Warning: IK success rate is only {success_count/len(ik_results)*100:.1f}%", type="warning")
        beauty_print("Some poses may not be executed correctly", type="warning")
        user_input = input("Continue execution anyway? (y/n): ").strip().lower()
        if user_input != 'y':
            beauty_print("Execution cancelled by user", type="warning")
            robot.disconnect()
            return
    
    beauty_print(f"Executing trajectory with {len(joint_angles)} waypoints...")
    beauty_print(f"Speed: {args.speed_deg_s} deg/s")
    beauty_print(f"Trajectory duration: {trajectory['t'][-1]:.3f} s")
    beauty_print(f"Control frequency: {len(joint_angles) / trajectory['t'][-1]:.1f} Hz")
    
    # Get trajectory time array
    t_trajectory = to_numpy(trajectory['t'])
    
    # Move to first point with wait
    beauty_print("Moving to starting position...")
    robot.set_robot_target(
        target_joints=joint_angles[0],
        joint_format='rad',
        speed_deg_s=args.speed_deg_s,
        tolerance=0.5,
        wait_for_completion=True,
        timeout=args.timeout
    )
    time.sleep(0.1)  # Small delay before starting trajectory
    
    start_exec_time = time.time()
    executed_count = 0
    trajectory_start_time = time.time()
    
    for i, q in enumerate(joint_angles):
        # Calculate when this point should be executed based on trajectory timing
        if i == 0:
            # First point already reached, just mark time
            point_target_time = trajectory_start_time
        else:
            # Calculate time since trajectory start
            point_target_time = trajectory_start_time + t_trajectory[i]
        
        # Send command
        success = robot.set_robot_target(
            target_joints=q,
            joint_format='rad',
            speed_deg_s=args.speed_deg_s,
            tolerance=0.5,
            wait_for_completion=False,
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
        
        # Wait until next point's target time (except for last point)
        if i < len(joint_angles) - 1:
            current_time = time.time()
            next_target_time = trajectory_start_time + t_trajectory[i + 1]
            wait_time = next_target_time - current_time
            
            if wait_time > 0:
                time.sleep(wait_time)
            elif wait_time < -0.01:  # If we're more than 10ms behind, warn
                if (i + 1) % 100 == 0:
                    beauty_print(f"  Warning: Running {abs(wait_time)*1000:.1f}ms behind schedule at point {i+1}", type="warning")
    
    exec_time = time.time() - start_exec_time
    beauty_print(f"Trajectory execution completed:")
    print(f"  Executed: {executed_count}/{len(joint_angles)} points")
    print(f"  Total time: {exec_time:.4f} s (target: {trajectory['t'][-1]:.3f} s)")
    print(f"  Average time per point: {exec_time/executed_count*1000:.4f} ms" if executed_count > 0 else "")

    robot.disconnect()

    return {
        'trajectory': trajectory,
        'joint_angles': joint_angles,
        'ik_results': ik_results,
        'success_rate': success_count / len(ik_results),
        'waypoints': waypoints
    }




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Cartesian Spline Planning with IK Batch Solver')
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v5_6", help="机器人版本")
    parser.add_argument('--gripper_type', type=str, default="50mm", help="夹爪类型")
    parser.add_argument('--base_link', type=str, default="world", help="基座链路名称, world 或 base_link等")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称, tool0 或 Link6等")
    
    # Trajectory planning settings
    parser.add_argument('--duration', type=float, default=1.0, help='Trajectory duration in seconds')
    parser.add_argument('--num-points', type=int, default=100, help='Number of points in trajectory, should be 200hz')
    parser.add_argument('--waypoints-file', type=str, default=None,
                        help='Path to JSON file containing waypoints. If provided, overrides --num-waypoints. '
                             'Format: list of waypoints, each can be:\n'
                             '  - 4x4 matrix: [[r11,r12,r13,x],[r21,r22,r23,y],[r31,r32,r33,z],[0,0,0,1]]\n'
                             '  - dict with "position" [x,y,z] and optional "orientation" (quaternion [x,y,z,w] or RPY [roll,pitch,yaw])')
    parser.add_argument('--num-waypoints', type=int, default=5, help='Number of waypoints (default: 5, ignored if --waypoints-file is provided)')
    parser.add_argument('--workspace-scale', type=float, default=0.6,
                        help='Workspace scale factor for random poses (0.0 to 1.0, default: 0.6, ignored if --waypoints-file is provided)')
    
    # IK settings
    parser.add_argument('--method', type=str, default='dls', choices=['dls', 'pinv', 'transpose'],
                        help='IK method (default: dls)')
    parser.add_argument('--max-iters', type=int, default=100, help='Maximum IK iterations')
    parser.add_argument('--pos-tol', type=float, default=1e-2, help='Position tolerance (m)')
    parser.add_argument('--ori-tol', type=float, default=1e-2, help='Orientation tolerance (rad)')
    parser.add_argument('--init-scale', type=float, default=0.6,
                        help='Scale factor for initial guesses (0.0 to 1.0, default: 0.6)')
    parser.add_argument('--num-inits', type=int, default=5,
                        help='Number of initial guesses (default: 10)')
    parser.add_argument('--init-strategy', type=str, default='current',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform', 'current'],
                        help='Initial guess strategy (default: current=use current joint angles)')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    
    # Execution settings
    parser.add_argument('--speed-deg-s', type=int, default=20, 
                        help="关节运动速度 (单位: 度/秒，默认: 20，范围: 5-400度/秒)")
    parser.add_argument('--timeout', type=float, default=10.0, 
                        help='Timeout for each motion command (seconds)')
    
    # Other settings
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'],
                        help='Backend (default: numpy)')
    parser.add_argument('--device', type=str, default='cpu', help='Device (cpu/cuda)')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    args = parser.parse_args()

    main(args)
