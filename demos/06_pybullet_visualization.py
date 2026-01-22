#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization with Obstacle Avoidance.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- IK-based goal pose specification with automatic retry
- Obstacle avoidance planning using PyBullet's collision detection
- Path smoothing for natural robot motion
- Trajectory visualization with end-effector trail

Run with: python demos/06_pybullet_visualization.py
"""

import numpy as np
import time

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.path_smoothing import smooth_path
from core.pose_utils import IKSolver, make_transform
from core.kinematics.opw import OPWKinematics
from core.kinematics.opw_parameters import OPWParameters
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
except ImportError:
    print("ERROR: PyBullet not installed. Install with: pip install pybullet")
    exit(1)


# UR5 realistic limits from URDF
UR5_VELOCITY_LIMITS = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])  # rad/s
UR5_ACCEL_LIMITS = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # rad/s^2


def is_within_joint_limits(q: np.ndarray, robot) -> bool:
    """Check if joint configuration is within robot joint limits."""
    lower, upper = robot.joint_limits()
    return np.all(q >= lower) and np.all(q <= upper)


def create_workspace_obstacles(viz):
    """Create floor to constrain the workspace."""
    obstacle_ids = []

    # Floor
    floor_id = viz.add_box(
        center=(0.0, 0.0, -0.005),
        size=(2.0, 2.0, 0.01),
        color=(0.5, 0.5, 0.5, 0.3),
    )
    obstacle_ids.append(floor_id)

    return obstacle_ids


def find_valid_goal(
    ik_solver: IKSolver,
    robot,
    reference_q: np.ndarray,
    reference_rotation: np.ndarray,
    collision_check,
    workspace_bounds: dict,
    max_attempts: int = 100,
) -> tuple:
    """
    Find a random collision-free goal pose within workspace bounds.

    Returns:
        (q_goal, goal_position) or (None, None) if not found
    """
    lower, upper = robot.joint_limits()

    for attempt in range(max_attempts):
        # Random position within workspace
        x = np.random.uniform(*workspace_bounds['x'])
        y = np.random.uniform(*workspace_bounds['y'])
        z = np.random.uniform(*workspace_bounds['z'])
        position = np.array([x, y, z])

        # Solve IK
        q_goal = ik_solver.solve_from_position(
            position,
            rotation=reference_rotation,
            reference_q=reference_q,
            collision_check=collision_check,
        )

        if q_goal is None:
            continue

        # Explicitly check joint limits
        if not is_within_joint_limits(q_goal, robot):
            continue

        # Check collision
        if collision_check(q_goal):
            continue

        return q_goal, position

    return None, None


def show_debug_markers(viz, q_start, q_goal, goal_position):
    """Show start and goal markers for debugging."""
    # Show start
    viz.set_joint_positions(q_start)
    start_ee, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(start_ee), color=(0, 1, 0), size=0.04)  # Green

    # Show goal
    if q_goal is not None:
        viz.set_joint_positions(q_goal)
        goal_ee, _ = viz.get_end_effector_pose()
        viz.add_marker(tuple(goal_ee), color=(1, 0, 0), size=0.04)  # Red
    elif goal_position is not None:
        # Show target position even if IK failed
        viz.add_marker(tuple(goal_position), color=(1, 0.5, 0), size=0.04)  # Orange

    # Return to start
    viz.set_joint_positions(q_start)


def run_planning_attempt(viz, robot, ik_solver, state_space, obstacle_ids, attempt_num):
    """
    Run a single planning attempt. Returns trajectory data or None on failure.
    """
    print(f"\n--- Attempt {attempt_num} ---")

    # Create collision checker
    pybullet_collision_check = viz.create_collision_checker(obstacle_ids)
    robot.in_collision = pybullet_collision_check

    # Start configuration
    q_start = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])

    # Check if start is valid
    if pybullet_collision_check(q_start):
        print("    Start in collision, trying alternative...")
        q_start = np.array([0.0, -np.pi/3, np.pi/3, -np.pi/2, np.pi/2, 0.0])
        if pybullet_collision_check(q_start):
            print("    Alternative start also in collision!")
            return None

    # Get start EE pose for reference
    start_pose = robot.fk(q_start)
    start_rotation = start_pose[:3, :3]

    # Workspace bounds - keep positions reachable
    workspace = {
        'x': (-0.2, 0.4),
        'y': (-0.3, 0.3),
        'z': (0.25, 0.65),
    }

    # Find valid goal
    print("    Searching for valid goal...")
    q_goal, goal_position = find_valid_goal(
        ik_solver, robot, q_start, start_rotation,
        pybullet_collision_check, workspace,
        max_attempts=100,
    )

    if q_goal is None:
        print("    Could not find valid goal pose!")
        show_debug_markers(viz, q_start, None, None)
        return None

    # Validate goal is within joint limits
    lower, upper = robot.joint_limits()
    if not is_within_joint_limits(q_goal, robot):
        print(f"    Goal outside joint limits!")
        for i in range(len(q_goal)):
            if q_goal[i] < lower[i] or q_goal[i] > upper[i]:
                print(f"      Joint {i}: {q_goal[i]:.3f} not in [{lower[i]:.3f}, {upper[i]:.3f}]")
        show_debug_markers(viz, q_start, q_goal, goal_position)
        return None

    print(f"    Start: {np.round(q_start, 2)}")
    print(f"    Goal:  {np.round(q_goal, 2)}")
    print(f"    Goal pos: [{goal_position[0]:.2f}, {goal_position[1]:.2f}, {goal_position[2]:.2f}]")

    # Show markers before planning
    show_debug_markers(viz, q_start, q_goal, goal_position)

    # Plan path
    print("    Planning path...")
    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=5.0)

    if path is None:
        print("    Planning failed!")
        time.sleep(1.0)  # Show the failed attempt briefly
        return None

    print(f"    Path found: {len(path)} waypoints")

    return {
        'q_start': q_start,
        'q_goal': q_goal,
        'goal_position': goal_position,
        'path': path,
        'collision_check': pybullet_collision_check,
    }


def main():
    print("=" * 60)
    print("PYBULLET 3D VISUALIZATION WITH OBSTACLE AVOIDANCE")
    print("=" * 60)

    # ---------------------------
    # 1. Setup robot and IK solver
    # ---------------------------
    print("\n[1] Setting up robot and IK solver...")

    robot = UR5RobotModel()
    opw_params = OPWParameters.make_ur5()
    opw_kin = OPWKinematics(robot, opw_params)
    ik_solver = IKSolver(robot, opw_kin)

    # Print joint limits for reference
    lower, upper = robot.joint_limits()
    print("    Joint limits (rad):")
    for i in range(6):
        print(f"      J{i+1}: [{lower[i]:.2f}, {upper[i]:.2f}]")

    # ---------------------------
    # 2. Create visualizer and obstacles
    # ---------------------------
    print("\n[2] Starting PyBullet and adding obstacles...")

    viz = PyBulletVisualizer(
        gui=True,
        base_position=(0, 0, 0),
        camera_distance=1.5,
        camera_yaw=45,
        camera_pitch=-30,
    )

    # Add workspace constraints
    obstacle_ids = create_workspace_obstacles(viz)

    # Add a random obstacle
    obstacle_center = np.array([
        np.random.uniform(0.0, 0.3),
        np.random.uniform(-0.2, 0.2),
        np.random.uniform(0.3, 0.5),
    ])
    obstacle_size = (0.10, 0.10, 0.15)

    obstacle_id = viz.add_box(
        center=tuple(obstacle_center),
        size=obstacle_size,
        color=(0.9, 0.2, 0.2, 0.7),
    )
    obstacle_ids.append(obstacle_id)

    print(f"    Obstacle at: [{obstacle_center[0]:.2f}, {obstacle_center[1]:.2f}, {obstacle_center[2]:.2f}]")

    # ---------------------------
    # 3. Setup state space
    # ---------------------------
    print("\n[3] Setting up planning...")

    state_space = JointStateSpace(robot)

    # ---------------------------
    # 4. Try planning with automatic retry
    # ---------------------------
    print("\n[4] Finding valid start/goal and planning...")

    max_retries = 10
    result = None

    for attempt in range(1, max_retries + 1):
        result = run_planning_attempt(viz, robot, ik_solver, state_space, obstacle_ids, attempt)
        if result is not None:
            print(f"\n    Success on attempt {attempt}!")
            break
        print(f"    Retrying... ({attempt}/{max_retries})")

    if result is None:
        print(f"\nERROR: Failed after {max_retries} attempts!")
        print("    Try running again - random obstacles may be in a better position.")
        time.sleep(3.0)
        viz.close()
        return

    # Extract results
    q_start = result['q_start']
    q_goal = result['q_goal']
    path = result['path']
    pybullet_collision_check = result['collision_check']

    # ---------------------------
    # 5. Smooth path
    # ---------------------------
    print("\n[5] Smoothing path...")

    smoothed_path = smooth_path(
        path,
        collision_check=pybullet_collision_check,
        shortcut_iterations=100,
        spline_points=150,
        step_size=0.02,
    )

    # Verify collision-free
    collisions = sum(1 for q in smoothed_path if pybullet_collision_check(q))
    if collisions > 0:
        print(f"    WARNING: {collisions} collisions in smoothed path, using original")
        smoothed_path = np.array(path)
    else:
        print(f"    Smoothed: {len(smoothed_path)} points")

    path = list(smoothed_path)

    # ---------------------------
    # 6. Time-parameterize
    # ---------------------------
    print("\n[6] Time-parameterizing...")

    toppra = ToppraTimeParameterizer(UR5_VELOCITY_LIMITS, UR5_ACCEL_LIMITS)
    time_stamps, trajectory = toppra.compute(path)

    print(f"    Duration: {time_stamps[-1]:.2f}s, Samples: {len(trajectory)}")

    # ---------------------------
    # 7. Visualize
    # ---------------------------
    print("\n[7] Visualizing...")
    print("    Controls: Mouse=rotate, Scroll=zoom, Q=quit")

    # Show start
    viz.visualize_configuration(q_start, duration=1.0)

    # Show path preview
    print("\n    Showing path preview...")
    viz.visualize_path(
        np.array(path),
        color=(1.0, 0.5, 0.0),
        line_width=2,
        show_waypoints=False,
    )
    time.sleep(2.0)

    # Execute trajectory
    print(f"\n    Executing trajectory ({time_stamps[-1]:.1f}s)...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
        trail_length=200,
    )

    # Hold final pose
    print("\n    Done! Holding final pose...")
    viz.visualize_configuration(trajectory[-1], duration=3.0)

    viz.close()
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
