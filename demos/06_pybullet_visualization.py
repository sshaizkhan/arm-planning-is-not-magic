#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization with Obstacle Avoidance.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- IK-based goal pose specification
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


def create_workspace_obstacles(viz):
    """Create floor and walls to constrain the workspace."""
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
    reference_q: np.ndarray,
    reference_rotation: np.ndarray,
    collision_check,
    workspace_bounds: dict,
    max_attempts: int = 50,
) -> tuple:
    """
    Find a random collision-free goal pose within workspace bounds.

    Args:
        ik_solver: IKSolver instance
        reference_q: Reference joint configuration
        reference_rotation: Desired end-effector orientation
        collision_check: Collision checking function
        workspace_bounds: Dict with 'x', 'y', 'z' ranges as (min, max) tuples
        max_attempts: Maximum attempts to find valid goal

    Returns:
        (q_goal, goal_position) or (None, None) if not found
    """
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

        if q_goal is not None and not collision_check(q_goal):
            return q_goal, position

    return None, None


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

    # Add a random obstacle between where start and goal might be
    obstacle_center = np.array([
        np.random.uniform(-0.2, 0.4),
        np.random.uniform(-0.2, 0.4),
        np.random.uniform(0.3, 0.7),
    ])
    obstacle_size = (0.12, 0.12, 0.20)

    obstacle_id = viz.add_box(
        center=tuple(obstacle_center),
        size=obstacle_size,
        color=(0.9, 0.2, 0.2, 0.7),
    )
    obstacle_ids.append(obstacle_id)

    print(f"    Obstacle at: [{obstacle_center[0]:.2f}, {obstacle_center[1]:.2f}, {obstacle_center[2]:.2f}]")

    # ---------------------------
    # 3. Setup collision-aware planning
    # ---------------------------
    print("\n[3] Setting up collision-aware planning...")

    pybullet_collision_check = viz.create_collision_checker(obstacle_ids)
    robot.in_collision = pybullet_collision_check
    state_space = JointStateSpace(robot)

    # ---------------------------
    # 4. Define start and goal poses
    # ---------------------------
    print("\n[4] Computing start and goal configurations...")

    # Start configuration (known valid pose)
    q_start = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])

    # Verify start is collision-free
    if pybullet_collision_check(q_start):
        print("    WARNING: Start in collision, trying alternative...")
        q_start = np.array([0.0, -np.pi/3, np.pi/3, -np.pi/2, np.pi/2, 0.0])

    # Get start EE pose for reference orientation
    start_pose = robot.fk(q_start)
    start_rotation = start_pose[:3, :3]

    # Workspace bounds for goal search
    workspace = {
        'x': (-0.3, 0.5),
        'y': (-0.4, 0.4),
        'z': (0.2, 0.8),
    }

    # Find collision-free goal
    print("    Searching for valid goal pose...")
    q_goal, goal_position = find_valid_goal(
        ik_solver, q_start, start_rotation,
        pybullet_collision_check, workspace,
    )

    if q_goal is None:
        print("ERROR: Could not find valid goal pose!")
        viz.close()
        return

    print(f"    Start: {np.round(q_start, 3)}")
    print(f"    Goal:  {np.round(q_goal, 3)}")
    print(f"    Goal position: [{goal_position[0]:.2f}, {goal_position[1]:.2f}, {goal_position[2]:.2f}]")

    # ---------------------------
    # 5. Plan path
    # ---------------------------
    print("\n[5] Planning collision-free path...")

    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=10.0)

    if path is None:
        print("ERROR: No path found!")
        viz.close()
        return

    print(f"    Raw path: {len(path)} waypoints")

    # ---------------------------
    # 6. Smooth path
    # ---------------------------
    print("\n[6] Smoothing path...")

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
        print(f"    Smoothed path: {len(smoothed_path)} points (collision-free)")

    path = list(smoothed_path)

    # ---------------------------
    # 7. Time-parameterize
    # ---------------------------
    print("\n[7] Time-parameterizing trajectory...")

    toppra = ToppraTimeParameterizer(UR5_VELOCITY_LIMITS, UR5_ACCEL_LIMITS)
    time_stamps, trajectory = toppra.compute(path)

    print(f"    Duration: {time_stamps[-1]:.2f}s")
    print(f"    Samples: {len(trajectory)}")

    # ---------------------------
    # 8. Visualize
    # ---------------------------
    print("\n[8] Visualizing...")
    print("    Controls: Mouse=rotate, Scroll=zoom, Q=quit")

    # Show start
    viz.visualize_configuration(q_start, duration=1.0)

    # Add markers
    start_ee, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(start_ee), color=(0, 1, 0), size=0.03)

    viz.set_joint_positions(q_goal)
    goal_ee, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(goal_ee), color=(1, 0, 0), size=0.03)

    viz.visualize_configuration(q_start, duration=0.5)

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
    print("\n    Done! Holding final pose for 3s...")
    viz.visualize_configuration(trajectory[-1], duration=3.0)

    viz.close()
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
