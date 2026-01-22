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


# UR5 realistic limits
UR5_VELOCITY_LIMITS = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])
UR5_ACCEL_LIMITS = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

# Safe start configurations (arm pointing up/out, away from base)
SAFE_START_CONFIGS = [
    np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]),  # Arm up
    np.array([0.0, -np.pi/2, np.pi/3, -np.pi/3, -np.pi/2, 0.0]),  # Arm up variant
    np.array([np.pi/4, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]),  # Rotated
    np.array([-np.pi/4, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]),  # Rotated other way
]


def is_within_joint_limits(q: np.ndarray, robot) -> bool:
    """Check if joint configuration is within robot joint limits."""
    lower, upper = robot.joint_limits()
    return np.all(q >= lower) and np.all(q <= upper)


def find_collision_free_start(viz, robot, obstacle_ids):
    """Find a start configuration that doesn't collide with obstacles."""
    collision_check = viz.create_collision_checker(obstacle_ids)

    for i, q in enumerate(SAFE_START_CONFIGS):
        if not collision_check(q) and is_within_joint_limits(q, robot):
            return q, collision_check

    return None, collision_check


def find_valid_goal(
    ik_solver: IKSolver,
    robot,
    reference_q: np.ndarray,
    reference_rotation: np.ndarray,
    collision_check,
    workspace_bounds: dict,
    max_attempts: int = 100,
) -> tuple:
    """Find a random collision-free goal pose within workspace bounds."""
    for attempt in range(max_attempts):
        x = np.random.uniform(*workspace_bounds['x'])
        y = np.random.uniform(*workspace_bounds['y'])
        z = np.random.uniform(*workspace_bounds['z'])
        position = np.array([x, y, z])

        q_goal = ik_solver.solve_from_position(
            position,
            rotation=reference_rotation,
            reference_q=reference_q,
            collision_check=collision_check,
        )

        if q_goal is None:
            continue

        if not is_within_joint_limits(q_goal, robot):
            continue

        if collision_check(q_goal):
            continue

        return q_goal, position

    return None, None


def main():
    print("=" * 60)
    print("PYBULLET 3D VISUALIZATION WITH OBSTACLE AVOIDANCE")
    print("=" * 60)

    # ---------------------------
    # 1. Setup robot and IK solver
    # ---------------------------
    print("\n[1] Setting up robot...")

    robot = UR5RobotModel()
    opw_params = OPWParameters.make_ur5()
    opw_kin = OPWKinematics(robot, opw_params)
    ik_solver = IKSolver(robot, opw_kin)

    # ---------------------------
    # 2. Create visualizer
    # ---------------------------
    print("\n[2] Starting PyBullet...")

    viz = PyBulletVisualizer(
        gui=True,
        base_position=(0, 0, 0),
        camera_distance=1.5,
        camera_yaw=45,
        camera_pitch=-30,
    )

    # Start with just the floor
    floor_id = viz.add_box(
        center=(0.0, 0.0, -0.01),
        size=(2.0, 2.0, 0.02),
        color=(0.5, 0.5, 0.5, 0.3),
    )
    obstacle_ids = [floor_id]

    # ---------------------------
    # 3. Find valid start configuration
    # ---------------------------
    print("\n[3] Finding valid start configuration...")

    q_start, collision_check = find_collision_free_start(viz, robot, obstacle_ids)

    if q_start is None:
        print("ERROR: Could not find valid start (even without obstacle)!")
        viz.close()
        return

    # Show the start configuration
    viz.set_joint_positions(q_start)
    start_ee, _ = viz.get_end_effector_pose()
    print(f"    Start config: {np.round(q_start, 2)}")
    print(f"    Start EE pos: [{start_ee[0]:.2f}, {start_ee[1]:.2f}, {start_ee[2]:.2f}]")

    # ---------------------------
    # 4. Place obstacle away from start
    # ---------------------------
    print("\n[4] Placing obstacle...")

    # Place obstacle in front of the robot, not at the start position
    # Try a few random positions until we find one that doesn't collide with start
    for _ in range(20):
        obstacle_center = np.array([
            np.random.uniform(0.2, 0.5),   # In front of robot
            np.random.uniform(-0.3, 0.3),  # Left/right
            np.random.uniform(0.3, 0.6),   # At arm height
        ])
        obstacle_size = (0.10, 0.10, 0.15)

        # Temporarily add obstacle to check
        temp_obstacle_id = viz.add_box(
            center=tuple(obstacle_center),
            size=obstacle_size,
            color=(0.9, 0.2, 0.2, 0.7),
        )

        # Check if start is still valid
        temp_collision_check = viz.create_collision_checker([floor_id, temp_obstacle_id])
        if not temp_collision_check(q_start):
            # Good - start doesn't collide with this obstacle position
            obstacle_ids.append(temp_obstacle_id)
            print(f"    Obstacle at: [{obstacle_center[0]:.2f}, {obstacle_center[1]:.2f}, {obstacle_center[2]:.2f}]")
            break
        else:
            # Remove and try again
            viz.remove_body(temp_obstacle_id)
    else:
        print("    WARNING: Could not place obstacle without hitting start, proceeding without obstacle")

    # Update collision checker with final obstacles
    collision_check = viz.create_collision_checker(obstacle_ids)
    robot.in_collision = collision_check

    # ---------------------------
    # 5. Find valid goal
    # ---------------------------
    print("\n[5] Finding valid goal...")

    start_pose = robot.fk(q_start)
    start_rotation = start_pose[:3, :3]

    workspace = {
        'x': (0.2, 0.5),
        'y': (-0.3, 0.3),
        'z': (0.2, 0.6),
    }

    q_goal, goal_position = find_valid_goal(
        ik_solver, robot, q_start, start_rotation,
        collision_check, workspace, max_attempts=200,
    )

    if q_goal is None:
        print("ERROR: Could not find valid goal pose!")
        # Show start marker before closing
        viz.add_marker(tuple(start_ee), color=(0, 1, 0), size=0.04)
        time.sleep(3.0)
        viz.close()
        return

    print(f"    Goal config: {np.round(q_goal, 2)}")
    print(f"    Goal position: [{goal_position[0]:.2f}, {goal_position[1]:.2f}, {goal_position[2]:.2f}]")

    # Add markers
    viz.add_marker(tuple(start_ee), color=(0, 1, 0), size=0.03)  # Green = start

    viz.set_joint_positions(q_goal)
    goal_ee, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(goal_ee), color=(1, 0, 0), size=0.03)  # Red = goal

    viz.set_joint_positions(q_start)

    # ---------------------------
    # 6. Plan path
    # ---------------------------
    print("\n[6] Planning path...")

    state_space = JointStateSpace(robot)
    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=10.0)

    if path is None:
        print("ERROR: No path found!")
        time.sleep(3.0)
        viz.close()
        return

    print(f"    Raw path: {len(path)} waypoints")

    # ---------------------------
    # 7. Smooth path
    # ---------------------------
    print("\n[7] Smoothing path...")

    smoothed_path = smooth_path(
        path,
        collision_check=collision_check,
        shortcut_iterations=100,
        spline_points=150,
        step_size=0.02,
    )

    collisions = sum(1 for q in smoothed_path if collision_check(q))
    if collisions > 0:
        print(f"    WARNING: {collisions} collisions, using original path")
        smoothed_path = np.array(path)
    else:
        print(f"    Smoothed: {len(smoothed_path)} points")

    path = list(smoothed_path)

    # ---------------------------
    # 8. Time-parameterize
    # ---------------------------
    print("\n[8] Time-parameterizing...")

    toppra = ToppraTimeParameterizer(UR5_VELOCITY_LIMITS, UR5_ACCEL_LIMITS)
    time_stamps, trajectory = toppra.compute(path)

    print(f"    Duration: {time_stamps[-1]:.2f}s")

    # ---------------------------
    # 9. Visualize
    # ---------------------------
    print("\n[9] Executing trajectory...")
    print("    Controls: Mouse=rotate, Scroll=zoom, Q=quit")

    viz.visualize_configuration(q_start, duration=1.0)

    # Show path
    viz.visualize_path(
        np.array(path),
        color=(1.0, 0.5, 0.0),
        line_width=2,
        show_waypoints=False,
    )
    time.sleep(2.0)

    # Execute
    print(f"\n    Playing ({time_stamps[-1]:.1f}s)...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
        trail_length=200,
    )

    print("\n    Done!")
    viz.visualize_configuration(trajectory[-1], duration=3.0)

    viz.close()
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
