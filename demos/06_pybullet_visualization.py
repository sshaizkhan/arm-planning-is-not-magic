#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization with Obstacle Avoidance.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- Obstacle avoidance planning using PyBullet's collision detection
- Path preview (showing planned waypoints as a line before execution)
- Trajectory visualization with end-effector trail
- Interactive camera controls

Run with: python demos/06_pybullet_visualization.py
"""

import numpy as np
import time

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.path_smoothing import smooth_path
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
except ImportError:
    print("ERROR: PyBullet not installed. Install with: pip install pybullet")
    exit(1)


def main():
    print("=" * 60)
    print("PYBULLET 3D VISUALIZATION WITH OBSTACLE AVOIDANCE")
    print("=" * 60)

    # ---------------------------
    # 1. Setup robot
    # ---------------------------
    print("\n[1] Setting up robot...")

    robot = UR5RobotModel()

    # Define start and goal configurations
    q_start = np.array([0.0, -np.pi/4, np.pi/3, -np.pi/2, np.pi/4, 0.0])
    q_goal = np.array([np.pi/2, -np.pi/3, np.pi/4, -np.pi/4, -np.pi/4, np.pi/2])

    # ---------------------------
    # 2. Create visualizer and add obstacle
    # ---------------------------
    print("\n[2] Starting PyBullet and adding obstacle...")

    viz = PyBulletVisualizer(
        gui=True,
        base_position=(0, 0, 0),
        camera_distance=1.8,
        camera_yaw=50,
        camera_pitch=-25,
    )

    # Get EE positions using PyBullet's FK (for visual consistency)
    viz.set_joint_positions(q_start)
    start_ee, _ = viz.get_end_effector_pose()
    viz.set_joint_positions(q_goal)
    goal_ee, _ = viz.get_end_effector_pose()

    print(f"    Start EE position: [{start_ee[0]:.3f}, {start_ee[1]:.3f}, {start_ee[2]:.3f}]")
    print(f"    Goal EE position:  [{goal_ee[0]:.3f}, {goal_ee[1]:.3f}, {goal_ee[2]:.3f}]")

    # Place obstacle between start and goal
    obstacle_center = (start_ee + goal_ee) / 2
    obstacle_center[2] += 0.0  # Raise slightly
    obstacle_size = (0.10, 0.10, 0.98)

    print(f"    Obstacle center: [{obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}, {obstacle_center[2]:.3f}]")

    # Add obstacle to PyBullet (returns body ID)
    obstacle_id = viz.add_box(
        center=tuple(obstacle_center),
        size=obstacle_size,
        color=(0.9, 0.2, 0.2, 0.7),
    )

    # Add floor obstacle to prevent robot from going under ground
    floor_height = 0.01  # Thickness of floor
    floor_size = (2.0, 2.0, floor_height)  # Large floor covering workspace
    floor_center = (0.0, 0.0, -floor_height / 2)  # Positioned at z=0 (slightly below)
    floor_id = viz.add_box(
        center=floor_center,
        size=floor_size,
        color=(0.5, 0.5, 0.5, 0.8),  # Gray floor
    )
    print(f"    Floor obstacle added at z={floor_center[2]:.3f}")

    # Add walls and ceiling to constrain workspace
    workspace_size = 1.2  # Workspace half-size (meters)
    workspace_height = 1.5  # Height of workspace (meters)
    wall_thickness = 0.02  # Thickness of walls

    # Create list to store all obstacle IDs
    obstacle_ids = [obstacle_id, floor_id]

    # Front wall (positive Y)
    front_wall_id = viz.add_box(
        center=(0.0, workspace_size, workspace_height / 2),
        size=(workspace_size * 2, wall_thickness, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(front_wall_id)

    # Back wall (negative Y)
    back_wall_id = viz.add_box(
        center=(0.0, -workspace_size, workspace_height / 2),
        size=(workspace_size * 2, wall_thickness, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(back_wall_id)

    # Left wall (negative X)
    left_wall_id = viz.add_box(
        center=(-workspace_size, 0.0, workspace_height / 2),
        size=(wall_thickness, workspace_size * 2, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(left_wall_id)

    # Right wall (positive X)
    right_wall_id = viz.add_box(
        center=(workspace_size, 0.0, workspace_height / 2),
        size=(wall_thickness, workspace_size * 2, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(right_wall_id)

    # Ceiling
    ceiling_id = viz.add_box(
        center=(0.0, 0.0, workspace_height),
        size=(workspace_size * 2, workspace_size * 2, wall_thickness),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(ceiling_id)

    print(f"    Workspace enclosure added: {workspace_size*2:.1f}m x {workspace_size*2:.1f}m x {workspace_height:.1f}m")

    # ---------------------------
    # 3. Setup collision-aware planning using PyBullet
    # ---------------------------
    print("\n[3] Setting up collision-aware planning...")

    # Create collision checker that uses PyBullet (includes all obstacles)
    pybullet_collision_check = viz.create_collision_checker(obstacle_ids)

    # Create a custom collision manager that uses PyBullet's collision detection
    class PyBulletCollisionManager:
        def __init__(self, check_func):
            self._check = check_func

        def in_collision(self, q):
            return self._check(q)

    collision_manager = PyBulletCollisionManager(pybullet_collision_check)
    robot.in_collision = collision_manager.in_collision

    state_space = JointStateSpace(robot)

    # ---------------------------
    # 4. Plan path avoiding obstacle
    # ---------------------------
    print("\n[4] Planning collision-free path...")

    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=10.0)

    if path is None:
        print("ERROR: No path found! Try adjusting obstacle position or size.")
        viz.close()
        return

    print(f"    Raw path found with {len(path)} waypoints")

    # ---------------------------
    # 5. Smooth the path
    # ---------------------------
    print("\n[5] Smoothing path...")

    # Apply shortcut + spline smoothing
    smoothed_path = smooth_path(
        path,
        collision_check=pybullet_collision_check,
        shortcut_iterations=100,  # Remove unnecessary waypoints
        spline_points=150,        # Smooth curve with 150 points
        step_size=0.02,
    )

    print(f"    Smoothed path: {len(smoothed_path)} points")

    # Verify smoothed path is collision-free
    collisions = sum(1 for q in smoothed_path if pybullet_collision_check(q))
    if collisions > 0:
        print(f"    WARNING: Smoothed path has {collisions} collisions, using original")
        smoothed_path = np.array(path)
    else:
        print("    Smoothed path verified collision-free!")

    # Use smoothed path for visualization and execution
    path = [smoothed_path[i] for i in range(len(smoothed_path))]

    # ---------------------------
    # 6. Time-parameterize path
    # ---------------------------
    print("\n[6] Time-parameterizing path...")

    dof = robot.dof()
    v_max = np.ones(dof) * 1.5
    a_max = np.ones(dof) * 3.0

    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_stamps, trajectory = toppra.compute(path)
    print(f"    Trajectory: {time_stamps[-1]:.3f}s duration, {len(time_stamps)} samples")

    # ---------------------------
    # 7. Visualize
    # ---------------------------
    print("\n[7] Visualizing...")
    print("    Controls:")
    print("    - Mouse: Rotate camera")
    print("    - Scroll: Zoom")
    print("    - Q: Quit during playback")

    # Show start configuration
    print("\n    Showing start configuration...")
    viz.visualize_configuration(q_start, duration=1.0)

    # Add markers for start and goal
    start_ee_pos, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(start_ee_pos), color=(0, 1, 0), size=0.04)  # Green = start

    viz.set_joint_positions(q_goal)
    goal_ee_pos, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(goal_ee_pos), color=(1, 0, 0), size=0.04)  # Red = goal

    viz.visualize_configuration(q_start, duration=0.5)

    # Show planned path
    print("\n    Showing planned path (orange) - notice it avoids the obstacle...")
    viz.visualize_path(
        np.array(path),
        color=(1.0, 0.5, 0.0),
        line_width=3,
        show_waypoints=True,
        waypoint_color=(1.0, 0.3, 0.0),
        waypoint_size=0.012,
    )
    time.sleep(3.0)

    # Show smooth trajectory
    print("    Showing smooth trajectory (cyan line)...")
    viz.visualize_trajectory_path(
        trajectory,
        color=(0.0, 0.8, 1.0),
        line_width=2,
        sample_every=10,
    )
    time.sleep(2.0)

    # Play trajectory
    print("\n    Playing trajectory...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
        trail_length=200,
    )

    # Hold final position
    print("\n    Trajectory complete. Holding final position...")
    print("    Press Q in PyBullet window or wait 5 seconds to exit.")
    viz.visualize_configuration(trajectory[-1], duration=5.0)

    viz.close()

    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
