#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- Path preview (showing planned waypoints as a line before execution)
- Trajectory visualization with end-effector trail
- Interactive camera controls

Run with: python demos/06_pybullet_visualization.py
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
except ImportError:
    print("ERROR: PyBullet not installed. Install with: pip install pybullet")
    exit(1)


def main():
    print("=" * 60)
    print("PYBULLET 3D VISUALIZATION DEMO")
    print("=" * 60)

    # ---------------------------
    # 1. Setup robot and plan path
    # ---------------------------
    print("\n[1] Setting up robot and planning path...")

    robot = UR5RobotModel()
    collision_manager = NullCollisionManager()
    robot.set_collision_manager(collision_manager)
    state_space = JointStateSpace(robot)

    # Define start and goal configurations
    q_start = np.array([0.0, -np.pi/4, np.pi/3, -np.pi/2, np.pi/4, 0.0])
    q_goal = np.array([np.pi/2, -np.pi/3, np.pi/4, -np.pi/4, -np.pi/4, np.pi/2])

    # Plan path
    planner = OMPLRRTConnectPlanner(state_space, step_size=0.1)
    path = planner.plan(q_start, q_goal, timeout=3.0)

    if path is None:
        print("ERROR: No path found!")
        return

    print(f"    Path found with {len(path)} waypoints")

    # ---------------------------
    # 2. Time-parameterize path
    # ---------------------------
    print("\n[2] Time-parameterizing path...")

    dof = robot.dof()
    v_max = np.ones(dof) * 1.5  # rad/s
    a_max = np.ones(dof) * 3.0  # rad/s^2

    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_stamps, trajectory = toppra.compute(path)
    print(f"    Trajectory: {time_stamps[-1]:.3f}s duration, {len(time_stamps)} samples")

    # ---------------------------
    # 3. Visualize with PyBullet
    # ---------------------------
    print("\n[3] Starting PyBullet visualization...")
    print("    Controls:")
    print("    - Mouse: Rotate camera")
    print("    - Scroll: Zoom")
    print("    - Q: Quit during playback")

    with PyBulletVisualizer(
        gui=True,
        base_position=(0, 0, 0),
        camera_distance=1.8,
        camera_yaw=50,
        camera_pitch=-25,
    ) as viz:
        # Show start configuration
        print("\n    Showing start configuration...")
        viz.visualize_configuration(q_start, duration=1.5)

        # Add markers for start and goal end-effector positions
        start_ee_pos, _ = viz.get_end_effector_pose()
        viz.add_marker(start_ee_pos, color=(0, 1, 0), size=0.04)  # Green = start

        # Set goal to see end position
        viz.set_joint_positions(q_goal)
        goal_ee_pos, _ = viz.get_end_effector_pose()
        viz.add_marker(goal_ee_pos, color=(1, 0, 0), size=0.04)  # Red = goal

        # Reset to start
        viz.visualize_configuration(q_start, duration=0.5)

        # ---------------------------
        # Show planned path preview
        # ---------------------------
        print("\n    Showing planned path preview (orange line)...")
        viz.visualize_path(
            path,
            fk_func=robot.fk,
            color=(1.0, 0.5, 0.0),  # Orange for planned waypoints
            line_width=3,
            show_waypoints=True,
            waypoint_color=(1.0, 0.3, 0.0),
            waypoint_size=0.012,
        )

        # Let user see the planned path
        import time
        time.sleep(2.0)

        # ---------------------------
        # Show smooth trajectory preview
        # ---------------------------
        print("    Showing smooth trajectory (cyan line)...")
        viz.visualize_trajectory_path(
            trajectory,
            fk_func=robot.fk,
            color=(0.0, 0.8, 1.0),  # Cyan for smooth trajectory
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

    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
