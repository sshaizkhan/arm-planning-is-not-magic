#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization with Obstacle Avoidance.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- Obstacle avoidance planning (robot avoids a box obstacle)
- Path preview (showing planned waypoints as a line before execution)
- Trajectory visualization with end-effector trail
- Interactive camera controls

Run with: python demos/06_pybullet_visualization.py
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import ShapeCollisionManager, Box
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
    # 2. Compute obstacle position using PyBullet's FK (for visual consistency)
    # ---------------------------
    print("\n[2] Computing obstacle position...")

    # We need to use PyBullet's FK to get EE positions that match the visualization
    # Create a temporary visualizer just to get the FK
    import pybullet as p
    temp_client = p.connect(p.DIRECT)  # Headless
    p.setAdditionalSearchPath(__import__('pybullet_data').getDataPath())

    # Load the same URDF
    from visualization.pybullet_visualizer import get_default_ur5_urdf
    urdf_path = get_default_ur5_urdf()
    temp_robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

    # Get joint indices
    joint_indices = []
    for i in range(p.getNumJoints(temp_robot)):
        if p.getJointInfo(temp_robot, i)[2] == p.JOINT_REVOLUTE:
            joint_indices.append(i)
    ee_link = p.getNumJoints(temp_robot) - 1

    # Compute start EE position
    for i, idx in enumerate(joint_indices):
        p.resetJointState(temp_robot, idx, q_start[i])
    start_ee = np.array(p.getLinkState(temp_robot, ee_link)[0])

    # Compute goal EE position
    for i, idx in enumerate(joint_indices):
        p.resetJointState(temp_robot, idx, q_goal[i])
    goal_ee = np.array(p.getLinkState(temp_robot, ee_link)[0])

    p.disconnect(temp_client)

    print(f"    Start EE position: [{start_ee[0]:.3f}, {start_ee[1]:.3f}, {start_ee[2]:.3f}]")
    print(f"    Goal EE position:  [{goal_ee[0]:.3f}, {goal_ee[1]:.3f}, {goal_ee[2]:.3f}]")

    # ---------------------------
    # 3. Add obstacle between start and goal
    # ---------------------------
    print("\n[3] Adding obstacle between start and goal...")

    # Place obstacle roughly between start and goal EE positions
    obstacle_center = (start_ee + goal_ee) / 2
    obstacle_center[2] += 0.05  # Raise it slightly
    obstacle_size = np.array([0.12, 0.12, 0.20])  # Box dimensions

    print(f"    Obstacle center: [{obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}, {obstacle_center[2]:.3f}]")
    print(f"    Obstacle size: {obstacle_size[0]:.2f} x {obstacle_size[1]:.2f} x {obstacle_size[2]:.2f}")

    # Setup collision manager with the obstacle
    # Note: ShapeCollisionManager uses robot.fk() which may differ from PyBullet's FK
    # For planning, we use the robot's own FK (which is what the planner sees)
    collision_manager = ShapeCollisionManager(robot)
    collision_manager.add_shape(Box(obstacle_center, obstacle_size))
    robot.set_collision_manager(collision_manager)

    state_space = JointStateSpace(robot)

    # ---------------------------
    # 4. Plan path avoiding obstacle
    # ---------------------------
    print("\n[4] Planning collision-free path...")

    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=5.0)

    if path is None:
        print("ERROR: No path found! Try adjusting obstacle position or size.")
        return

    print(f"    Path found with {len(path)} waypoints")

    # Verify path is collision-free
    collisions = sum(1 for q in path if collision_manager.in_collision(q))
    if collisions > 0:
        print(f"    WARNING: Path has {collisions} collisions!")
    else:
        print("    Path verified collision-free!")

    # ---------------------------
    # 5. Time-parameterize path
    # ---------------------------
    print("\n[5] Time-parameterizing path...")

    dof = robot.dof()
    v_max = np.ones(dof) * 1.5  # rad/s
    a_max = np.ones(dof) * 3.0  # rad/s^2

    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_stamps, trajectory = toppra.compute(path)
    print(f"    Trajectory: {time_stamps[-1]:.3f}s duration, {len(time_stamps)} samples")

    # ---------------------------
    # 6. Visualize with PyBullet
    # ---------------------------
    print("\n[6] Starting PyBullet visualization...")
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
        # Add the obstacle to the visualization (semi-transparent red box)
        print("\n    Adding obstacle to scene...")
        viz.add_box(
            center=tuple(obstacle_center),
            size=tuple(obstacle_size),
            color=(0.9, 0.2, 0.2, 0.6),  # Semi-transparent red
        )

        # Show start configuration
        print("    Showing start configuration...")
        viz.visualize_configuration(q_start, duration=1.0)

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
        # Note: Don't pass fk_func - use PyBullet's internal FK to match the visual robot
        print("\n    Showing planned path (orange) - notice it avoids the obstacle...")
        viz.visualize_path(
            path,
            color=(1.0, 0.5, 0.0),  # Orange for planned waypoints
            line_width=3,
            show_waypoints=True,
            waypoint_color=(1.0, 0.3, 0.0),
            waypoint_size=0.012,
        )

        # Let user see the planned path
        import time
        time.sleep(3.0)

        # ---------------------------
        # Show smooth trajectory preview
        # ---------------------------
        print("    Showing smooth trajectory (cyan line)...")
        viz.visualize_trajectory_path(
            trajectory,
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
