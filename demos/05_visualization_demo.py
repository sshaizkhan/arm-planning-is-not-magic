#!/usr/bin/env python3
"""
Demo 05: Visualization utilities for arm planning.

This demo shows how to visualize:
1. Joint-space trajectories (position, velocity, acceleration)
2. End-effector paths in 3D
3. Phase portraits
4. Robot arm configurations and animations

Plots are shown interactively one by one. Close each plot window to see the next.

Run with: python demos/05_visualization_demo.py
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from parameterization.ruckig_parameterization import RuckigTimeParameterizer

from visualization.trajectory_plots import (
    plot_joint_trajectory,
    plot_joint_comparison,
    plot_velocity_profile,
    plot_acceleration_profile,
    plot_phase_portrait,
)
from visualization.path_3d import (
    plot_ee_path_3d,
    plot_ee_path_with_waypoints,
    plot_ee_components,
)

from visualization.robot_visualizer import (
    RobotVisualizer,
    animate_trajectory,
)


def main():
    print("=" * 60)
    print("ARM PLANNING VISUALIZATION DEMO")
    print("=" * 60)
    print("\nClose each plot window to see the next visualization.\n")

    # ---------------------------
    # 1. Setup robot and plan path
    # ---------------------------
    print("[1] Setting up robot and planning path...")

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
    # 2. Time-parameterize with both TOPP-RA and Ruckig
    # ---------------------------
    print("\n[2] Time-parameterizing path...")

    dof = robot.dof()
    v_max = np.ones(dof) * 1.5  # rad/s
    a_max = np.ones(dof) * 3.0  # rad/s^2
    j_max = np.ones(dof) * 10.0  # rad/s^3

    # TOPP-RA
    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_toppra, traj_toppra = toppra.compute(path)
    print(f"    TOPP-RA: {time_toppra[-1]:.3f}s duration, {len(time_toppra)} samples")

    # Ruckig
    ruckig = RuckigTimeParameterizer(v_max, a_max, j_max)
    time_ruckig, traj_ruckig = ruckig.compute(path)
    print(f"    Ruckig:  {time_ruckig[-1]:.3f}s duration, {len(time_ruckig)} samples")

    joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    visualizer = RobotVisualizer()

    # ---------------------------
    # 3. Show plots one by one
    # ---------------------------
    print("\n[3] Displaying visualizations (close each window to continue)...\n")

    # Plot 1: Joint trajectory
    print("    [1/10] Joint Trajectory (TOPP-RA)")
    plot_joint_trajectory(
        time_toppra, traj_toppra,
        joint_names=joint_names,
        title="TOPP-RA Joint Trajectory",
        show=True
    )

    # Plot 2: Comparison
    print("    [2/10] TOPP-RA vs Ruckig Comparison")
    plot_joint_comparison(
        time_toppra, traj_toppra,
        time_ruckig, traj_ruckig,
        label_1="TOPP-RA",
        label_2="Ruckig",
        joint_names=joint_names,
        title="TOPP-RA vs Ruckig Trajectory Comparison",
        show=True
    )

    # Plot 3: Velocity profile
    print("    [3/10] Velocity Profile with Limits")
    plot_velocity_profile(
        time_toppra, traj_toppra,
        v_max=v_max,
        joint_names=joint_names,
        title="TOPP-RA Velocity Profile (green = within limits)",
        show=True
    )

    # Plot 4: Acceleration profile
    print("    [4/10] Acceleration Profile with Limits")
    plot_acceleration_profile(
        time_toppra, traj_toppra,
        a_max=a_max,
        joint_names=joint_names,
        title="TOPP-RA Acceleration Profile (green = within limits)",
        show=True
    )

    # Plot 5: Phase portrait
    print("    [5/10] Phase Portrait (Velocity vs Position)")
    plot_phase_portrait(
        time_toppra, traj_toppra,
        joint_indices=[0, 1, 2],  # First 3 joints
        joint_names=joint_names,
        title="Phase Portrait - First 3 Joints",
        show=True
    )

    # Plot 6: 3D end-effector path
    print("    [6/10] End-Effector 3D Path")
    plot_ee_path_3d(
        traj_toppra,
        robot.fk,
        title="End-Effector Path in 3D Space",
        show=True
    )

    # Plot 7: Path waypoints vs trajectory
    print("    [7/10] Path Waypoints vs Smooth Trajectory")
    plot_ee_path_with_waypoints(
        path, traj_toppra, robot.fk,
        title="Path Waypoints (orange) vs Smooth Trajectory (blue)",
        show=True
    )

    # Plot 8: EE components over time
    print("    [8/10] End-Effector X/Y/Z over Time")
    plot_ee_components(
        time_toppra, traj_toppra, robot.fk,
        title="End-Effector Position Components",
        show=True
    )

    # Plot 9: Robot start configuration
    print("    [9/10] Robot Start Configuration")
    visualizer.plot_configuration(
        q_start,
        title="Start Configuration",
        show=True
    )

    # Plot 10: Trajectory snapshots
    print("    [10/10] Trajectory Snapshots")
    visualizer.plot_trajectory_snapshots(
        traj_toppra,
        n_snapshots=7,
        title="Trajectory Motion (green=start, red=end)",
        show=True
    )

    # Plot 11: Trajectory animation
    print("    [11/10] Trajectory Animation")
    anim = animate_trajectory(traj_toppra, visualizer)
    anim.save("trajectory.gif", writer="pillow")
    print("    Animation saved to trajectory.gif")

    # ---------------------------
    # Summary
    # ---------------------------
    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)

    print("\nTo create an animation, run:")
    print("  from visualization import animate_trajectory, RobotVisualizer")
    print("  import matplotlib.pyplot as plt")
    print("  viz = RobotVisualizer()")
    print("  anim = animate_trajectory(trajectory, viz)")
    print("  plt.show()  # Or save with save_path='robot.gif'")


if __name__ == "__main__":
    main()
