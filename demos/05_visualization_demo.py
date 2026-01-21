#!/usr/bin/env python3
"""
Demo 05: Visualization utilities for arm planning.

This demo shows how to visualize:
1. Joint-space trajectories (position, velocity, acceleration)
2. End-effector paths in 3D
3. Phase portraits
4. Robot arm configurations and animations

Run with: python demos/05_visualization_demo.py
"""

import numpy as np
import matplotlib.pyplot as plt

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

    print(f"   Path found with {len(path)} waypoints")

    # ---------------------------
    # 2. Time-parameterize with both TOPP-RA and Ruckig
    # ---------------------------
    print("\n[2] Time-parameterizing path...")

    dof = robot.dof()
    v_max = np.ones(dof) * 1.5  # rad/s
    a_max = np.ones(dof) * 3.0  # rad/s²
    j_max = np.ones(dof) * 10.0  # rad/s³

    # TOPP-RA
    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_toppra, traj_toppra = toppra.compute(path)
    print(f"   TOPP-RA: {time_toppra[-1]:.3f}s duration, {len(time_toppra)} samples")

    # Ruckig
    ruckig = RuckigTimeParameterizer(v_max, a_max, j_max)
    time_ruckig, traj_ruckig = ruckig.compute(path)
    print(f"   Ruckig:  {time_ruckig[-1]:.3f}s duration, {len(time_ruckig)} samples")

    # ---------------------------
    # 3. Joint-space visualization
    # ---------------------------
    print("\n[3] Creating joint-space plots...")

    joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]

    # Joint trajectory
    print("   - Joint trajectory plot")
    plot_joint_trajectory(
        time_toppra, traj_toppra,
        joint_names=joint_names,
        title="TOPP-RA Joint Trajectory",
        show=False
    )
    plt.savefig("joint_trajectory.png", dpi=150)
    plt.close()

    # Comparison plot
    print("   - TOPP-RA vs Ruckig comparison")
    plot_joint_comparison(
        time_toppra, traj_toppra,
        time_ruckig, traj_ruckig,
        label_1="TOPP-RA",
        label_2="Ruckig",
        joint_names=joint_names,
        title="TOPP-RA vs Ruckig Trajectory Comparison",
        show=False
    )
    plt.savefig("trajectory_comparison.png", dpi=150)
    plt.close()

    # Velocity profile
    print("   - Velocity profile with limits")
    plot_velocity_profile(
        time_toppra, traj_toppra,
        v_max=v_max,
        joint_names=joint_names,
        title="TOPP-RA Velocity Profile",
        show=False
    )
    plt.savefig("velocity_profile.png", dpi=150)
    plt.close()

    # Acceleration profile
    print("   - Acceleration profile with limits")
    plot_acceleration_profile(
        time_toppra, traj_toppra,
        a_max=a_max,
        joint_names=joint_names,
        title="TOPP-RA Acceleration Profile",
        show=False
    )
    plt.savefig("acceleration_profile.png", dpi=150)
    plt.close()

    # Phase portrait
    print("   - Phase portrait")
    plot_phase_portrait(
        time_toppra, traj_toppra,
        joint_indices=[0, 1, 2],  # First 3 joints
        joint_names=joint_names,
        title="Phase Portrait (Velocity vs Position)",
        show=False
    )
    plt.savefig("phase_portrait.png", dpi=150)
    plt.close()

    # ---------------------------
    # 4. 3D end-effector visualization
    # ---------------------------
    print("\n[4] Creating 3D end-effector plots...")

    # 3D path
    print("   - End-effector 3D path")
    plot_ee_path_3d(
        traj_toppra,
        robot.fk,
        title="End-Effector Path in 3D Space",
        show=False
    )
    plt.savefig("ee_path_3d.png", dpi=150)
    plt.close()

    # Path vs trajectory comparison
    print("   - Path waypoints vs trajectory")
    plot_ee_path_with_waypoints(
        path, traj_toppra, robot.fk,
        title="Path Waypoints vs Smooth Trajectory",
        show=False
    )
    plt.savefig("ee_path_waypoints.png", dpi=150)
    plt.close()

    # EE components over time
    print("   - End-effector X/Y/Z over time")
    plot_ee_components(
        time_toppra, traj_toppra, robot.fk,
        title="End-Effector Position Components",
        show=False
    )
    plt.savefig("ee_components.png", dpi=150)
    plt.close()

    # ---------------------------
    # 5. Robot arm visualization
    # ---------------------------
    print("\n[5] Creating robot arm visualizations...")

    visualizer = RobotVisualizer()

    # Single configuration
    print("   - Start configuration")
    visualizer.plot_configuration(
        q_start,
        title="Start Configuration",
        show=False
    )
    plt.savefig("robot_start.png", dpi=150)
    plt.close()

    # Goal configuration
    print("   - Goal configuration")
    visualizer.plot_configuration(
        q_goal,
        title="Goal Configuration",
        show=False
    )
    plt.savefig("robot_goal.png", dpi=150)
    plt.close()

    # Trajectory snapshots
    print("   - Trajectory snapshots")
    visualizer.plot_trajectory_snapshots(
        traj_toppra,
        n_snapshots=7,
        title="Trajectory Motion",
        show=False
    )
    plt.savefig("trajectory_snapshots.png", dpi=150)
    plt.close()

    # ---------------------------
    # Summary
    # ---------------------------
    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)
    print("\nGenerated files:")
    print("  - joint_trajectory.png")
    print("  - trajectory_comparison.png")
    print("  - velocity_profile.png")
    print("  - acceleration_profile.png")
    print("  - phase_portrait.png")
    print("  - ee_path_3d.png")
    print("  - ee_path_waypoints.png")
    print("  - ee_components.png")
    print("  - robot_start.png")
    print("  - robot_goal.png")
    print("  - trajectory_snapshots.png")

    print("\nTo create an animation, use:")
    print("  from visualization import animate_trajectory, RobotVisualizer")
    print("  viz = RobotVisualizer()")
    print("  anim = animate_trajectory(trajectory, viz, save_path='robot_anim.gif')")
    print("  plt.show()")


if __name__ == "__main__":
    main()
