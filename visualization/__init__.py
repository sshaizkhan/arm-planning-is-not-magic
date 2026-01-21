"""Visualization utilities for robot arm motion planning.

This module provides tools to visualize:
- Joint-space trajectories (position, velocity, acceleration vs time)
- End-effector paths in 3D Cartesian space
- Simple stick-figure robot arm animations

Usage:
    from visualization import plot_joint_trajectory, plot_ee_path_3d, RobotVisualizer
"""

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
)
from visualization.robot_visualizer import (
    RobotVisualizer,
    animate_trajectory,
)

__all__ = [
    # Joint-space plots
    "plot_joint_trajectory",
    "plot_joint_comparison",
    "plot_velocity_profile",
    "plot_acceleration_profile",
    "plot_phase_portrait",
    # 3D path plots
    "plot_ee_path_3d",
    "plot_ee_path_with_waypoints",
    # Robot visualization
    "RobotVisualizer",
    "animate_trajectory",
]
