"""Visualization utilities for robot arm motion planning.

This module provides tools to visualize:
- Joint-space trajectories (position, velocity, acceleration vs time)
- End-effector paths in 3D Cartesian space
- Simple stick-figure robot arm animations
- PyBullet-based 3D visualization

Usage:
    from visualization import plot_joint_trajectory, plot_ee_path_3d, RobotVisualizer
    from visualization import PyBulletVisualizer
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

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    PyBulletVisualizer = None

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
    # PyBullet visualization
    "PyBulletVisualizer",
    "PYBULLET_AVAILABLE",
]
