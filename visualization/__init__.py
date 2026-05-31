"""Visualization utilities for robot arm motion planning.

This module provides tools to visualize:
- Joint-space trajectories (position, velocity, acceleration vs time)
- End-effector paths in 3D Cartesian space
- Simple stick-figure robot arm animations
- PyBullet-based 3D visualization
- Meshcat browser-based 3D visualization

Usage:
    from visualization import plot_joint_trajectory, plot_ee_path_3d, RobotVisualizer
    from visualization import PyBulletVisualizer
    from visualization import MeshcatVisualizer
"""

from visualization.path_3d import (
    plot_ee_path_3d,
    plot_ee_path_with_waypoints,
)
from visualization.robot_visualizer import (
    RobotVisualizer,
    animate_trajectory,
)
from visualization.trajectory_plots import (
    plot_acceleration_profile,
    plot_joint_comparison,
    plot_joint_trajectory,
    plot_phase_portrait,
    plot_velocity_profile,
)

# PyBullet and Meshcat visualizers are imported lazily via __getattr__ below.
# Importing them eagerly loads heavy C extensions (pybullet prints a build-time
# banner on import), which pollutes stdout for users who only need one backend.


def _installed(module: str) -> bool:
    """True if a module can be located. Treats already-imported modules
    (including test mocks without a spec) as available."""
    import sys
    if module in sys.modules:
        return True
    import importlib.util
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False


def _pybullet_available() -> bool:
    return _installed("pybullet")


def _meshcat_available() -> bool:
    return _installed("meshcat") and _installed("yourdfpy")


# Module-level availability flags (cheap — only checks if the package is installed,
# does not import it).
PYBULLET_AVAILABLE = _pybullet_available()
MESHCAT_AVAILABLE = _meshcat_available()


def __getattr__(name):
    """Lazily import visualizer classes only when first accessed."""
    if name == "PyBulletVisualizer":
        if not PYBULLET_AVAILABLE:
            return None
        from visualization.pybullet_visualizer import PyBulletVisualizer
        return PyBulletVisualizer
    if name == "MeshcatVisualizer":
        if not MESHCAT_AVAILABLE:
            return None
        from visualization.meshcat_visualizer import MeshcatVisualizer
        return MeshcatVisualizer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


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
    # Meshcat visualization
    "MeshcatVisualizer",
    "MESHCAT_AVAILABLE",
]
