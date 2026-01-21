"""Joint-space trajectory visualization.

This module provides functions to visualize trajectories in joint space:
- Position vs time for each joint
- Velocity profiles
- Acceleration profiles
- Phase portraits (velocity vs position)

These plots are essential for:
- Verifying constraint satisfaction
- Comparing different parameterizers (TOPP-RA vs Ruckig)
- Understanding trajectory quality
"""

from typing import List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure


def plot_joint_trajectory(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    joint_names: Optional[List[str]] = None,
    title: str = "Joint Trajectory",
    figsize: Tuple[int, int] = (12, 8),
    show: bool = True,
) -> Figure:
    """
    Plot joint positions over time.

    Args:
        time_stamps: Array of time values, shape (N,)
        trajectory: Joint positions, shape (N, dof)
        joint_names: Optional names for each joint
        title: Plot title
        figsize: Figure size (width, height)
        show: If True, display the plot immediately

    Returns:
        matplotlib Figure object
    """
    n_points, dof = trajectory.shape

    if joint_names is None:
        joint_names = [f"Joint {i+1}" for i in range(dof)]

    fig, axes = plt.subplots(dof, 1, figsize=figsize, sharex=True)
    if dof == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, dof))

    for i, (ax, name, color) in enumerate(zip(axes, joint_names, colors)):
        ax.plot(time_stamps, trajectory[:, i], color=color, linewidth=1.5)
        ax.set_ylabel(f"{name}\n(rad)", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title(title)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_joint_comparison(
    time_stamps_1: np.ndarray,
    trajectory_1: np.ndarray,
    time_stamps_2: np.ndarray,
    trajectory_2: np.ndarray,
    label_1: str = "Trajectory 1",
    label_2: str = "Trajectory 2",
    joint_names: Optional[List[str]] = None,
    title: str = "Trajectory Comparison",
    figsize: Tuple[int, int] = (12, 10),
    show: bool = True,
) -> Figure:
    """
    Compare two trajectories side by side.

    Useful for comparing TOPP-RA vs Ruckig or different planner outputs.

    Args:
        time_stamps_1, trajectory_1: First trajectory data
        time_stamps_2, trajectory_2: Second trajectory data
        label_1, label_2: Labels for legend
        joint_names: Optional joint names
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    dof = trajectory_1.shape[1]

    if joint_names is None:
        joint_names = [f"Joint {i+1}" for i in range(dof)]

    fig, axes = plt.subplots(dof, 1, figsize=figsize, sharex=True)
    if dof == 1:
        axes = [axes]

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
        ax.plot(time_stamps_1, trajectory_1[:, i], 'b-', linewidth=1.5, label=label_1)
        ax.plot(time_stamps_2, trajectory_2[:, i], 'r--', linewidth=1.5, label=label_2)
        ax.set_ylabel(f"{name}\n(rad)", fontsize=9)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc='upper right')

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title(title)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def compute_derivatives(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute velocity and acceleration from trajectory via finite differences.

    Args:
        time_stamps: Time array, shape (N,)
        trajectory: Position array, shape (N, dof)

    Returns:
        velocity: shape (N-1, dof)
        acceleration: shape (N-2, dof)
    """
    dt = np.diff(time_stamps)

    # Velocity: dq/dt
    dq = np.diff(trajectory, axis=0)
    velocity = dq / dt[:, np.newaxis]

    # Acceleration: d²q/dt²
    dv = np.diff(velocity, axis=0)
    dt_acc = (dt[:-1] + dt[1:]) / 2  # Average dt for acceleration
    acceleration = dv / dt_acc[:, np.newaxis]

    return velocity, acceleration


def plot_velocity_profile(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    v_max: Optional[np.ndarray] = None,
    joint_names: Optional[List[str]] = None,
    title: str = "Velocity Profile",
    figsize: Tuple[int, int] = (12, 8),
    show: bool = True,
) -> Figure:
    """
    Plot joint velocities over time with optional limit lines.

    Args:
        time_stamps: Time array, shape (N,)
        trajectory: Position array, shape (N, dof)
        v_max: Optional velocity limits, shape (dof,)
        joint_names: Optional joint names
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    velocity, _ = compute_derivatives(time_stamps, trajectory)
    dof = trajectory.shape[1]
    t_vel = (time_stamps[:-1] + time_stamps[1:]) / 2  # Midpoints

    if joint_names is None:
        joint_names = [f"Joint {i+1}" for i in range(dof)]

    fig, axes = plt.subplots(dof, 1, figsize=figsize, sharex=True)
    if dof == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, dof))

    for i, (ax, name, color) in enumerate(zip(axes, joint_names, colors)):
        ax.plot(t_vel, velocity[:, i], color=color, linewidth=1.5)
        ax.set_ylabel(f"{name}\n(rad/s)", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

        # Plot velocity limits if provided
        if v_max is not None:
            ax.axhline(y=v_max[i], color='red', linestyle='--', alpha=0.7, label='limit')
            ax.axhline(y=-v_max[i], color='red', linestyle='--', alpha=0.7)
            ax.fill_between(t_vel, -v_max[i], v_max[i], alpha=0.1, color='green')

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title(title)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_acceleration_profile(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    a_max: Optional[np.ndarray] = None,
    joint_names: Optional[List[str]] = None,
    title: str = "Acceleration Profile",
    figsize: Tuple[int, int] = (12, 8),
    show: bool = True,
) -> Figure:
    """
    Plot joint accelerations over time with optional limit lines.

    Args:
        time_stamps: Time array, shape (N,)
        trajectory: Position array, shape (N, dof)
        a_max: Optional acceleration limits, shape (dof,)
        joint_names: Optional joint names
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    _, acceleration = compute_derivatives(time_stamps, trajectory)
    dof = trajectory.shape[1]

    # Time for acceleration (need two midpoint operations)
    t_mid = (time_stamps[:-1] + time_stamps[1:]) / 2
    t_acc = (t_mid[:-1] + t_mid[1:]) / 2

    if joint_names is None:
        joint_names = [f"Joint {i+1}" for i in range(dof)]

    fig, axes = plt.subplots(dof, 1, figsize=figsize, sharex=True)
    if dof == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, dof))

    for i, (ax, name, color) in enumerate(zip(axes, joint_names, colors)):
        ax.plot(t_acc, acceleration[:, i], color=color, linewidth=1.5)
        ax.set_ylabel(f"{name}\n(rad/s²)", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

        # Plot acceleration limits if provided
        if a_max is not None:
            ax.axhline(y=a_max[i], color='red', linestyle='--', alpha=0.7)
            ax.axhline(y=-a_max[i], color='red', linestyle='--', alpha=0.7)
            ax.fill_between(t_acc, -a_max[i], a_max[i], alpha=0.1, color='green')

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title(title)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_phase_portrait(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    joint_indices: Optional[List[int]] = None,
    joint_names: Optional[List[str]] = None,
    title: str = "Phase Portrait (Velocity vs Position)",
    figsize: Tuple[int, int] = (12, 8),
    show: bool = True,
) -> Figure:
    """
    Plot phase portrait (velocity vs position) for selected joints.

    Phase portraits show the relationship between position and velocity,
    revealing the dynamic behavior of the trajectory.

    Args:
        time_stamps: Time array, shape (N,)
        trajectory: Position array, shape (N, dof)
        joint_indices: Which joints to plot (default: all)
        joint_names: Optional joint names
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    velocity, _ = compute_derivatives(time_stamps, trajectory)
    dof = trajectory.shape[1]

    if joint_indices is None:
        joint_indices = list(range(dof))

    if joint_names is None:
        joint_names = [f"Joint {i+1}" for i in range(dof)]

    n_joints = len(joint_indices)
    cols = min(3, n_joints)
    rows = (n_joints + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    if n_joints == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    colors = plt.cm.viridis(np.linspace(0, 1, len(velocity)))

    for idx, (ax, joint_idx) in enumerate(zip(axes[:n_joints], joint_indices)):
        # Plot with color gradient showing time progression
        for i in range(len(velocity) - 1):
            ax.plot(
                trajectory[i:i+2, joint_idx],
                velocity[i:i+2, joint_idx],
                color=colors[i],
                linewidth=1.5
            )

        # Mark start and end
        ax.scatter(trajectory[0, joint_idx], velocity[0, joint_idx],
                   color='green', s=100, marker='o', label='Start', zorder=5)
        ax.scatter(trajectory[-2, joint_idx], velocity[-1, joint_idx],
                   color='red', s=100, marker='s', label='End', zorder=5)

        ax.set_xlabel("Position (rad)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title(joint_names[joint_idx])
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(x=0, color='gray', linestyle='--', alpha=0.5)

        if idx == 0:
            ax.legend(loc='upper right')

    # Hide unused subplots
    for ax in axes[n_joints:]:
        ax.set_visible(False)

    fig.suptitle(title, fontsize=12)
    plt.tight_layout()

    if show:
        plt.show()

    return fig
