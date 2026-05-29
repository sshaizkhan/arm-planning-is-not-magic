"""3D End-effector path visualization.

This module provides functions to visualize the end-effector trajectory
in 3D Cartesian space using forward kinematics.

Key insight:
-----------
While planning happens in joint space (C-space), humans understand
Cartesian space better. These visualizations bridge the gap by showing
what the robot's tool tip actually does in the real world.
"""

from typing import Callable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure


def extract_ee_positions(
    trajectory: np.ndarray,
    fk_func: Callable[[np.ndarray], np.ndarray],
) -> np.ndarray:
    """
    Extract end-effector positions from joint trajectory using FK.

    Args:
        trajectory: Joint positions, shape (N, dof)
        fk_func: Forward kinematics function that takes q and returns 4x4 matrix

    Returns:
        End-effector positions, shape (N, 3)
    """
    n_points = trajectory.shape[0]
    ee_positions = np.zeros((n_points, 3))

    for i in range(n_points):
        pose = fk_func(trajectory[i])
        if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
            ee_positions[i] = pose[:3, 3]
        else:
            raise ValueError(f"FK must return 4x4 matrix, got {type(pose)}")

    return ee_positions


def plot_ee_path_3d(
    trajectory: np.ndarray,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "End-Effector Path in 3D",
    figsize: Tuple[int, int] = (10, 8),
    show_start_end: bool = True,
    color_by_time: bool = True,
    show: bool = True,
) -> Figure:
    """
    Plot the end-effector path in 3D Cartesian space.

    Args:
        trajectory: Joint positions, shape (N, dof)
        fk_func: Forward kinematics function (q -> 4x4 matrix)
        title: Plot title
        figsize: Figure size
        show_start_end: If True, mark start and end points
        color_by_time: If True, color the path by time progression
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    ee_positions = extract_ee_positions(trajectory, fk_func)

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    x, y, z = ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2]

    if color_by_time:
        # Create line segments colored by time
        points = ee_positions.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Color by time
        colors = plt.cm.viridis(np.linspace(0, 1, len(segments)))
        for i, (seg, color) in enumerate(zip(segments, colors)):
            ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], color=color, linewidth=2)

        # Add colorbar
        sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, 1))
        sm.set_array([])
        _ = plt.colorbar(sm, ax=ax, shrink=0.6, label='Time (normalized)')
    else:
        ax.plot(x, y, z, 'b-', linewidth=2)

    if show_start_end:
        ax.scatter(*ee_positions[0], color='green', s=150, marker='o',
                   label='Start', edgecolors='black', linewidths=1)
        ax.scatter(*ee_positions[-1], color='red', s=150, marker='s',
                   label='End', edgecolors='black', linewidths=1)
        ax.legend()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)

    # Make axes equal scale
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_ee_path_with_waypoints(
    path: List[np.ndarray],
    trajectory: np.ndarray,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "Path vs Trajectory (End-Effector)",
    figsize: Tuple[int, int] = (10, 8),
    show: bool = True,
) -> Figure:
    """
    Compare original path waypoints with time-parameterized trajectory.

    This visualization shows:
    - Original path waypoints (from planner) as large markers
    - Smooth trajectory (after time parameterization) as a line

    Useful for understanding how parameterization interpolates the path.

    Args:
        path: List of joint configurations (planner output)
        trajectory: Dense joint trajectory (parameterizer output)
        fk_func: Forward kinematics function
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    # Convert path list to array
    path_array = np.array(path)

    # Get EE positions
    path_ee = extract_ee_positions(path_array, fk_func)
    traj_ee = extract_ee_positions(trajectory, fk_func)

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory (smooth line)
    ax.plot(traj_ee[:, 0], traj_ee[:, 1], traj_ee[:, 2],
            'b-', linewidth=1.5, alpha=0.7, label='Trajectory')

    # Plot path waypoints
    ax.scatter(path_ee[:, 0], path_ee[:, 1], path_ee[:, 2],
               color='orange', s=80, marker='o', edgecolors='black',
               linewidths=1, label=f'Path waypoints ({len(path)})', zorder=5)

    # Mark start and end
    ax.scatter(*path_ee[0], color='green', s=150, marker='^',
               edgecolors='black', linewidths=2, label='Start', zorder=10)
    ax.scatter(*path_ee[-1], color='red', s=150, marker='v',
               edgecolors='black', linewidths=2, label='End', zorder=10)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()

    # Equal aspect ratio
    x_all = np.concatenate([path_ee[:, 0], traj_ee[:, 0]])
    y_all = np.concatenate([path_ee[:, 1], traj_ee[:, 1]])
    z_all = np.concatenate([path_ee[:, 2], traj_ee[:, 2]])

    max_range = np.array([x_all.max()-x_all.min(),
                          y_all.max()-y_all.min(),
                          z_all.max()-z_all.min()]).max() / 2.0

    mid_x = (x_all.max()+x_all.min()) * 0.5
    mid_y = (y_all.max()+y_all.min()) * 0.5
    mid_z = (z_all.max()+z_all.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_ee_components(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "End-Effector Position Components",
    figsize: Tuple[int, int] = (12, 6),
    show: bool = True,
) -> Figure:
    """
    Plot X, Y, Z components of end-effector position over time.

    Args:
        time_stamps: Time array, shape (N,)
        trajectory: Joint positions, shape (N, dof)
        fk_func: Forward kinematics function
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure object
    """
    ee_positions = extract_ee_positions(trajectory, fk_func)

    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
    labels = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']

    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(time_stamps, ee_positions[:, i], color=color, linewidth=1.5)
        ax.set_ylabel(f'{label} (m)')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title(title)

    plt.tight_layout()

    if show:
        plt.show()

    return fig


def plot_multi_planner_3d(
    planner_results: dict,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "Multi-Planner End-Effector Paths",
    figsize: Tuple[int, int] = (10, 8),
    show_waypoints: bool = False,
    show: bool = True,
) -> Figure:
    """
    Plot all planner trajectories in a single 3D axes, color-coded by planner.

    Args:
        planner_results: Dict mapping planner names to result dicts with keys:
            - 'trajectory': Dense joint trajectory (N, dof)
            - 'path': (optional) Original waypoints
        fk_func: Forward kinematics function returning 4x4 matrix
        title: Plot title
        figsize: Figure size
        show_waypoints: If True, also plot raw path waypoints
        show: If True, display immediately

    Returns:
        matplotlib Figure
    """
    colors = [
        '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
        '#9467bd', '#8c564b', '#e377c2', '#7f7f7f',
    ]

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    all_positions = []

    for idx, (planner_name, result) in enumerate(planner_results.items()):
        color = colors[idx % len(colors)]

        if result.get('trajectory') is not None:
            ee_positions = extract_ee_positions(result['trajectory'], fk_func)
            all_positions.append(ee_positions)
            ax.plot(
                ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2],
                color=color, linewidth=2, alpha=0.8, label=planner_name,
            )

        if show_waypoints and result.get('path') is not None:
            path_ee = extract_ee_positions(np.array(result['path']), fk_func)
            ax.scatter(
                path_ee[:, 0], path_ee[:, 1], path_ee[:, 2],
                color=color, s=30, alpha=0.5, edgecolors='black', linewidths=0.5,
            )

    if all_positions:
        first = all_positions[0]
        ax.scatter(*first[0], color='green', s=200, marker='^', edgecolors='black',
                   linewidths=2, label='Start', zorder=10)
        ax.scatter(*first[-1], color='red', s=200, marker='v', edgecolors='black',
                   linewidths=2, label='Goal', zorder=10)

        all_pos = np.vstack(all_positions)
        rng = np.array([
            all_pos[:, 0].max() - all_pos[:, 0].min(),
            all_pos[:, 1].max() - all_pos[:, 1].min(),
            all_pos[:, 2].max() - all_pos[:, 2].min(),
        ]).max() / 2.0
        mid = all_pos.mean(axis=0)
        ax.set_xlim(mid[0] - rng, mid[0] + rng)
        ax.set_ylim(mid[1] - rng, mid[1] + rng)
        ax.set_zlim(mid[2] - rng, mid[2] + rng)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend(loc='upper left', fontsize=9)
    plt.tight_layout()

    if show:
        plt.show()
    return fig


def plot_multi_planner_grid(
    planner_results: dict,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "Multi-Planner Comparison",
    figsize: Tuple[int, int] = (16, 12),
    show: bool = True,
) -> Figure:
    """
    Show each planner in its own 3D subplot with per-planner metrics as subtitle.

    Args:
        planner_results: Dict mapping planner names to result dicts
        fk_func: Forward kinematics function
        title: Overall figure title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure
    """
    n = len(planner_results)
    if n == 0:
        raise ValueError("No planner results provided")

    n_cols = min(3, n)
    n_rows = (n + n_cols - 1) // n_cols
    colors = [
        '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
        '#9467bd', '#8c564b', '#e377c2', '#7f7f7f',
    ]

    fig = plt.figure(figsize=figsize)
    fig.suptitle(title, fontsize=14, fontweight='bold')

    all_positions = [
        extract_ee_positions(r['trajectory'], fk_func)
        for r in planner_results.values()
        if r.get('trajectory') is not None
    ]

    limits = None
    if all_positions:
        all_pos = np.vstack(all_positions)
        rng = np.array([
            all_pos[:, 0].max() - all_pos[:, 0].min(),
            all_pos[:, 1].max() - all_pos[:, 1].min(),
            all_pos[:, 2].max() - all_pos[:, 2].min(),
        ]).max() / 2.0 * 1.1
        mid = all_pos.mean(axis=0)
        limits = (mid, rng)

    for idx, (name, result) in enumerate(planner_results.items()):
        ax = fig.add_subplot(n_rows, n_cols, idx + 1, projection='3d')
        color = colors[idx % len(colors)]

        if result.get('trajectory') is not None:
            ee = extract_ee_positions(result['trajectory'], fk_func)
            ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], color=color, linewidth=2)
            ax.scatter(*ee[0], color='green', s=100, marker='^', zorder=10)
            ax.scatter(*ee[-1], color='red', s=100, marker='v', zorder=10)

            parts = [name]
            if result.get('planning_time') is not None:
                parts.append(f"Plan: {result['planning_time']:.3f}s")
            if result.get('duration') is not None:
                parts.append(f"Exec: {result['duration']:.2f}s")
            ax.set_title('\n'.join(parts), fontsize=10)
        else:
            ax.text2D(0.5, 0.5, 'Failed', ha='center', va='center',
                      fontsize=12, color='red', transform=ax.transAxes)
            ax.set_title(f"{name}\n(Failed)", fontsize=10, color='red')

        ax.set_xlabel('X', fontsize=8)
        ax.set_ylabel('Y', fontsize=8)
        ax.set_zlabel('Z', fontsize=8)

        if limits:
            mid, rng = limits
            ax.set_xlim(mid[0] - rng, mid[0] + rng)
            ax.set_ylim(mid[1] - rng, mid[1] + rng)
            ax.set_zlim(mid[2] - rng, mid[2] + rng)

    plt.tight_layout()
    if show:
        plt.show()
    return fig


def plot_ee_components_over_time(
    time_stamps: np.ndarray,
    trajectory: np.ndarray,
    fk_func: Callable[[np.ndarray], np.ndarray],
    title: str = "End-Effector Position Components",
    figsize: Tuple[int, int] = (12, 6),
    show: bool = True,
) -> Figure:
    """
    Plot X, Y, Z components of end-effector position over time.

    Args:
        time_stamps: Time values, shape (N,)
        trajectory: Joint positions, shape (N, dof)
        fk_func: Forward kinematics function
        title: Plot title
        figsize: Figure size
        show: If True, display immediately

    Returns:
        matplotlib Figure
    """
    ee_positions = extract_ee_positions(trajectory, fk_func)

    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
    for i, (ax, label, color) in enumerate(zip(axes, ['X', 'Y', 'Z'], ['red', 'green', 'blue'])):
        ax.plot(time_stamps, ee_positions[:, i], color=color, linewidth=1.5)
        ax.set_ylabel(f'{label} (m)')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title(title)
    plt.tight_layout()

    if show:
        plt.show()
    return fig
