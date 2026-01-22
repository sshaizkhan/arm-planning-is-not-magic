"""Simple stick-figure robot arm visualization.

This module provides a basic 3D visualization of a 6-DOF robot arm
as a stick figure, useful for:
- Understanding robot configurations
- Animating trajectories
- Debugging planning results

For production visualization, consider PyBullet or MuJoCo.
"""

from typing import Optional, List, Callable, Tuple
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation


class RobotVisualizer:
    """
    Simple stick-figure visualizer for 6-DOF robot arms.

    This visualizer computes joint positions using DH-like transformations
    and renders the robot as connected line segments.
    """

    def __init__(
        self,
        link_lengths: Optional[List[float]] = None,
        base_position: Tuple[float, float, float] = (0, 0, 0),
    ):
        """
        Initialize the robot visualizer.

        Args:
            link_lengths: List of 6 link lengths [d1, a2, a3, d4, d5, d6]
                         If None, uses default UR5-like dimensions
            base_position: (x, y, z) position of robot base
        """
        if link_lengths is None:
            # Default UR5-like link lengths (simplified)
            self.link_lengths = [0.089, 0.425, 0.392, 0.109, 0.095, 0.082]
        else:
            self.link_lengths = link_lengths

        self.base = np.array(base_position)

    def compute_joint_positions(self, q: np.ndarray) -> np.ndarray:
        """
        Compute 3D positions of all joints for visualization.

        This is a simplified model for visualization purposes.
        Uses a standard 6R configuration similar to UR robots.

        Args:
            q: Joint angles, shape (6,)

        Returns:
            Joint positions, shape (7, 3) - base + 6 joints
        """
        d1, a2, a3, d4, d5, d6 = self.link_lengths

        # Initialize with base position
        positions = [self.base.copy()]

        # Current transformation (cumulative)
        pos = self.base.copy()

        # Joint 1 - Rotation about Z (vertical)
        pos = pos + np.array([0, 0, d1])
        positions.append(pos.copy())

        # Joint 2 - Rotation about Y (shoulder)
        c1, s1 = np.cos(q[0]), np.sin(q[0])
        c2, s2 = np.cos(q[1]), np.sin(q[1])

        # Link 2 direction in world frame
        link2_local = np.array([a2 * c2, 0, a2 * s2])
        link2_world = np.array([
            c1 * link2_local[0] - s1 * link2_local[1],
            s1 * link2_local[0] + c1 * link2_local[1],
            link2_local[2]
        ])
        pos = pos + link2_world
        positions.append(pos.copy())

        # Joint 3 - Rotation about Y (elbow)
        c23, s23 = np.cos(q[1] + q[2]), np.sin(q[1] + q[2])
        link3_local = np.array([a3 * c23, 0, a3 * s23])
        link3_world = np.array([
            c1 * link3_local[0] - s1 * link3_local[1],
            s1 * link3_local[0] + c1 * link3_local[1],
            link3_local[2]
        ])
        pos = pos + link3_world
        positions.append(pos.copy())

        # Joints 4, 5, 6 - Wrist (simplified as small offsets)
        # For visualization, we'll add small segments
        c4, s4 = np.cos(q[3]), np.sin(q[3])
        wrist_offset = np.array([
            c1 * d4 * c23,
            s1 * d4 * c23,
            d4 * s23
        ])
        pos = pos + wrist_offset * 0.3
        positions.append(pos.copy())

        # Joint 5
        pos = pos + wrist_offset * 0.3
        positions.append(pos.copy())

        # Joint 6 (end effector)
        pos = pos + wrist_offset * 0.4
        positions.append(pos.copy())

        return np.array(positions)

    def plot_configuration(
        self,
        q: np.ndarray,
        ax: Optional[Axes3D] = None,
        title: str = "Robot Configuration",
        show_joints: bool = True,
        link_color: str = 'blue',
        joint_color: str = 'red',
        figsize: Tuple[int, int] = (10, 8),
        show: bool = True,
    ) -> Tuple[Figure, Axes3D]:
        """
        Plot a single robot configuration.

        Args:
            q: Joint angles, shape (6,)
            ax: Optional existing 3D axes
            title: Plot title
            show_joints: If True, mark joint positions
            link_color: Color for robot links
            joint_color: Color for joint markers
            figsize: Figure size
            show: If True, display immediately

        Returns:
            (figure, axes) tuple
        """
        if ax is None:
            fig = plt.figure(figsize=figsize)
            ax = fig.add_subplot(111, projection='3d')
        else:
            fig = ax.get_figure()

        positions = self.compute_joint_positions(q)

        # Plot links
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                color=link_color, linewidth=3, marker='', zorder=1)

        if show_joints:
            # Plot joints
            ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                       color=joint_color, s=100, marker='o', zorder=2)

            # Mark base
            ax.scatter(*self.base, color='black', s=200, marker='s',
                       label='Base', zorder=3)

            # Mark end effector
            ax.scatter(*positions[-1], color='green', s=150, marker='^',
                       label='End Effector', zorder=3)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(title)

        # Set reasonable axis limits
        all_coords = positions.flatten()
        margin = 0.2
        max_val = max(abs(all_coords.min()), abs(all_coords.max())) + margin
        ax.set_xlim(-max_val, max_val)
        ax.set_ylim(-max_val, max_val)
        ax.set_zlim(-0.1, max_val * 1.5)

        if show_joints:
            ax.legend()

        if show:
            plt.show()

        return fig, ax

    def plot_trajectory_snapshots(
        self,
        trajectory: np.ndarray,
        n_snapshots: int = 5,
        title: str = "Trajectory Snapshots",
        figsize: Tuple[int, int] = (12, 8),
        show: bool = True,
    ) -> Figure:
        """
        Plot multiple configurations from a trajectory.

        Shows the robot at evenly spaced time steps to visualize
        the overall motion.

        Args:
            trajectory: Joint trajectory, shape (N, 6)
            n_snapshots: Number of configurations to show
            title: Plot title
            figsize: Figure size
            show: If True, display immediately

        Returns:
            matplotlib Figure object
        """
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection='3d')

        n_points = len(trajectory)
        indices = np.linspace(0, n_points - 1, n_snapshots, dtype=int)

        # Color gradient from green (start) to red (end)
        colors = plt.cm.coolwarm(np.linspace(0, 1, n_snapshots))

        for i, (idx, color) in enumerate(zip(indices, colors)):
            q = trajectory[idx]
            positions = self.compute_joint_positions(q)

            alpha = 0.3 + 0.7 * (i / (n_snapshots - 1))
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                    color=color, linewidth=2, alpha=alpha)

        # Mark start and end clearly
        start_pos = self.compute_joint_positions(trajectory[0])
        end_pos = self.compute_joint_positions(trajectory[-1])

        ax.scatter(*start_pos[-1], color='green', s=200, marker='o',
                   label='Start', zorder=10, edgecolors='black', linewidths=2)
        ax.scatter(*end_pos[-1], color='red', s=200, marker='s',
                   label='End', zorder=10, edgecolors='black', linewidths=2)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f"{title}\n({n_snapshots} snapshots)")
        ax.legend()

        # Set axis limits based on all positions
        all_positions = []
        for idx in indices:
            all_positions.append(self.compute_joint_positions(trajectory[idx]))
        all_positions = np.vstack(all_positions)

        margin = 0.2
        for i, (setter, getter) in enumerate([
            (ax.set_xlim, lambda p: (p[:, 0].min(), p[:, 0].max())),
            (ax.set_ylim, lambda p: (p[:, 1].min(), p[:, 1].max())),
            (ax.set_zlim, lambda p: (p[:, 2].min(), p[:, 2].max())),
        ]):
            vmin, vmax = getter(all_positions)
            setter(vmin - margin, vmax + margin)

        if show:
            plt.show()

        return fig


def animate_trajectory(
    trajectory: np.ndarray,
    visualizer: Optional[RobotVisualizer] = None,
    interval: int = 50,
    title: str = "Robot Trajectory Animation",
    figsize: Tuple[int, int] = (10, 8),
    save_path: Optional[str] = None,
) -> FuncAnimation:
    """
    Create an animation of the robot moving through a trajectory.

    Args:
        trajectory: Joint trajectory, shape (N, 6)
        visualizer: RobotVisualizer instance (creates default if None)
        interval: Milliseconds between frames
        title: Animation title
        figsize: Figure size
        save_path: If provided, save animation to this file (e.g., 'anim.gif')

    Returns:
        matplotlib FuncAnimation object
    """
    if visualizer is None:
        visualizer = RobotVisualizer()

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    # Compute all positions for axis limits
    all_positions = []
    for q in trajectory:
        all_positions.append(visualizer.compute_joint_positions(q))
    all_positions = np.vstack(all_positions)

    margin = 0.2
    ax.set_xlim(all_positions[:, 0].min() - margin, all_positions[:, 0].max() + margin)
    ax.set_ylim(all_positions[:, 1].min() - margin, all_positions[:, 1].max() + margin)
    ax.set_zlim(all_positions[:, 2].min() - margin, all_positions[:, 2].max() + margin)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # Initialize plot elements
    line, = ax.plot([], [], [], 'b-', linewidth=3)
    joints = ax.scatter([], [], [], color='red', s=100)
    ee_trail, = ax.plot([], [], [], 'g--', linewidth=1, alpha=0.5)

    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)

    ee_history = []

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        joints._offsets3d = ([], [], [])
        ee_trail.set_data([], [])
        ee_trail.set_3d_properties([])
        time_text.set_text('')
        return line, joints, ee_trail, time_text

    def update(frame):
        q = trajectory[frame]
        positions = visualizer.compute_joint_positions(q)

        line.set_data(positions[:, 0], positions[:, 1])
        line.set_3d_properties(positions[:, 2])

        joints._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])

        # Update end-effector trail
        ee_history.append(positions[-1].copy())
        if len(ee_history) > 1:
            trail = np.array(ee_history)
            ee_trail.set_data(trail[:, 0], trail[:, 1])
            ee_trail.set_3d_properties(trail[:, 2])

        progress = 100 * frame / (len(trajectory) - 1)
        time_text.set_text(f'Frame: {frame}/{len(trajectory)-1} ({progress:.0f}%)')

        return line, joints, ee_trail, time_text

    ax.set_title(title)

    anim = FuncAnimation(
        fig, update, init_func=init,
        frames=len(trajectory), interval=interval, blit=False
    )

    if save_path:
        print(f"Saving animation to {save_path}...")
        anim.save(save_path, writer='pillow', fps=1000//interval)
        print("Done!")

    return anim
