#!/usr/bin/env python3
"""
Demo 07: Multi-Planner Trajectory Visualization.

This demo runs multiple motion planners on the same problem and visualizes
their trajectories side-by-side, allowing users to compare:
- Path quality (smoothness, efficiency)
- Planning time
- Trajectory duration
- End-effector path in 3D

Visualization options:
1. Overlaid 3D plot - All trajectories in one view
2. Grid view - Each planner in its own subplot
3. PyBullet - Interactive 3D with colored trajectories (optional)
"""

import argparse
import numpy as np
import time
from typing import Dict, List, Optional, Tuple

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager, ShapeCollisionManager, Box
from planners import (
    OMPLRRTPlanner,
    OMPLRRTConnectPlanner,
    OMPLRRTStarPlanner,
    OMPLPRMPlanner,
    OMPLKPIECE1Planner,
    OMPLESTPlanner,
    OMPLBiTRRTPlanner,
)
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from visualization.path_3d import (
    plot_multi_planner_comparison,
    plot_multi_planner_grid,
)


# Available planners
PLANNERS = {
    "RRT": (OMPLRRTPlanner, {"step_size": 0.1}),
    "RRT-Connect": (OMPLRRTConnectPlanner, {"step_size": 0.1}),
    "RRT*": (OMPLRRTStarPlanner, {"step_size": 0.1}),
    "PRM": (OMPLPRMPlanner, {}),
    "KPIECE1": (OMPLKPIECE1Planner, {}),
    "EST": (OMPLESTPlanner, {"step_size": 0.1}),
    "BiTRRT": (OMPLBiTRRTPlanner, {"step_size": 0.1}),
}

# Colorblind-friendly colors for PyBullet visualization
PYBULLET_COLORS = [
    (0.12, 0.47, 0.71),  # blue
    (1.00, 0.50, 0.05),  # orange
    (0.17, 0.63, 0.17),  # green
    (0.84, 0.15, 0.16),  # red
    (0.58, 0.40, 0.74),  # purple
    (0.55, 0.34, 0.29),  # brown
    (0.89, 0.47, 0.76),  # pink
]


def compute_path_length(path: List[np.ndarray]) -> float:
    """Compute total joint-space path length."""
    if path is None or len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += np.linalg.norm(path[i + 1] - path[i])
    return total


def run_planner(
    planner_name: str,
    state_space: JointStateSpace,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    timeout: float = 2.0,
) -> Dict:
    """
    Run a single planner and return results.

    Returns:
        Dictionary with keys: success, path, trajectory, timestamps,
        planning_time, path_length, duration, num_waypoints
    """
    planner_class, kwargs = PLANNERS[planner_name]

    try:
        planner = planner_class(state_space, **kwargs)
    except TypeError:
        planner = planner_class(state_space)

    start_time = time.time()
    path = planner.plan(q_start, q_goal, timeout=timeout)
    planning_time = time.time() - start_time

    if path is None:
        return {
            "success": False,
            "path": None,
            "trajectory": None,
            "timestamps": None,
            "planning_time": planning_time,
            "path_length": 0.0,
            "duration": 0.0,
            "num_waypoints": 0,
        }

    path_length = compute_path_length(path)

    # Time-parameterize the path
    dof = state_space.dim
    v_max = np.ones(dof) * 1.0
    a_max = np.ones(dof) * 2.0

    try:
        param = ToppraTimeParameterizer(v_max, a_max)
        timestamps, trajectory = param.compute(path)
        duration = timestamps[-1] if len(timestamps) > 0 else 0.0
    except Exception as e:
        print(f"  Warning: Parameterization failed for {planner_name}: {e}")
        timestamps = None
        trajectory = None
        duration = 0.0

    return {
        "success": True,
        "path": path,
        "trajectory": trajectory,
        "timestamps": timestamps,
        "planning_time": planning_time,
        "path_length": path_length,
        "duration": duration,
        "num_waypoints": len(path),
    }


def run_all_planners(
    state_space: JointStateSpace,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    planners: Optional[List[str]] = None,
    timeout: float = 2.0,
) -> Dict[str, Dict]:
    """
    Run multiple planners on the same problem.

    Args:
        state_space: Joint state space with collision checking
        q_start: Start configuration
        q_goal: Goal configuration
        planners: List of planner names to run. If None, runs all.
        timeout: Planning timeout per planner

    Returns:
        Dictionary mapping planner names to result dictionaries
    """
    if planners is None:
        planners = list(PLANNERS.keys())

    results = {}
    for name in planners:
        if name not in PLANNERS:
            print(f"Unknown planner: {name}")
            continue

        print(f"Running {name}...", end=" ", flush=True)
        result = run_planner(name, state_space, q_start, q_goal, timeout)

        status = "OK" if result["success"] else "FAILED"
        print(f"{status} ({result['planning_time']:.3f}s)")

        results[name] = result

    return results


def print_metrics_table(results: Dict[str, Dict]):
    """Print a comparison table of planner metrics."""
    print("\n" + "=" * 85)
    print("PLANNER COMPARISON")
    print("=" * 85)

    headers = ["Planner", "Status", "Plan Time", "Path Len", "Waypoints", "Exec Time"]
    widths = [15, 8, 12, 12, 10, 12]

    header_row = " | ".join(h.center(w) for h, w in zip(headers, widths))
    print(header_row)
    print("-" * len(header_row))

    for name, r in results.items():
        if r["success"]:
            row = [
                name[:widths[0]].ljust(widths[0]),
                "OK".center(widths[1]),
                f"{r['planning_time']:.4f}".rjust(widths[2]),
                f"{r['path_length']:.4f}".rjust(widths[3]),
                str(r['num_waypoints']).rjust(widths[4]),
                f"{r['duration']:.4f}".rjust(widths[5]),
            ]
        else:
            row = [
                name[:widths[0]].ljust(widths[0]),
                "FAIL".center(widths[1]),
                f"{r['planning_time']:.4f}".rjust(widths[2]),
                "-".rjust(widths[3]),
                "-".rjust(widths[4]),
                "-".rjust(widths[5]),
            ]
        print(" | ".join(row))

    print("=" * 85)

    # Find best performers
    successful = [(n, r) for n, r in results.items() if r["success"]]
    if successful:
        fastest_plan = min(successful, key=lambda x: x[1]["planning_time"])
        shortest_path = min(successful, key=lambda x: x[1]["path_length"])
        fastest_exec = min(successful, key=lambda x: x[1]["duration"])

        print(f"\nFastest planning:  {fastest_plan[0]} ({fastest_plan[1]['planning_time']:.4f}s)")
        print(f"Shortest path:     {shortest_path[0]} ({shortest_path[1]['path_length']:.4f})")
        print(f"Fastest execution: {fastest_exec[0]} ({fastest_exec[1]['duration']:.4f}s)")


def visualize_pybullet(
    results: Dict[str, Dict],
    robot: UR5RobotModel,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    obstacles: Optional[List] = None,
):
    """
    Visualize trajectories in PyBullet with different colors.

    Shows all planned paths overlaid in the same 3D scene.
    """
    try:
        from visualization.pybullet_visualizer import PyBulletVisualizer
    except ImportError:
        print("PyBullet not available. Skipping 3D visualization.")
        return

    print("\nStarting PyBullet visualization...")
    print("Controls: Drag to rotate, scroll to zoom, press 'q' to quit")

    with PyBulletVisualizer(gui=True) as viz:
        # Add obstacles if any
        if obstacles:
            for obs in obstacles:
                if hasattr(obs, 'center') and hasattr(obs, 'size'):
                    viz.add_box(obs.center, obs.size, color=(1.0, 0.0, 0.0, 0.5))

        # Show start configuration
        viz.visualize_configuration(q_start, duration=0.5)

        # Draw all trajectories as colored paths
        successful_results = {n: r for n, r in results.items() if r["success"] and r.get("trajectory") is not None}

        print(f"\nVisualizing {len(successful_results)} trajectories:")
        for idx, (name, result) in enumerate(successful_results.items()):
            color = PYBULLET_COLORS[idx % len(PYBULLET_COLORS)]
            print(f"  - {name}: RGB{tuple(round(c, 2) for c in color)}")

            viz.visualize_trajectory_path(
                result["trajectory"],
                fk_func=robot.fk,
                color=color,
                line_width=3,
                sample_every=2,
            )

        # Add markers for start and goal
        start_pose = robot.fk(q_start)
        goal_pose = robot.fk(q_goal)
        viz.add_marker(start_pose[:3, 3], color=(0, 1, 0), size=0.03)  # Green start
        viz.add_marker(goal_pose[:3, 3], color=(1, 0, 0), size=0.03)   # Red goal

        # Animate through each trajectory
        print("\nAnimating trajectories sequentially. Press 'q' to skip.")
        for name, result in successful_results.items():
            print(f"  Playing: {name}")
            viz.visualize_trajectory(
                result["trajectory"],
                result["timestamps"],
                real_time=True,
                speed=1.5,
                show_ee_trail=False,  # Already showing path lines
            )
            time.sleep(0.5)

        # Hold final view
        print("\nVisualization complete. Close window or press 'q' to exit.")
        input("Press Enter to close...")


def main():
    parser = argparse.ArgumentParser(
        description="Compare motion planners with trajectory visualization"
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        default=None,
        choices=list(PLANNERS.keys()),
        help="Planners to compare (default: all)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Planning timeout per planner (default: 2.0s)",
    )
    parser.add_argument(
        "--view",
        choices=["overlay", "grid", "both"],
        default="both",
        help="Visualization mode (default: both)",
    )
    parser.add_argument(
        "--pybullet",
        action="store_true",
        help="Enable PyBullet 3D visualization",
    )
    parser.add_argument(
        "--obstacles",
        action="store_true",
        help="Add obstacles to make planning more interesting",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Save plots to file (e.g., 'comparison.png')",
    )
    args = parser.parse_args()

    # Setup robot and state space
    print("=" * 60)
    print("MULTI-PLANNER TRAJECTORY VISUALIZATION")
    print("=" * 60)

    robot = UR5RobotModel()

    # Setup collision checking
    if args.obstacles:
        print("\nSetting up environment with obstacles...")
        collision_manager = ShapeCollisionManager(robot)
        # Add an obstacle in the workspace
        obstacle = Box(center=(0.4, 0.0, 0.4), size=(0.15, 0.3, 0.15))
        collision_manager.add_shape(obstacle)
        robot.set_collision_manager(collision_manager)
        obstacles = [obstacle]
        print(f"  Added box obstacle at {obstacle.center}")
    else:
        collision_manager = NullCollisionManager()
        robot.in_collision = collision_manager.in_collision
        obstacles = None

    state_space = JointStateSpace(robot)

    # Define start and goal
    q_start = np.array([0.0, -0.5, 0.5, -0.5, -0.5, 0.0])
    q_goal = np.array([1.5, -1.0, 1.0, -1.0, 0.5, 0.5])

    print(f"\nStart: {np.round(q_start, 3)}")
    print(f"Goal:  {np.round(q_goal, 3)}")

    # Run planners
    print("\n" + "-" * 60)
    print("Running planners...")
    print("-" * 60)

    results = run_all_planners(
        state_space,
        q_start,
        q_goal,
        planners=args.planners,
        timeout=args.timeout,
    )

    # Print metrics table
    print_metrics_table(results)

    # Filter successful results for visualization
    successful_results = {
        name: result
        for name, result in results.items()
        if result["success"] and result.get("trajectory") is not None
    }

    if not successful_results:
        print("\nNo successful plans to visualize!")
        return

    print(f"\nVisualizing {len(successful_results)} successful trajectories...")

    # Matplotlib visualization
    if args.view in ["overlay", "both"]:
        fig1 = plot_multi_planner_comparison(
            successful_results,
            robot.fk,
            title="Multi-Planner Trajectory Comparison (Overlaid)",
            show_waypoints=True,
            show=not args.save,
        )
        if args.save:
            overlay_path = args.save.replace(".", "_overlay.")
            fig1.savefig(overlay_path, dpi=150, bbox_inches='tight')
            print(f"Saved overlay plot to: {overlay_path}")

    if args.view in ["grid", "both"]:
        fig2 = plot_multi_planner_grid(
            successful_results,
            robot.fk,
            title="Multi-Planner Comparison (Individual Views)",
            show=not args.save,
        )
        if args.save:
            grid_path = args.save.replace(".", "_grid.")
            fig2.savefig(grid_path, dpi=150, bbox_inches='tight')
            print(f"Saved grid plot to: {grid_path}")

    if args.save:
        import matplotlib.pyplot as plt
        plt.show()

    # PyBullet visualization
    if args.pybullet:
        visualize_pybullet(results, robot, q_start, q_goal, obstacles)


if __name__ == "__main__":
    main()
