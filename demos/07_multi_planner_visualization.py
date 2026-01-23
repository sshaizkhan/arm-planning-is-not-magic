#!/usr/bin/env python3
"""
Demo 07: Multi-Planner Trajectory Visualization in PyBullet.

This demo runs multiple motion planners on the same problem and visualizes
their trajectories in PyBullet 3D, allowing users to compare:
- Path quality (smoothness, directness)
- Different colored trajectories for each planner
- Animated playback of each trajectory

Usage:
    python demos/07_multi_planner_visualization.py
    python demos/07_multi_planner_visualization.py --planners RRT RRT-Connect "RRT*"
    python demos/07_multi_planner_visualization.py --obstacles
    python demos/07_multi_planner_visualization.py --animate
    python demos/07_multi_planner_visualization.py --environment  # Add floor and walls
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

# Colors for each planner (RGB 0-1)
PLANNER_COLORS = {
    "RRT": (0.12, 0.47, 0.71),        # blue
    "RRT-Connect": (1.00, 0.50, 0.05), # orange
    "RRT*": (0.17, 0.63, 0.17),        # green
    "PRM": (0.84, 0.15, 0.16),         # red
    "KPIECE1": (0.58, 0.40, 0.74),     # purple
    "EST": (0.55, 0.34, 0.29),         # brown
    "BiTRRT": (0.89, 0.47, 0.76),      # pink
}


def create_environment_boundaries(
    floor_height: float = 0.0,
    wall_distance: float = 0.8,
    wall_height: float = 1.0,
    include_ceiling: bool = False,
) -> List[Tuple[Box, Tuple[float, float, float, float]]]:
    """
    Create floor and wall obstacles to constrain robot motion.

    Args:
        floor_height: Z position of floor surface (default: 0.0)
        wall_distance: Distance from robot base to walls (default: 0.8m)
        wall_height: Height of walls (default: 1.0m)
        include_ceiling: If True, add a ceiling

    Returns:
        List of (Box, color) tuples for collision checking and visualization
    """
    obstacles = []
    wall_thickness = 0.02

    # Floor - thin box below ground level
    floor = Box(
        center=(0.0, 0.0, floor_height - wall_thickness / 2),
        size=(wall_distance * 2.5, wall_distance * 2.5, wall_thickness)
    )
    floor_color = (0.6, 0.6, 0.6, 0.8)  # Gray
    obstacles.append((floor, floor_color))

    # Back wall (behind robot, -X direction)
    back_wall = Box(
        center=(-wall_distance, 0.0, wall_height / 2),
        size=(wall_thickness, wall_distance * 2, wall_height)
    )
    back_wall_color = (0.7, 0.8, 0.9, 0.5)  # Light blue, semi-transparent
    obstacles.append((back_wall, back_wall_color))

    # Left wall (-Y direction)
    left_wall = Box(
        center=(0.0, -wall_distance, wall_height / 2),
        size=(wall_distance * 2, wall_thickness, wall_height)
    )
    left_wall_color = (0.9, 0.8, 0.7, 0.5)  # Light orange, semi-transparent
    obstacles.append((left_wall, left_wall_color))

    # Right wall (+Y direction)
    right_wall = Box(
        center=(0.0, wall_distance, wall_height / 2),
        size=(wall_distance * 2, wall_thickness, wall_height)
    )
    right_wall_color = (0.8, 0.9, 0.7, 0.5)  # Light green, semi-transparent
    obstacles.append((right_wall, right_wall_color))

    # Optional ceiling
    if include_ceiling:
        ceiling = Box(
            center=(0.0, 0.0, wall_height + wall_thickness / 2),
            size=(wall_distance * 2, wall_distance * 2, wall_thickness)
        )
        ceiling_color = (0.8, 0.8, 0.8, 0.3)  # Light gray, very transparent
        obstacles.append((ceiling, ceiling_color))

    return obstacles


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
    """Run a single planner and return results."""
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
    """Run multiple planners on the same problem."""
    if planners is None:
        planners = list(PLANNERS.keys())

    results = {}
    for name in planners:
        if name not in PLANNERS:
            print(f"Unknown planner: {name}")
            continue

        print(f"  {name}...", end=" ", flush=True)
        result = run_planner(name, state_space, q_start, q_goal, timeout)

        status = "OK" if result["success"] else "FAILED"
        if result["success"]:
            print(f"{status} (plan: {result['planning_time']:.3f}s, "
                  f"exec: {result['duration']:.2f}s)")
        else:
            print(f"{status} ({result['planning_time']:.3f}s)")

        results[name] = result

    return results


def print_comparison_table(results: Dict[str, Dict]):
    """Print a comparison table of planner metrics."""
    print("\n" + "=" * 70)
    print("PLANNER COMPARISON")
    print("=" * 70)

    headers = ["Planner", "Plan Time", "Path Len", "Waypoints", "Exec Time"]
    widths = [12, 12, 10, 10, 10]

    header_row = " | ".join(h.center(w) for h, w in zip(headers, widths))
    print(header_row)
    print("-" * len(header_row))

    for name, r in results.items():
        if r["success"]:
            row = [
                name[:widths[0]].ljust(widths[0]),
                f"{r['planning_time']:.4f}s".rjust(widths[1]),
                f"{r['path_length']:.2f}".rjust(widths[2]),
                str(r['num_waypoints']).center(widths[3]),
                f"{r['duration']:.2f}s".rjust(widths[4]),
            ]
        else:
            row = [
                name[:widths[0]].ljust(widths[0]),
                f"{r['planning_time']:.4f}s".rjust(widths[1]),
                "FAILED".center(widths[2]),
                "-".center(widths[3]),
                "-".center(widths[4]),
            ]
        print(" | ".join(row))

    print("=" * 70)


def print_legend(results: Dict[str, Dict]):
    """Print color legend for the planners."""
    print("\nTrajectory Colors:")
    for name in results.keys():
        if results[name]["success"]:
            color = PLANNER_COLORS.get(name, (0.5, 0.5, 0.5))
            # Convert to approximate terminal color name
            r, g, b = [int(c * 255) for c in color]
            print(f"  {name}: RGB({r}, {g}, {b})")


def main():
    parser = argparse.ArgumentParser(
        description="Compare motion planners with PyBullet 3D visualization"
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["RRT", "RRT-Connect", "RRT*"],
        choices=list(PLANNERS.keys()),
        help="Planners to compare (default: RRT, RRT-Connect, RRT*)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Planning timeout per planner in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--obstacles",
        action="store_true",
        help="Add obstacles for more interesting planning scenarios",
    )
    parser.add_argument(
        "--environment",
        action="store_true",
        help="Add floor and walls to constrain robot motion",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Animate robot through each trajectory after showing paths",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.5,
        help="Animation speed multiplier (default: 1.5)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("MULTI-PLANNER TRAJECTORY VISUALIZATION")
    print("=" * 60)

    # Setup robot and state space
    robot = UR5RobotModel()
    obstacles = []  # List of (Box, color) tuples
    collision_manager = ShapeCollisionManager(robot)

    # Add environment boundaries (floor and walls) if requested
    if args.environment:
        print("\nSetting up environment with floor and walls...")
        env_obstacles = create_environment_boundaries(
            floor_height=0.0,
            wall_distance=0.7,
            wall_height=0.9,
            include_ceiling=False,
        )
        for box, color in env_obstacles:
            collision_manager.add_shape(box)
            obstacles.append((box, color))
        print(f"  Added floor and {len(env_obstacles) - 1} walls")

    # Add additional box obstacle if requested
    if args.obstacles:
        print("\nAdding box obstacle...")
        obstacle = Box(center=(0.3, 0.0, 0.35), size=(0.12, 0.25, 0.12))
        collision_manager.add_shape(obstacle)
        obstacle_color = (0.8, 0.2, 0.2, 0.7)  # Red
        obstacles.append((obstacle, obstacle_color))
        print(f"  Added box obstacle at {obstacle.center}")

    # Set collision manager on robot
    if obstacles:
        robot.set_collision_manager(collision_manager)
    else:
        # No obstacles - use null collision manager
        null_manager = NullCollisionManager()
        robot.in_collision = null_manager.in_collision

    state_space = JointStateSpace(robot)

    # Define start and goal configurations
    # Use different configs when environment (walls) is enabled to stay within bounds
    if args.environment:
        # Configurations that stay within wall boundaries (0.7m from base)
        # These keep the arm more upright to avoid colliding with walls
        q_start = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])
        q_goal = np.array([0.9, -1.1, 0.9, -0.2, 0.1, 0.0])
    else:
        q_start = np.array([0.0, -0.5, 0.5, -0.5, -0.5, 0.0])
        q_goal = np.array([1.5, -1.0, 1.0, -1.0, 0.5, 0.5])

    print(f"\nStart config: {np.round(q_start, 2)}")
    print(f"Goal config:  {np.round(q_goal, 2)}")

    # Run planners
    print("\nRunning planners...")
    results = run_all_planners(
        state_space,
        q_start,
        q_goal,
        planners=args.planners,
        timeout=args.timeout,
    )

    # Print comparison
    print_comparison_table(results)

    # Filter successful results
    successful_results = {
        name: result
        for name, result in results.items()
        if result["success"] and result.get("trajectory") is not None
    }

    if not successful_results:
        print("\nNo successful plans to visualize!")
        return

    # PyBullet visualization
    print("\nStarting PyBullet visualization...")
    print_legend(successful_results)

    try:
        from visualization.pybullet_visualizer import PyBulletVisualizer
    except ImportError:
        print("Error: PyBullet not available. Install with: pip install pybullet")
        return

    print("\nControls:")
    print("  - Drag to rotate view")
    print("  - Scroll to zoom")
    print("  - Press Enter in terminal to proceed")

    with PyBulletVisualizer(gui=True, camera_distance=1.8, camera_yaw=60, camera_pitch=-25) as viz:
        # Add obstacles (floor, walls, boxes) if present
        for obs, color in obstacles:
            if hasattr(obs, 'center') and hasattr(obs, 'size'):
                viz.add_box(obs.center, obs.size, color=color)

        # Show start configuration and get EE positions using PyBullet's FK
        viz.visualize_configuration(q_start, duration=0.5)
        start_ee_pos, _ = viz.get_end_effector_pose()

        viz.visualize_configuration(q_goal, duration=0.1)
        goal_ee_pos, _ = viz.get_end_effector_pose()

        # Return to start for visualization
        viz.visualize_configuration(q_start, duration=0.3)

        # Add start and goal markers using PyBullet's EE positions
        viz.add_marker(start_ee_pos, color=(0, 1, 0), size=0.04)  # Green
        viz.add_marker(goal_ee_pos, color=(1, 0, 0), size=0.04)   # Red

        # Draw all trajectories with different colors
        # Use PyBullet's internal FK (fk_func=None) for consistent visualization
        print("\nDrawing all planner trajectories...")
        viz.visualize_multi_planner_trajectories(
            successful_results,
            fk_func=None,  # Use PyBullet's FK for consistency
            show_labels=True,
            line_width=4,
            sample_every=2,
        )

        input("\nPress Enter to continue...")

        # Optionally animate through each trajectory
        if args.animate:
            print("\nAnimating trajectories...")
            for name, result in successful_results.items():
                print(f"  Playing: {name}")

                # Clear previous trails
                viz.clear_path()

                # Re-draw all paths (so user can see comparison)
                viz.visualize_multi_planner_trajectories(
                    successful_results,
                    fk_func=None,  # Use PyBullet's FK for consistency
                    show_labels=False,
                    line_width=2,
                    sample_every=3,
                )

                # Animate this trajectory
                viz.visualize_trajectory(
                    result["trajectory"],
                    result["timestamps"],
                    real_time=True,
                    speed=args.speed,
                    show_ee_trail=True,
                    trail_length=80,
                )

                time.sleep(0.5)

        print("\nVisualization complete!")
        input("Press Enter to close...")


if __name__ == "__main__":
    main()
