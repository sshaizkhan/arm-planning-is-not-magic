#!/usr/bin/env python3
"""
Demo 07: Multi-Planner Trajectory Visualization in PyBullet.

Runs multiple motion planners on the same problem and visualizes their
trajectories side-by-side in PyBullet 3D. Use this to compare:
- Path quality (smoothness, directness)
- Planning time per algorithm
- Trajectory duration after TOPP-RA parameterization

Usage:
    python demos/07_multi_planner_visualization.py
    python demos/07_multi_planner_visualization.py --planners RRT RRT-Connect "RRT*"
    python demos/07_multi_planner_visualization.py --obstacles
    python demos/07_multi_planner_visualization.py --animate
    python demos/07_multi_planner_visualization.py --environment
    python demos/07_multi_planner_visualization.py --random
"""

import argparse
import time
from typing import Dict, List, Optional

import numpy as np

from core.collision_manager import Box, NullCollisionManager, ShapeCollisionManager
from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from planners import (
    OMPLBiTRRTPlanner,
    OMPLESTPlanner,
    OMPLKPIECE1Planner,
    OMPLPRMPlanner,
    OMPLRRTConnectPlanner,
    OMPLRRTPlanner,
    OMPLRRTStarPlanner,
)

PLANNERS = {
    "RRT": (OMPLRRTPlanner, {"step_size": 0.1}),
    "RRT-Connect": (OMPLRRTConnectPlanner, {"step_size": 0.1}),
    "RRT*": (OMPLRRTStarPlanner, {"step_size": 0.1}),
    "PRM": (OMPLPRMPlanner, {}),
    "KPIECE1": (OMPLKPIECE1Planner, {}),
    "EST": (OMPLESTPlanner, {"step_size": 0.1}),
    "BiTRRT": (OMPLBiTRRTPlanner, {"step_size": 0.1}),
}

PLANNER_COLORS = {
    "RRT": (0.12, 0.47, 0.71),
    "RRT-Connect": (1.00, 0.50, 0.05),
    "RRT*": (0.17, 0.63, 0.17),
    "PRM": (0.84, 0.15, 0.16),
    "KPIECE1": (0.58, 0.40, 0.74),
    "EST": (0.55, 0.34, 0.29),
    "BiTRRT": (0.89, 0.47, 0.76),
}


def create_environment_boundaries(floor_height=0.0, wall_distance=0.8, wall_height=1.0):
    """Create floor and wall Box obstacles. Returns list of (Box, color) tuples."""
    t = 0.02
    obstacles = [
        (Box((0.0, 0.0, floor_height - t / 2), (wall_distance * 2.5, wall_distance * 2.5, t)),
         (0.6, 0.6, 0.6, 0.8)),
        (Box((-wall_distance, 0.0, wall_height / 2), (t, wall_distance * 2, wall_height)),
         (0.7, 0.8, 0.9, 0.5)),
        (Box((0.0, -wall_distance, wall_height / 2), (wall_distance * 2, t, wall_height)),
         (0.9, 0.8, 0.7, 0.5)),
        (Box((0.0, wall_distance, wall_height / 2), (wall_distance * 2, t, wall_height)),
         (0.8, 0.9, 0.7, 0.5)),
    ]
    return obstacles


def compute_path_length(path):
    if path is None or len(path) < 2:
        return 0.0
    return sum(np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1))


def run_planner(name, state_space, q_start, q_goal, timeout=2.0):
    planner_class, kwargs = PLANNERS[name]
    try:
        planner = planner_class(state_space, **kwargs)
    except TypeError:
        planner = planner_class(state_space)

    t0 = time.time()
    path = planner.plan(q_start, q_goal, timeout=timeout)
    planning_time = time.time() - t0

    if path is None:
        return {"success": False, "path": None, "trajectory": None, "timestamps": None,
                "planning_time": planning_time, "path_length": 0.0, "duration": 0.0,
                "num_waypoints": 0}

    v_max = np.ones(state_space.dim) * 1.0
    a_max = np.ones(state_space.dim) * 2.0
    try:
        timestamps, trajectory = ToppraTimeParameterizer(v_max, a_max).compute(path)
        duration = float(timestamps[-1])
    except Exception as e:
        print(f"  Warning: parameterization failed for {name}: {e}")
        timestamps = trajectory = None
        duration = 0.0

    return {"success": True, "path": path, "trajectory": trajectory, "timestamps": timestamps,
            "planning_time": planning_time, "path_length": compute_path_length(path),
            "duration": duration, "num_waypoints": len(path)}


def run_all_planners(state_space, q_start, q_goal, planners=None, timeout=2.0):
    if planners is None:
        planners = list(PLANNERS.keys())
    results = {}
    for name in planners:
        if name not in PLANNERS:
            print(f"  Unknown planner: {name}")
            continue
        print(f"  {name}...", end=" ", flush=True)
        r = run_planner(name, state_space, q_start, q_goal, timeout)
        if r["success"]:
            print(f"OK (plan: {r['planning_time']:.3f}s, exec: {r['duration']:.2f}s)")
        else:
            print(f"FAILED ({r['planning_time']:.3f}s)")
        results[name] = r
    return results


def print_comparison_table(results):
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
            row = [name[:widths[0]].ljust(widths[0]),
                   f"{r['planning_time']:.4f}s".rjust(widths[1]),
                   f"{r['path_length']:.2f}".rjust(widths[2]),
                   str(r['num_waypoints']).center(widths[3]),
                   f"{r['duration']:.2f}s".rjust(widths[4])]
        else:
            row = [name[:widths[0]].ljust(widths[0]),
                   f"{r['planning_time']:.4f}s".rjust(widths[1]),
                   "FAILED".center(widths[2]), "-".center(widths[3]), "-".center(widths[4])]
        print(" | ".join(row))
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Compare planners in PyBullet 3D")
    parser.add_argument("--planners", nargs="+", default=["RRT", "RRT-Connect", "RRT*"],
                        choices=list(PLANNERS.keys()))
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--obstacles", action="store_true", help="Add a box obstacle")
    parser.add_argument("--environment", action="store_true", help="Add floor and walls")
    parser.add_argument("--animate", action="store_true", help="Animate each trajectory")
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--random", action="store_true", help="Use random valid goal")
    args = parser.parse_args()

    print("=" * 60)
    print("MULTI-PLANNER TRAJECTORY VISUALIZATION")
    print("=" * 60)

    robot = UR5RobotModel()
    obstacles = []
    collision_manager = ShapeCollisionManager(robot)

    if args.environment:
        print("\nSetting up floor and walls...")
        for box, color in create_environment_boundaries(wall_distance=0.7, wall_height=0.9):
            collision_manager.add_shape(box)
            obstacles.append((box, color))
        print(f"  Added {len(obstacles)} environment shapes")

    if args.obstacles:
        center = (-0.35, -0.15, 0.65) if args.environment else (0.3, 0.0, 0.35)
        obs = Box(center=center, size=(0.12, 0.20, 0.15))
        collision_manager.add_shape(obs)
        obstacles.append((obs, (0.8, 0.2, 0.2, 0.7)))
        print(f"  Added box obstacle at {obs.center}")

    if obstacles:
        robot.set_collision_manager(collision_manager)
    else:
        robot.set_collision_manager(NullCollisionManager())

    state_space = JointStateSpace(robot)

    if args.environment:
        q_start = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])
        q_goal = np.array([0.9, -1.1, 0.9, -0.2, 0.1, 0.0])
    else:
        q_start = np.array([0.0, -0.5, 0.5, -0.5, -0.5, 0.0])
        q_goal = np.array([1.5, -1.0, 1.0, -1.0, 0.5, 0.5])

    if args.random:
        print("\nSearching for random valid goal...")
        lower, upper = robot.joint_limits()
        for attempt in range(100):
            q_rand = lower + np.random.random(6) * (upper - lower)
            if state_space.is_valid(q_rand) and np.linalg.norm(q_rand - q_start) > 1.0:
                q_goal = q_rand
                print(f"  Found valid goal after {attempt + 1} attempts")
                break
        else:
            print("  Warning: could not find random goal, using default")

    print(f"\nStart: {np.round(q_start, 2)}")
    print(f"Goal:  {np.round(q_goal, 2)}")

    print("\nRunning planners...")
    results = run_all_planners(state_space, q_start, q_goal,
                               planners=args.planners, timeout=args.timeout)
    print_comparison_table(results)

    successful = {n: r for n, r in results.items()
                  if r["success"] and r.get("trajectory") is not None}

    if not successful:
        print("\nNo successful plans to visualize!")
        return

    try:
        from visualization.pybullet_visualizer import PyBulletVisualizer
    except ImportError:
        print("PyBullet not installed. Install with: pip install pybullet")
        return

    print("\nStarting PyBullet... (drag to rotate, scroll to zoom)")

    with PyBulletVisualizer(gui=True, camera_distance=1.8, camera_yaw=60,
                            camera_pitch=-25) as viz:
        for obs, color in obstacles:
            viz.add_box(obs.center, obs.size, color=color)

        viz.visualize_configuration(q_start, duration=0.5)
        start_ee, _ = viz.get_end_effector_pose()

        viz.visualize_configuration(q_goal, duration=0.1)
        goal_ee, _ = viz.get_end_effector_pose()

        viz.visualize_configuration(q_start, duration=0.3)

        viz.add_marker(start_ee, color=(0, 1, 0), size=0.04)
        viz.add_marker(goal_ee, color=(1, 0, 0), size=0.04)

        print("\nDrawing all trajectories...")
        viz.visualize_multi_planner_trajectories(
            successful, fk_func=None, show_labels=True, line_width=4, sample_every=2,
        )

        input("\nPress Enter to continue...")

        if args.animate:
            print("\nAnimating trajectories...")
            for name, result in successful.items():
                print(f"  Playing: {name}")
                viz.clear_path()
                viz.visualize_multi_planner_trajectories(
                    successful, fk_func=None, show_labels=False, line_width=2, sample_every=3,
                )
                viz.visualize_trajectory(
                    result["trajectory"], result["timestamps"],
                    real_time=True, speed=args.speed, show_ee_trail=True, trail_length=80,
                )
                time.sleep(0.5)

        print("\nVisualization complete!")
        input("Press Enter to close...")


if __name__ == "__main__":
    main()
