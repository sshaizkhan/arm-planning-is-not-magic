#!/usr/bin/env python3
"""
Demo 03: Compare all OMPL planners.

Tests all available planners on the same problem and displays
comparison metrics in an ASCII table.
"""

import argparse
import numpy as np
import time

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager
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
from parameterization.ruckig_parameterization import RuckigTimeParameterizer


def compute_path_length(path):
    """Compute total path length in joint space."""
    if path is None or len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += np.linalg.norm(path[i + 1] - path[i])
    return total


def test_planner(planner_class, state_space, q_start, q_goal, timeout=2.0, parameterizer_type="toppra", **kwargs):
    """Test a single planner and return metrics."""
    try:
        planner = planner_class(state_space, **kwargs)
    except TypeError:
        # Some planners don't take step_size
        planner = planner_class(state_space)

    start_time = time.time()
    path = planner.plan(q_start, q_goal, timeout=timeout)
    planning_time = time.time() - start_time

    if path is None:
        return {
            "success": False,
            "planning_time": planning_time,
            "path_length": 0.0,
            "num_waypoints": 0,
            "trajectory_duration": 0.0,
        }

    path_length = compute_path_length(path)
    num_waypoints = len(path)

    # Time-parameterize the path
    try:
        dof = state_space.dim
        v_max = np.ones(dof) * 1.0
        a_max = np.ones(dof) * 2.0

        if parameterizer_type == "ruckig":
            j_max = np.ones(dof) * 5.0
            param = RuckigTimeParameterizer(v_max, a_max, j_max)
            time_stamps, _ = param.compute(path, dt=0.01)
        else:  # default to toppra
            param = ToppraTimeParameterizer(v_max, a_max)
            time_stamps, _ = param.compute(path)

        trajectory_duration = time_stamps[-1] if len(time_stamps) > 0 else 0.0
    except Exception:
        trajectory_duration = 0.0

    return {
        "success": True,
        "planning_time": planning_time,
        "path_length": path_length,
        "num_waypoints": num_waypoints,
        "trajectory_duration": trajectory_duration,
    }


def print_comparison_table(results):
    """Print comparison results in an ASCII table."""
    # Table header
    print("\n" + "=" * 100)
    print("PLANNER COMPARISON RESULTS")
    print("=" * 100)

    # Column headers
    headers = [
        "Planner",
        "Success",
        "Planning Time (s)",
        "Path Length",
        "Waypoints",
        "Traj Duration (s)",
    ]
    col_widths = [20, 10, 18, 15, 12, 18]

    # Print header
    header_row = " | ".join(h.ljust(w) for h, w in zip(headers, col_widths))
    print(header_row)
    print("-" * len(header_row))

    # Print results
    for planner_name, metrics in results.items():
        success_str = "✓" if metrics["success"] else "✗"
        planning_time_str = f"{metrics['planning_time']:.4f}" if metrics["success"] else "N/A"
        path_length_str = f"{metrics['path_length']:.4f}" if metrics["success"] else "N/A"
        waypoints_str = f"{metrics['num_waypoints']}" if metrics["success"] else "0"
        traj_duration_str = f"{metrics['trajectory_duration']:.4f}" if metrics["success"] else "N/A"

        row = [
            planner_name.ljust(col_widths[0]),
            success_str.center(col_widths[1]),
            planning_time_str.rjust(col_widths[2]),
            path_length_str.rjust(col_widths[3]),
            waypoints_str.rjust(col_widths[4]),
            traj_duration_str.rjust(col_widths[5]),
        ]
        print(" | ".join(row))

    print("=" * 100)

    # Analysis
    successful = [(name, r) for name, r in results.items() if r["success"]]
    if successful:
        print("\n" + "=" * 100)
        print("ANALYSIS")
        print("=" * 100)

        # 1. Top 3 fastest planners
        print("\n1. TOP 3 FASTEST PLANNERS (by planning time):")
        print("-" * 50)
        fastest = sorted(successful, key=lambda x: x[1]["planning_time"])[:3]
        for i, (name, metrics) in enumerate(fastest, 1):
            print(f"   {i}. {name:15s}: {metrics['planning_time']:.4f} s")

        # 2. Path quality analysis
        print("\n2. PATH QUALITY (shorter is better):")
        print("-" * 50)
        path_quality = sorted(successful, key=lambda x: x[1]["path_length"])
        shortest = path_quality[0]
        longest = path_quality[-1]
        median_idx = len(path_quality) // 2
        median = path_quality[median_idx]

        print(f"   Shortest:  {shortest[0]:15s}: {shortest[1]['path_length']:.4f}")
        print(f"   Median:    {median[0]:15s}: {median[1]['path_length']:.4f}")
        print(f"   Longest:   {longest[0]:15s}: {longest[1]['path_length']:.4f}")
        print(f"   Range:     {longest[1]['path_length'] - shortest[1]['path_length']:.4f}")

        # 3. Trajectory duration analysis
        print("\n3. TRAJECTORY DURATION (faster execution is better):")
        print("-" * 50)
        traj_sorted = sorted(successful, key=lambda x: x[1]["trajectory_duration"])
        fastest_traj = traj_sorted[0]
        slowest_traj = traj_sorted[-1]
        median_traj_idx = len(traj_sorted) // 2
        median_traj = traj_sorted[median_traj_idx]

        print(f"   Fastest:   {fastest_traj[0]:15s}: {fastest_traj[1]['trajectory_duration']:.4f} s")
        print(f"   Median:    {median_traj[0]:15s}: {median_traj[1]['trajectory_duration']:.4f} s")
        print(f"   Slowest:   {slowest_traj[0]:15s}: {slowest_traj[1]['trajectory_duration']:.4f} s")
        print(f"   Range:     {slowest_traj[1]['trajectory_duration'] - fastest_traj[1]['trajectory_duration']:.4f} s")


def main():
    """Main comparison function."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Compare OMPL planners")
    parser.add_argument(
        "--parameterizer",
        choices=["toppra", "ruckig"],
        default="toppra",
        help="Time parameterizer to use (default: toppra)",
    )
    args = parser.parse_args()

    # ---------------------------
    # 1. Setup
    # ---------------------------
    print("=" * 100)
    print(f"TIME PARAMETERIZER: {args.parameterizer.upper()}")
    print("=" * 100)
    print("\nSetting up robot and state space...")
    robot = UR5RobotModel()
    collision_manager = NullCollisionManager()
    robot.in_collision = collision_manager.in_collision
    state_space = JointStateSpace(robot)

    q_start = np.array([0.0, 0.927747217, 0.0, 0.642, 0.0, 0.0])
    q_goal = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])

    print(f"Start: {q_start}")
    print(f"Goal:  {q_goal}")
    print("\nTesting all planners with timeout=2.0s...\n")

    # ---------------------------
    # 2. Test all planners
    # ---------------------------
    planners_to_test = [
        ("RRT", OMPLRRTPlanner, {"step_size": 0.1}),
        ("RRT Connect", OMPLRRTConnectPlanner, {"step_size": 0.1}),
        ("RRT*", OMPLRRTStarPlanner, {"step_size": 0.1}),
        ("PRM", OMPLPRMPlanner, {}),
        ("KPIECE1", OMPLKPIECE1Planner, {}),
        ("EST", OMPLESTPlanner, {"step_size": 0.1}),
        ("BiTRRT", OMPLBiTRRTPlanner, {"step_size": 0.1}),
    ]

    results = {}
    for planner_name, planner_class, kwargs in planners_to_test:
        print(f"Testing {planner_name}...", end=" ", flush=True)
        metrics = test_planner(
            planner_class, state_space, q_start, q_goal,
            timeout=2.0, parameterizer_type=args.parameterizer, **kwargs
        )
        results[planner_name] = metrics
        status = "✓" if metrics["success"] else "✗"
        print(f"{status} ({metrics['planning_time']:.3f}s)")

    # ---------------------------
    # 3. Display results
    # ---------------------------
    print_comparison_table(results)


if __name__ == "__main__":
    main()
