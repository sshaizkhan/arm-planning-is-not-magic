#!/usr/bin/env python3
"""
Demo 10: Multi-Planner Trajectory Visualization in Meshcat.

Runs multiple motion planners on the same problem and overlays all trajectories
in a browser tab via meshcat — no display server required. Use this to compare:
- Path quality (smoothness, directness)
- Planning time per algorithm
- Trajectory duration after TOPP-RA parameterization

Each planner's EE path is drawn in a distinct color with a label.

Run inside Docker:
    docker run --rm -p 7000:7000 arm-planning-is-not-magic:ci \\
        python3 demos/10_meshcat_multi_planner.py

Or locally (if meshcat installed):
    python3 demos/10_meshcat_multi_planner.py
    python3 demos/10_meshcat_multi_planner.py --planners RRT RRT-Connect "RRT*"
    python3 demos/10_meshcat_multi_planner.py --execute
    python3 demos/10_meshcat_multi_planner.py --execute --speed 0.75

Then open http://localhost:7000/static/ in your browser.
"""

import argparse
import time

import numpy as np

from core.collision_manager import Box, Sphere, ShapeCollisionManager
from core.kinematics.urdf_kinematics import URDFKinematics
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

try:
    from visualization.meshcat_visualizer import MeshcatVisualizer
except ImportError:
    print("ERROR: pip install meshcat yourdfpy")
    exit(1)


PLANNERS = {
    "RRT":         (OMPLRRTPlanner,        {}),
    "RRT-Connect": (OMPLRRTConnectPlanner,  {}),
    "RRT*":        (OMPLRRTStarPlanner,     {}),
    "PRM":         (OMPLPRMPlanner,         {}),
    "KPIECE1":     (OMPLKPIECE1Planner,     {}),
    "EST":         (OMPLESTPlanner,         {}),
    "BiTRRT":      (OMPLBiTRRTPlanner,      {}),
}


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

    path_len = sum(np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1))
    return {"success": True, "path": path, "trajectory": trajectory, "timestamps": timestamps,
            "planning_time": planning_time, "path_length": path_len,
            "duration": duration, "num_waypoints": len(path)}


def run_all_planners(state_space, q_start, q_goal, planners, timeout=2.0):
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
    parser = argparse.ArgumentParser(description="Compare planners in Meshcat browser")
    parser.add_argument("--planners", nargs="+", default=["RRT", "RRT-Connect", "RRT*"],
                        choices=list(PLANNERS.keys()))
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true",
                        help="Execute (animate) each planner's trajectory sequentially after paths are shown")
    parser.add_argument("--speed", type=float, default=1.5,
                        help="Trajectory playback speed multiplier (default: 1.5)")
    args = parser.parse_args()

    print("=" * 60)
    print("MULTI-PLANNER MESHCAT VISUALIZATION")
    print("=" * 60)

    # Same obstacle setup as demo 09 — verified collision-free at start and goal
    robot_bare = UR5RobotModel(use_opw=False, collision_manager=None)
    col_mgr = ShapeCollisionManager(robot_bare)

    FLOOR    = Box(np.array([0.0, 0.0, -1.57]),    np.array([6.0, 6.0, 3.0]))
    OBS_SLAB = Box(np.array([0.22, 0.22, 0.4]),    np.array([0.4, 0.06, 0.3]))
    OBS_PIL  = Box(np.array([0.15, 0.38, 0.38]),   np.array([0.06, 0.08, 0.55]))
    OBS_SPH  = Sphere(np.array([0.3, 0.3, 0.42]),  0.07)

    for shape in [FLOOR, OBS_SLAB, OBS_PIL, OBS_SPH]:
        col_mgr.add_shape(shape)

    robot = UR5RobotModel(use_opw=False, collision_manager=col_mgr)
    fk = URDFKinematics()
    state_space = JointStateSpace(robot)

    # Start/goal verified collision-free with above obstacles
    q_start = np.array([0.0,        -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    q_goal  = np.array([np.pi / 2,  -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])

    assert not robot.in_collision(q_start), "BUG: start in collision"
    assert not robot.in_collision(q_goal),  "BUG: goal in collision"

    print(f"\nStart: {np.round(q_start, 2)}")
    print(f"Goal:  {np.round(q_goal, 2)}")

    # Start meshcat
    viz = MeshcatVisualizer(open_browser=False)
    print(f"\n==> Open in browser: http://localhost:7000/static/")
    print("Waiting 8s before planning...")
    time.sleep(8)

    # Add obstacles to meshcat scene (same as demo 09)
    viz.add_box(center=(0.22, 0.22, 0.4),  size=(0.4, 0.06, 0.3),  color=(0.9, 0.2, 0.2, 0.75))
    viz.add_box(center=(0.15, 0.38, 0.38), size=(0.06, 0.08, 0.55), color=(0.9, 0.6, 0.1, 0.75))
    viz.add_sphere(center=(0.3, 0.3, 0.42), radius=0.07,            color=(0.2, 0.7, 0.9, 0.75))

    # Mark start and goal EE
    start_ee = fk.forward_kinematics(q_start)[:3, 3]
    goal_ee = fk.forward_kinematics(q_goal)[:3, 3]
    viz.add_marker(tuple(start_ee), color=(0.0, 1.0, 0.5), size=0.04)
    viz.add_marker(tuple(goal_ee), color=(1.0, 1.0, 0.0), size=0.04)
    viz.visualize_configuration(q_start)

    # Run planners
    print(f"\nRunning {args.planners} planners (timeout={args.timeout}s each)...")
    results = run_all_planners(state_space, q_start, q_goal,
                               args.planners, args.timeout)
    print_comparison_table(results)

    successful = {n: r for n, r in results.items()
                  if r["success"] and r.get("trajectory") is not None}

    if not successful:
        print("\nNo successful plans to visualize!")
        time.sleep(60)
        return

    # Draw all EE paths simultaneously with labels
    print(f"\nDrawing {len(successful)} trajectories in browser...")
    viz.visualize_multi_planner_trajectories(
        successful,
        fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None),
        show_labels=True,
        line_width=4,
        sample_every=2,
    )

    if not args.execute:
        print("\nAll paths shown. Add --execute to animate each trajectory. Ctrl+C to stop.")
        while True:
            time.sleep(5)

    # Execute each planner's trajectory sequentially
    print(f"\nExecuting {len(successful)} trajectories at {args.speed}x speed...")
    for name, result in successful.items():
        print(f"  Executing: {name}")
        viz.clear_path()
        # Redraw all paths faintly, then animate this one
        viz.visualize_multi_planner_trajectories(
            successful,
            fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None),
            show_labels=False,
            line_width=2,
            sample_every=3,
        )
        viz.visualize_trajectory(
            result["trajectory"],
            time_stamps=result["timestamps"],
            real_time=True,
            speed=args.speed,
            show_ee_trail=True,
            trail_length=80,
        )
        time.sleep(0.5)

    print("\nVisualization complete. Ctrl+C to stop.")
    while True:
        time.sleep(5)


if __name__ == "__main__":
    main()
