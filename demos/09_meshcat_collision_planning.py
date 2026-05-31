#!/usr/bin/env python3
"""
Demo 09: Meshcat Collision-Aware Planning.

Plans a collision-free trajectory around real obstacles using OMPL RRT-Connect.
Obstacles are wired into both the collision manager (so the planner avoids them)
and into meshcat (so you can see them in the browser).

The arm sweeps 90° (base rotation) and must navigate around three obstacles
that block the direct joint-space path.

Run inside Docker:
    docker run --rm -p 7000:7000 arm-planning-is-not-magic:ci \\
        python3 demos/09_meshcat_collision_planning.py

Then open http://localhost:7000/static/ in your browser.
"""

import time

import numpy as np

from core.collision_manager import Box, Sphere, ShapeCollisionManager
from core.kinematics.urdf_kinematics import URDFKinematics
from core.path_smoothing import smooth_path
from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner

try:
    from visualization.meshcat_visualizer import MeshcatVisualizer
except ImportError:
    print("ERROR: pip install meshcat yourdfpy")
    exit(1)


UR5_VEL_LIMITS = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
UR5_ACC_LIMITS = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5])

# Arm pointing forward (+X)
Q_START = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
# Arm pointing left (+Y), 90° base rotation — must navigate around obstacles
Q_GOAL = np.array([np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])

# Floor: top surface at z=-0.07 (just below base capsule radius 0.065), extends 3m deep.
# Prevents arm links from swinging below the ground plane during planning.
FLOOR = Box(np.array([0.0, 0.0, -1.57]), np.array([6.0, 6.0, 3.0]))

# Obstacles along the diagonal swept by the arm from start→goal
# Verified collision-free at both Q_START and Q_GOAL
OBSTACLES = [
    {
        "label": "diagonal slab",
        "meshcat": dict(center=(0.22, 0.22, 0.4), size=(0.4, 0.06, 0.3), color=(0.9, 0.2, 0.2, 0.75)),
        "collision": Box(np.array([0.22, 0.22, 0.4]), np.array([0.4, 0.06, 0.3])),
    },
    {
        "label": "side pillar",
        "meshcat": dict(center=(0.15, 0.38, 0.38), size=(0.06, 0.08, 0.55), color=(0.9, 0.6, 0.1, 0.75)),
        "collision": Box(np.array([0.15, 0.38, 0.38]), np.array([0.06, 0.08, 0.55])),
    },
    {
        "label": "upper sphere",
        "meshcat": dict(center=(0.3, 0.3, 0.42), radius=0.07, color=(0.2, 0.7, 0.9, 0.75)),
        "collision": Sphere(np.array([0.3, 0.3, 0.42]), 0.07),
    },
]


def main():
    print("=== Demo 09: Meshcat Collision-Aware Planning ===\n")

    # --- Build collision manager (floor + obstacles) ---
    robot_bare = UR5RobotModel(use_opw=False, collision_manager=None)
    col_mgr = ShapeCollisionManager(robot_bare)
    col_mgr.add_shape(FLOOR)  # prevent path from going underground
    for obs in OBSTACLES:
        col_mgr.add_shape(obs["collision"])

    robot = UR5RobotModel(use_opw=False, collision_manager=col_mgr)
    fk = URDFKinematics()
    state_space = JointStateSpace(robot)

    assert not robot.in_collision(Q_START), "BUG: start in collision"
    assert not robot.in_collision(Q_GOAL), "BUG: goal in collision"

    # --- Start meshcat ---
    viz = MeshcatVisualizer(open_browser=False)
    print()
    print("==> Open in browser: http://localhost:7000/static/")
    print("    Waiting 6s before continuing...")
    time.sleep(6)

    # --- Add obstacles to scene ---
    print("Adding obstacles to scene...")
    for obs in OBSTACLES:
        kw = obs["meshcat"]
        if "radius" in kw:
            viz.add_sphere(**kw)
        else:
            viz.add_box(**kw)
        print(f"  [{obs['label']}]")

    # --- Show start ---
    start_ee = fk.forward_kinematics(Q_START)[:3, 3]
    goal_ee = fk.forward_kinematics(Q_GOAL)[:3, 3]
    print(f"\nStart EE: {np.round(start_ee, 3)}")
    print(f"Goal  EE: {np.round(goal_ee, 3)}")
    viz.visualize_configuration(Q_START)
    viz.add_marker(tuple(start_ee), color=(0.0, 1.0, 1.0), size=0.03)
    viz.add_marker(tuple(goal_ee),  color=(1.0, 1.0, 0.0), size=0.03)
    time.sleep(3)

    # --- Plan ---
    print("\nPlanning with RRT-Connect (timeout 20s)...")
    t0 = time.time()
    planner = OMPLRRTConnectPlanner(state_space)
    result = planner.plan(Q_START, Q_GOAL, timeout=20.0)
    print(f"Planning took {time.time() - t0:.1f}s")

    if result is None:
        print("Planning failed.")
        time.sleep(60)
        return

    path = result
    print(f"Raw path: {len(path)} waypoints")
    path = smooth_path(path, collision_check=robot.in_collision, shortcut_iterations=150)
    print(f"Smoothed: {len(path)} waypoints")

    # --- Show planned EE path ---
    viz.visualize_path(
        path,
        fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None),
        color=(1.0, 0.7, 0.0),
        show_waypoints=True,
        waypoint_color=(1.0, 0.4, 0.0),
        waypoint_size=0.012,
    )
    print("Path shown in orange. Waiting 4s before executing...")
    time.sleep(4)

    # --- Time parameterization ---
    print("Parameterizing...")
    param = ToppraTimeParameterizer(v_max=UR5_VEL_LIMITS, a_max=UR5_ACC_LIMITS)
    time_stamps, trajectory = param.compute(path)
    print(f"Trajectory: {len(trajectory)} pts, {time_stamps[-1]:.2f}s")

    # --- Execute ---
    print("\nExecuting — watch the arm navigate around obstacles...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=0.75,
        show_ee_trail=True,
    )

    print("\nDone. Server alive — Ctrl+C to stop.")
    while True:
        time.sleep(5)


if __name__ == "__main__":
    main()
