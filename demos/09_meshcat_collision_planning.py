#!/usr/bin/env python3
"""
Demo 09: Meshcat Collision-Aware Planning.

Plans a collision-free trajectory around real obstacles using OMPL RRT-Connect.
Obstacles are wired into both the collision manager (so the planner avoids them)
and into meshcat (so you can see them).

Run inside Docker:
    docker run --rm -p 7000:7000 arm-planning-is-not-magic:ci python3 demos/09_meshcat_collision_planning.py

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

# Start: arm upright
Q_START = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
# Goal: arm reaching to the right side (must navigate around obstacles)
Q_GOAL = np.array([-np.pi / 2, -np.pi / 3, np.pi / 3, -np.pi / 2, -np.pi / 2, 0.0])

# Obstacles: (type, meshcat_kwargs, collision_shape)
OBSTACLES = [
    {
        "label": "front wall",
        "meshcat": dict(center=(0.35, 0.0, 0.35), size=(0.08, 0.5, 0.5), color=(0.9, 0.2, 0.2, 0.6)),
        "collision": Box(center=np.array([0.35, 0.0, 0.35]), size=np.array([0.08, 0.5, 0.5])),
    },
    {
        "label": "left sphere",
        "meshcat": dict(center=(-0.1, 0.45, 0.4), radius=0.12, color=(0.2, 0.7, 0.2, 0.6)),
        "collision": Sphere(center=np.array([-0.1, 0.45, 0.4]), radius=0.12),
    },
    {
        "label": "top box",
        "meshcat": dict(center=(0.0, 0.2, 0.65), size=(0.4, 0.15, 0.08), color=(0.2, 0.4, 0.9, 0.6)),
        "collision": Box(center=np.array([0.0, 0.2, 0.65]), size=np.array([0.4, 0.15, 0.08])),
    },
]


def main():
    print("=== Demo 09: Meshcat Collision-Aware Planning ===\n")

    # --- Collision manager with obstacles ---
    robot_no_col = UR5RobotModel(use_opw=False, collision_manager=None)
    col_mgr = ShapeCollisionManager(robot_no_col)
    for obs in OBSTACLES:
        col_mgr.add_shape(obs["collision"])

    robot = UR5RobotModel(use_opw=False, collision_manager=col_mgr)
    fk = URDFKinematics()
    state_space = JointStateSpace(robot)

    # --- Start meshcat ---
    viz = MeshcatVisualizer(open_browser=False)
    print()
    print("==> Open in browser: http://localhost:7000/static/")
    print("    Waiting 10s before continuing...")
    time.sleep(10)

    # --- Add obstacles to meshcat scene ---
    print("Adding obstacles...")
    for obs in OBSTACLES:
        kw = obs["meshcat"]
        if "radius" in kw:
            viz.add_sphere(**kw)
        else:
            viz.add_box(**kw)
        print(f"  [{obs['label']}]")

    # --- Show start config ---
    print(f"\nStart: {np.round(Q_START, 2)}")
    print(f"Goal:  {np.round(Q_GOAL, 2)}")
    viz.visualize_configuration(Q_START)
    time.sleep(2)

    # --- Check start/goal reachability ---
    if robot.in_collision(Q_START):
        print("ERROR: start config is in collision")
        return
    if robot.in_collision(Q_GOAL):
        print("ERROR: goal config is in collision")
        return

    # --- Plan ---
    print("\nPlanning with RRT-Connect (timeout 15s)...")
    planner = OMPLRRTConnectPlanner(state_space)
    result = planner.plan(Q_START, Q_GOAL, timeout=15.0)

    if result["path"] is None:
        print("Planning failed — try adjusting obstacles or goal config.")
        return

    path = result["path"]
    print(f"Raw path: {len(path)} waypoints")

    path = smooth_path(path, state_space, iterations=100)
    print(f"Smoothed: {len(path)} waypoints")

    # --- Show planned EE path ---
    viz.visualize_path(
        path,
        fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None),
        color=(1.0, 0.8, 0.0),
        show_waypoints=True,
        waypoint_color=(1.0, 0.4, 0.0),
    )
    print("Path shown (orange). Waiting 4s...")
    time.sleep(4)

    # --- Time parameterization ---
    print("Parameterizing trajectory...")
    param = ToppraTimeParameterizer(
        velocity_limits=UR5_VEL_LIMITS,
        acceleration_limits=UR5_ACC_LIMITS,
    )
    traj_result = param.parameterize(path, Q_START, Q_GOAL)
    trajectory = traj_result["trajectory"]
    time_stamps = traj_result["time_stamps"]
    print(f"Trajectory: {len(trajectory)} pts over {time_stamps[-1]:.2f}s")

    # --- Execute ---
    print("\nExecuting trajectory (real-time)...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
    )

    print("\nDone. Keeping server alive — Ctrl+C to stop.")
    while True:
        time.sleep(5)


if __name__ == "__main__":
    main()
