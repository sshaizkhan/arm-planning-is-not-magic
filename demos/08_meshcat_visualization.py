#!/usr/bin/env python3
"""
Demo 08: Meshcat Browser-Based Visualization with Obstacle Avoidance.

This demo mirrors demo 06 (PyBullet) but uses meshcat for visualization —
no display server required, just open the printed URL in any browser.

Features demonstrated:
- Meshcat browser-based 3D visualization
- UR5 model loaded from URDF via yourdfpy
- IK-based goal pose specification
- Obstacle avoidance planning
- Path visualization before trajectory execution
- Trajectory playback with EE trail

Run with: python3 demos/08_meshcat_visualization.py
Then open the URL printed to the terminal in your browser.
"""


import numpy as np

from core.kinematics.urdf_kinematics import URDFKinematics
from core.path_smoothing import smooth_path
from core.pose_utils import IKSolver
from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner

try:
    from visualization.meshcat_visualizer import MeshcatVisualizer
except ImportError:
    print("ERROR: meshcat/yourdfpy not installed.")
    print("Install with: pip install meshcat yourdfpy")
    exit(1)


UR5_VELOCITY_LIMITS = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])
UR5_ACCEL_LIMITS = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

SAFE_START_CONFIGS = [
    np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
    np.array([0.0, -np.pi / 2, np.pi / 3, -np.pi / 3, -np.pi / 2, 0.0]),
]


def main():
    print("=== Demo 08: Meshcat Visualization ===\n")

    robot = UR5RobotModel(use_opw=False, collision_manager=None)
    fk = URDFKinematics()
    ik_solver = IKSolver(robot)
    state_space = JointStateSpace(robot)

    viz = MeshcatVisualizer(open_browser=True)
    print("\nOpen the URL above in your browser, then press Enter to continue.")
    input()

    print("Adding obstacles...")
    viz.add_box(
        center=(0.4, 0.0, 0.4),
        size=(0.15, 0.15, 0.4),
        color=(1.0, 0.2, 0.2, 0.7),
    )
    viz.add_sphere(
        center=(0.0, 0.5, 0.3),
        radius=0.1,
        color=(0.2, 1.0, 0.2, 0.7),
    )

    goal_pos = np.array([0.4, 0.3, 0.5])
    goal_rot = np.eye(3)

    print("Solving IK for goal...")
    q_start = SAFE_START_CONFIGS[0]
    T_goal = np.eye(4)
    T_goal[:3, :3] = goal_rot
    T_goal[:3, 3] = goal_pos
    q_goals = ik_solver.solve(T_goal)
    if not q_goals:
        print("IK failed — using fallback goal config")
        q_goal = SAFE_START_CONFIGS[1]
    else:
        q_goal = q_goals[0]

    print(f"Start config: {np.round(q_start, 3)}")
    print(f"Goal config:  {np.round(q_goal, 3)}")

    viz.visualize_configuration(q_start)
    print("\nShowing start config. Press Enter to plan...")
    input()

    print("Planning with RRT-Connect...")
    planner = OMPLRRTConnectPlanner(state_space)
    result = planner.plan(q_start, q_goal, timeout=10.0)

    if result["path"] is None:
        print("Planning failed.")
        return

    path = result["path"]
    path = smooth_path(path, state_space, iterations=50)
    print(f"Path found: {len(path)} waypoints")

    viz.visualize_path(path, fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None))
    print("Planned path shown. Press Enter to parameterize and execute...")
    input()

    parameterizer = ToppraTimeParameterizer(
        velocity_limits=UR5_VELOCITY_LIMITS,
        acceleration_limits=UR5_ACCEL_LIMITS,
    )
    traj_result = parameterizer.parameterize(path, q_start, q_goal)
    trajectory = traj_result["trajectory"]
    time_stamps = traj_result["time_stamps"]
    print(f"Trajectory: {len(trajectory)} points over {time_stamps[-1]:.2f}s")

    print("Executing trajectory in browser...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
    )

    print("\nDone. Browser window remains open. Press Enter to exit.")
    input()


if __name__ == "__main__":
    main()
