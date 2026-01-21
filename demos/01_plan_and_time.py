#!/usr/bin/env python3
"""
Demo 01: Plan and time-parameterize a joint-space path for a 6DOF robot.

This demo shows the full pipeline (no visualization):
1. Define robot model
2. Define state space and collision manager
3. Plan a path using OMPL RRT
4. Time-parameterize path using TOPP-RA
5. Print summary statistics
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager
from planners.ompl_rrt import OMPLRRTPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer

# ---------------------------
# 1. Define robot
# ---------------------------
robot = UR5RobotModel()  # or any 6DOF industrial robot

# ---------------------------
# 2. State space and collision manager
# ---------------------------
collision_manager = NullCollisionManager()
robot.in_collision = collision_manager.in_collision  # delegate
state_space = JointStateSpace(robot)

# ---------------------------
# 3. Plan path using OMPL RRT
# ---------------------------
dof = robot.dof()
q_start = np.array([0.0, 0.927747217, 0.0, 0.642, 0.0, 0.0])
q_goal = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])

planner = OMPLRRTPlanner(state_space, step_size=0.1)
path = planner.plan(q_start, q_goal, timeout=2.0)

if path is None:
    print("No path found!")
    exit(1)

print(f"Planned path with {len(path)} waypoints.")

# ---------------------------
# 4. Time-parameterize using TOPP-RA
# ---------------------------
v_max = np.ones(dof) * 1.0  # rad/s
a_max = np.ones(dof) * 2.0  # rad/s^2

toppra = ToppraTimeParameterizer(v_max, a_max)
time_stamps, trajectory = toppra.compute(path)

# ---------------------------
# 5. Print summary statistics
# ---------------------------
print(f"Trajectory duration: {time_stamps[-1]:.3f} s")
print(f"Number of samples: {trajectory.shape[0]}")
print(f"Sampled trajectory shape: {trajectory.shape}")
print(f"First waypoint: {trajectory[0]}")
print(f"Last waypoint:  {trajectory[-1]}")
