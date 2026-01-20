#!/usr/bin/env python3
"""
Demo 02: Compare TOPP-RA vs Ruckig time parameterization for the same path.

This demo plans a single path using OMPL RRT, then computes two trajectories:
1. TOPP-RA (time-optimal)
2. Ruckig (jerk-limited, controller-friendly)

Prints summary stats and allows visual comparison.
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager
from planners.ompl_rrt import OMPLRRTPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from parameterization.rucking_parameterization import RuckigTimeParameterizer

# ---------------------------
# 1. Robot setup
# ---------------------------
robot = UR5RobotModel()
collision_manager = NullCollisionManager()
robot.in_collision = collision_manager.in_collision
state_space = JointStateSpace(robot)

# ---------------------------
# 2. Plan path
# ---------------------------
dof = robot.dof()
q_start = np.zeros(dof)
q_goal = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
planner = OMPLRRTPlanner(state_space, step_size=0.1)
path = planner.plan(q_start, q_goal, timeout=2.0)

if path is None:
    print("No path found!")
    exit(1)

print(f"Planned path with {len(path)} waypoints.")

# ---------------------------
# 3. TOPP-RA time parameterization
# ---------------------------
v_max = np.ones(dof) * 1.0
a_max = np.ones(dof) * 2.0

toppra = ToppraTimeParameterizer(v_max, a_max)
time_toppra, traj_toppra = toppra.compute(path)

print("\nTOPP-RA Trajectory:")
print(f"Duration: {time_toppra[-1]:.3f} s")
print(f"Samples: {traj_toppra.shape[0]}")
print(f"First waypoint: {traj_toppra[0]}")
print(f"Last waypoint: {traj_toppra[-1]}")

# ---------------------------
# 4. Ruckig time parameterization
# ---------------------------
j_max = np.ones(dof) * 5.0  # jerk limit
ruckig_param = RuckigTimeParameterizer(v_max, a_max, j_max)
time_ruckig, traj_ruckig = ruckig_param.compute(path, dt=0.01)

print("\nRuckig Trajectory:")
print(f"Duration: {time_ruckig[-1]:.3f} s")
print(f"Samples: {traj_ruckig.shape[0]}")
print(f"First waypoint: {traj_ruckig[0]}")
print(f"Last waypoint: {traj_ruckig[-1]}")

# ---------------------------
# 5. Optional: Compare max velocities / accelerations / jerks
# ---------------------------


def compute_derivatives(traj, dt):
    vel = np.diff(traj, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    jerk = np.diff(acc, axis=0) / dt
    return vel, acc, jerk


vel_t, acc_t, jerk_t = compute_derivatives(traj_toppra, time_toppra[1]-time_toppra[0])
vel_r, acc_r, jerk_r = compute_derivatives(traj_ruckig, time_ruckig[1]-time_ruckig[0])

print("\nTOPP-RA max velocity / acceleration / jerk:")
print(np.max(np.abs(vel_t)), np.max(np.abs(acc_t)), np.max(np.abs(jerk_t)))

print("Ruckig max velocity / acceleration / jerk:")
print(np.max(np.abs(vel_r)), np.max(np.abs(acc_r)), np.max(np.abs(jerk_r)))
