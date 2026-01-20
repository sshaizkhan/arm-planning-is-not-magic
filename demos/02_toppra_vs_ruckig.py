"""
Demo 02 Extended: Compare TOPP-RA vs Ruckig with plots

Shows joint positions, velocities, accelerations, and jerk for both
time parameterizations.
"""

import numpy as np
import matplotlib.pyplot as plt

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

# ---------------------------
# 4. Ruckig time parameterization
# ---------------------------
j_max = np.ones(dof) * 5.0
ruckig_param = RuckigTimeParameterizer(v_max, a_max, j_max)
time_ruckig, traj_ruckig = ruckig_param.compute(path, dt=0.01)

# ---------------------------
# 5. Compute derivatives
# ---------------------------


def compute_derivatives(traj, time):
    dt = np.diff(time)
    vel = np.diff(traj, axis=0) / dt[:, None]
    acc = np.diff(vel, axis=0) / dt[:-1][:, None]
    jerk = np.diff(acc, axis=0) / dt[:-2][:, None]

    t_vel = time[1:]
    t_acc = time[2:]
    t_jerk = time[3:]

    return vel, acc, jerk, t_vel, t_acc, t_jerk


vel_t, acc_t, jerk_t, t_vel_t, t_acc_t, t_jerk_t = compute_derivatives(traj_toppra, time_toppra)
vel_r, acc_r, jerk_r, t_vel_r, t_acc_r, t_jerk_r = compute_derivatives(traj_ruckig, time_ruckig)

# ---------------------------
# 6. Plot
# ---------------------------
joint_labels = [f'Joint {i+1}' for i in range(dof)]

fig, axes = plt.subplots(4, 1, figsize=(12, 16))
plt.subplots_adjust(hspace=0.4)

# Joint positions
for i in range(dof):
    axes[0].plot(time_toppra, traj_toppra[:, i], '--', label=f'TOPP-RA {joint_labels[i]}')
    axes[0].plot(time_ruckig, traj_ruckig[:, i], '-', label=f'Ruckig {joint_labels[i]}')
axes[0].set_title("Joint Positions")
axes[0].set_xlabel("Time [s]")
axes[0].set_ylabel("Position [rad]")
axes[0].legend()

# Joint velocities
for i in range(dof):
    axes[1].plot(t_vel_t, vel_t[:, i], '--', label=f'TOPP-RA {joint_labels[i]}')
    axes[1].plot(t_vel_r, vel_r[:, i], '-', label=f'Ruckig {joint_labels[i]}')
axes[1].set_title("Joint Velocities")
axes[1].set_xlabel("Time [s]")
axes[1].set_ylabel("Velocity [rad/s]")
axes[1].legend()

# Joint accelerations
for i in range(dof):
    axes[2].plot(t_acc_t, acc_t[:, i], '--', label=f'TOPP-RA {joint_labels[i]}')
    axes[2].plot(t_acc_r, acc_r[:, i], '-', label=f'Ruckig {joint_labels[i]}')
axes[2].set_title("Joint Accelerations")
axes[2].set_xlabel("Time [s]")
axes[2].set_ylabel("Acceleration [rad/s^2]")
axes[2].legend()

# Joint jerks
for i in range(dof):
    axes[3].plot(t_jerk_t, jerk_t[:, i], '--', label=f'TOPP-RA {joint_labels[i]}')
    axes[3].plot(t_jerk_r, jerk_r[:, i], '-', label=f'Ruckig {joint_labels[i]}')
axes[3].set_title("Joint Jerks")
axes[3].set_xlabel("Time [s]")
axes[3].set_ylabel("Jerk [rad/s^3]")
axes[3].legend()

plt.show()
