# arm-planning-is-not-magic 🦾
The goal of this repository is to make robot arm planning understandable. The repo is structured in separate modules of geometry, planning, optimization and timing. The idea is to show why things exists and not just how.


> Motion planning for robot arms is not magic.
> This repo exists to prove that — step by step.

This repository is a **from-first-principles walkthrough** of how
5–6 DOF robotic arm motion planning actually works, from geometry to
time-parameterized trajectories.

The goal is **understanding**, not replacing MoveIt or industrial planners.

---

## Why This Repo Exists

Most tutorials jump straight to:
- “Use MoveIt”
- “Tune these parameters”
- “It works (somehow)”

This repo instead answers:
- What is configuration space?
- Why IK is not planning
- Why OMPL outputs paths, not trajectories
- Where TOPP-RA and Ruckig fit
- Why speed changes waypoint count
- Why industrial robots retime everything anyway

If you've ever thought:
> “I know how to *use* planners, but I don’t really know what they do”

This repo is for you.

---

## Learning Philosophy

We build the planning stack **in the same order it exists conceptually**:

1. Configuration space
2. Kinematics (FK / IK)
3. Collision checking
4. Path planning (OMPL-style)
5. Path post-processing
6. Time parameterization (TOPP-RA)
7. Jerk-limited execution (Ruckig)

Each layer:
- Is minimal
- Is visualized
- Depends only on previous layers

---

## Repo Structure

```bash
arm-planning-is-not-magic/
│
├── docs/                      # Concepts, diagrams, explanations
│   ├── 01_configuration_space.md
│   ├── 02_ik_vs_planning.md
│   ├── 03_collision_checking.md
│   ├── 04_sampling_based_planning.md
│   ├── 05_path_vs_trajectory.md
│   ├── 06_toppra.md
│   ├── 07_ruckig_vs_toppra.md
│   └── ...
│
├── core/                      # Core abstractions
│   ├── robot_model.py         # Robot interface + UR5 implementation
│   ├── state_space.py         # C-space sampling, validation, interpolation
│   ├── collision_manager.py   # Collision shapes and checking
│   └── kinematics/
│       ├── opw.py             # OPW closed-form IK (8 solutions)
│       └── opw_parameters.py  # Robot-specific kinematic parameters
│
├── planners/                  # OMPL-based sampling planners
│   ├── ompl_rrt.py            # RRT (single-tree)
│   ├── ompl_rrt_connect.py    # RRT-Connect (bidirectional)
│   ├── ompl_rrt_star.py       # RRT* (asymptotically optimal)
│   ├── ompl_prm.py            # PRM (roadmap)
│   ├── ompl_kpiece1.py        # KPIECE1 (cell decomposition)
│   ├── ompl_est.py            # EST (expansive trees)
│   └── ompl_bitrrt.py         # BiTRRT (bidirectional transition)
│
├── parameterization/          # Path → Trajectory conversion
│   ├── toppra_parameterization.py   # TOPP-RA (time-optimal, offline)
│   └── ruckig_parameterization.py   # Ruckig (jerk-limited, online)
│
├── demos/                     # End-to-end examples
│   ├── 01_plan_and_time.py    # Basic planning + timing pipeline
│   ├── 02_toppra_vs_ruckig.py # Compare parameterizers visually
│   ├── 03_compare_planners.py # Benchmark all 7 OMPL planners
│   └── 04_collision_demo.py   # Planning with obstacles
│
├── docker/                    # Container setup
└── requirements.txt
```


---

## What This Repo Is NOT

- ❌ A MoveIt replacement
- ❌ A ROS tutorial
- ❌ A production motion planner

It **is**:
- A mental model builder
- A planning playground
- A bridge between theory and practice

---

## Getting Started

### Read the docs (in order)

```bash
docs/01_configuration_space.md   # What is C-space?
docs/02_ik_vs_planning.md        # Why IK ≠ planning
docs/03_collision_checking.md    # How collision works
docs/04_sampling_based_planning.md  # RRT, PRM, etc.
docs/05_path_vs_trajectory.md    # Path vs trajectory
docs/06_toppra.md                # Time-optimal parameterization
docs/07_ruckig_vs_toppra.md      # Industry reality
```

### Run the demos

```bash
# Basic planning + timing pipeline
python demos/01_plan_and_time.py

# Compare TOPP-RA vs Ruckig (generates plots)
python demos/02_toppra_vs_ruckig.py

# Benchmark all 7 planners
python demos/03_compare_planners.py --parameterizer toppra

# Planning with collision obstacles
python demos/04_collision_demo.py
```

### Quick code example

```python
from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from planners import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer
import numpy as np

# 1. Setup robot and state space
robot = UR5RobotModel()
state_space = JointStateSpace(robot)

# 2. Plan a collision-free path
planner = OMPLRRTConnectPlanner(state_space)
q_start = np.zeros(6)
q_goal = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
path = planner.plan(q_start, q_goal, timeout=2.0)

# 3. Time-parameterize the path
v_max = np.ones(6) * 1.0  # rad/s
a_max = np.ones(6) * 2.0  # rad/s^2
parameterizer = ToppraTimeParameterizer(v_max, a_max)
time_stamps, trajectory = parameterizer.compute(path)

print(f"Trajectory duration: {time_stamps[-1]:.2f}s")
```


---

## Inspiration & References

- OMPL
- MoveIt
- TOPP-RA
- Ruckig
- Modern industrial robot controllers

---

## License

MIT — use it, break it, teach with it.
