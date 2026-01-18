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
arm-planning-from-scratch/
│
├── docs/ # Concepts, diagrams, explanations
├── core/ # Core math & geometry
│ ├── kinematics/
│ ├── collision/
│ ├── path/
│ └── utils/
│
├── planners/ # Planning algorithms
│ ├── sampling/ # RRT, PRM
│ ├── optimization/ # TrajOpt / Ceres-style
│ └── cartesian/
│
├── timing/ # Time parameterization
│ ├── toppra/
│ ├── ruckig/
│ └── comparison/
│
├── demos/ # End-to-end demos
└── visualization/
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

Start here 👇

```bash
docs/00_big_picture.md
docs/01_configuration_space.md
```

Then run:

```bash
demos/01_ik_demo/
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
