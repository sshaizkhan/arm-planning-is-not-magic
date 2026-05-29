# Sampling-Based Planning (Why Randomness Wins)

> If collision checking were free, motion planning would be trivial.
> Because it isn’t, **sampling-based planners dominate robotics**.

This document explains *why* classical planning fails in high-dimensional configuration spaces and how algorithms like **RRT** and **PRM** exploit randomness to make planning feasible.

---

## 1. The Planning Problem (Restated Precisely)

Given:

* A start configuration :
```math
(q_{start} \in \mathcal{C}_{free})
```
* A goal configuration:
```math
(q_{goal} \in \mathcal{C}_{free})
```
* A collision checker

Find:

```math
[
q(t), ; t \in [0,1]
]
```
Such that:

```math
(q(0) = q_{start})
```
```math
(q(1) = q_{goal})
```
```math
(q(t) \in \mathcal{C}_{free}) for all (t)
```

Nothing more is required.

---

## 2. Why Grid-Based Planning Fails

A tempting idea:

* Discretize C-space
* Run **A\*** or **Dijkstra**

### The dimensionality problem

If each joint is discretized into (N) values:

```math
\text{states} = N^d
```

| DOF | N=100 | States |
| --- | ----- | ------ |
| 2   | 100²  | 10⁴    |
| 6   | 100⁶  | 10¹²   |

> You cannot even *store* the grid for real robots.

This is the **curse of dimensionality**.

---

## 3. The Key Insight: Most Space Is Empty

In high-dimensional C-space:

* Obstacles occupy a small fraction of volume
* Free space is vast but oddly shaped

Instead of enumerating space:

> **Sample it.**

Random sampling:

* Avoids exponential memory growth
* Focuses computation where needed

---

## 4. Sampling-Based Planning in One Sentence

> Build a graph or tree by randomly sampling valid configurations and connecting them locally.

Everything else is a detail.

---

## 5. Probabilistic Completeness (The Critical Guarantee)

Sampling-based planners are:

> **Probabilistically complete**

Meaning:

* If a path exists
* The probability of finding it approaches 1 as time → ∞

They are **not**:

* Deterministically complete
* Guaranteed to find a path quickly

This tradeoff is intentional.

---

## 6. Rapidly-Exploring Random Trees (RRT)

### Core idea

Grow a tree outward from the start:

1. Sample a random configuration (q_{rand})
2. Find nearest tree node (q_{near})
3. Steer toward (q_{rand})
4. Add the new node if collision-free

### Visualization

```
q_start ●───●───●───●
             \      \
              ●      ●
```

RRT naturally:

* Explores large spaces quickly
* Pushes toward unexplored regions

---

## 7. RRT Strengths and Weaknesses

### Strengths

* Extremely fast
* Handles narrow passages reasonably well
* Simple to implement

### Weaknesses

* Paths are jagged
* Not optimal
* Sensitive to step size

This leads to variants.

---

## 8. RRT* and Asymptotic Optimality

RRT* modifies RRT by:

* Rewiring the tree
* Minimizing path cost

Guarantee:

> As samples → ∞, solution → optimal

Cost:

* More collision checks
* Slower convergence

In practice:

* Often stopped early
* Followed by smoothing

---

## 9. Probabilistic Roadmaps (PRM)

PRM builds a **graph**, not a tree:

1. Sample many valid configurations
2. Connect nearby samples
3. Reuse the roadmap for many queries

### Visualization

```
●──●──●
|  |  |
●──●──●
```

PRM works best when:

* Environment is mostly static
* Many planning queries are needed

---

## 10. Why OMPL Exists

OMPL provides:

* RRT, RRT*, PRM, PRM*, EST, BIT*, etc.
* Abstract interfaces for:

  * Sampling
  * Distance metrics
  * Collision checking

OMPL does **not**:

* Know about robots
* Handle trajectories
* Enforce dynamics

> OMPL solves the *geometric* problem only.

---

## 11. Why Planners Don’t Care About Time

Sampling-based planners operate in:

```math
[
\mathcal{C}
]
```
Not:

```math
[
\mathcal{C} \times \mathbb{R}
]
```
They ignore:

* Velocity
* Acceleration
* Torque

This is by design.

> Time comes *after* geometry.

---

## 12. Path Quality and Post-Processing

Planner output:

* A **piecewise-linear path** in joint space

Typical post-processing:

* Shortcutting
* Spline fitting
* Waypoint reduction

Still no timing yet.

---

## 13. Takeaway

> Sampling-based planners trade certainty for scalability.

They work because:

* Collision checking is expensive
* High-dimensional geometry is hostile

Randomness is not a hack.
It is the solution.

---

## 14. What Comes Next

We now have:

* A collision-free **path**

But robots do not execute paths.
They execute **time-parameterized trajectories**.

The next missing piece:

> How do we add time, velocity, and acceleration *safely*?

## Exercises

1. **Planner variance.** Run `demos/03_compare_planners.py` and note which planner is fastest. Run it 5 more times. Does the ranking change? Why? What property of sampling-based planners causes this non-determinism, and how would you make the comparison reproducible?

2. **PRM in dynamic environments.** PRM builds a roadmap once and reuses it for many queries. Describe specifically what breaks if you use a prebuilt PRM roadmap after an obstacle has moved into the scene. Which part of the roadmap becomes invalid? Would a partial invalidation strategy (mark affected edges, replan only those) be correct? What would it cost?

3. **Counting collision checks.** Open `planners/ompl_rrt.py`. Add an integer counter that increments each time the validity checker is called. Expose this count as an attribute on the planner after `plan()` returns. Run two planning problems — one easy (open space) and one hard (narrow passage). How do the counts differ? What does this tell you about where planning time is spent?

4. **Step size sensitivity.** RRT's step size controls how far the tree extends toward each random sample. In the OMPL RRT wrapper, locate the parameter that controls this (often called `range` or `max_distance`). Double it and halve it, then compare path quality (waypoint count, path length) and planning time. Explain the tradeoff geometrically: why does a large step size hurt in cluttered environments?

---

Continue with:

[Path vs Trajectory](05_path_vs_trajectory.md)
