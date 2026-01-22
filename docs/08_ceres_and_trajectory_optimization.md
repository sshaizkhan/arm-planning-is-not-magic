# Ceres and Trajectory Optimization (Where Nonlinear Optimization Fits)

> If planners find *where* to move, and time-parameterization decides *how fast*,
> then trajectory optimization decides *how smooth, robust, and well-behaved* the motion is.

This document explains **where Ceres Solver fits in the motion-planning stack**, what problems it is good at, and—just as importantly—what problems it should *not* be used for.

---

## 1. What Ceres Solver Actually Is

Ceres Solver is a:

> **Nonlinear least-squares optimization library**

It solves problems of the form:

```math
\min_x \sum_i | r_i(x) |^2
```

Where:

* (x) are decision variables
* (r_i(x)) are residuals (errors)

Ceres is **not**:

* A planner
* A collision checker
* A controller

---

## 2. Why Ceres Is Attractive in Robotics

Robotics problems often:

* Are nonlinear
* Are over-constrained
* Require smooth tradeoffs

Examples:

* Smooth joint trajectories
* Fitting splines to waypoints
* Enforcing soft constraints

Ceres excels here.

---

## 3. Where Ceres Fits in the Planning Pipeline

Ceres typically appears **after** geometric planning:

```
Collision-free path (OMPL)
   ↓
Initial trajectory (TOPP-RA / Ruckig / interpolation)
   ↓
Ceres optimization
   ↓
Smoothed, refined trajectory
```

It refines motion — it does not discover it.

---

## 4. Path Optimization vs Trajectory Optimization

Important distinction:

* **Path optimization** modifies geometry
* **Trajectory optimization** modifies timing and smoothness

Ceres can do either *if*:

* You provide a feasible initial guess
* You encode constraints carefully

It does **not** guarantee feasibility on its own.

---

## 5. Typical Ceres Decision Variables

Depending on formulation:

* Joint positions at knot points
* Velocities and accelerations
* Spline control points

Example:

```math
x = { q_0, q_1, ..., q_N }
```

Or:

```math
x = { p_0, p_1, ..., p_M }]
```

for spline control points.

---

## 6. Residual Design (The Real Art)

Ceres is only as good as your residuals.

Common residuals:

### Smoothness

```math
r_{smooth} = q_{k+1} - 2q_k + q_{k-1}
```

### Velocity penalties

```math
r_{vel} = \max(0, |\dot{q}| - \dot{q}_{max})
```

### Acceleration penalties

```math
r_{acc} = \max(0, |\ddot{q}| - \ddot{q}_{max})
```

### Goal tracking

```math
r_{goal} = q_N - q_{goal}
```

This turns hard constraints into **soft costs**.

---

## 7. Why Collision Constraints Are Hard in Ceres

Collision avoidance introduces:

* Non-smoothness
* Discontinuities
* Expensive gradients

As a result:

* Collision costs are approximate
* Often handled via signed distance fields

This is fragile compared to discrete collision checking.

> This is why Ceres is rarely used as a primary planner.

---

## 8. Ceres vs TOPP-RA vs Ruckig

| Tool      | Solves        | Guarantees                 |
| --------- | ------------- | -------------------------- |
| OMPL      | Geometry      | Probabilistic completeness |
| TOPP-RA   | Timing        | Time optimality            |
| Ruckig    | Online timing | Constraint satisfaction    |
| **Ceres** | Refinement    | Local optimality           |

Each tool owns a layer.

---

## 9. Common Failure Modes

Using Ceres to:

* ❌ Plan from scratch
* ❌ Enforce hard collision constraints
* ❌ Replace controllers

Symptoms:

* Slow convergence
* Unsafe trajectories
* Non-reproducible behavior

---

## 10. When Ceres Is the Right Tool

Use Ceres when:

* You already have a feasible trajectory
* You want smoother motion
* You want to trade off multiple objectives

Examples:

* Trajectory smoothing
* Time refinement
* Removing jerk spikes

---

## 11. How Ceres Is Used in Practice

Typical pattern:

1. Planner finds a path
2. Time parameterization generates timing
3. Ceres smooths and refines
4. Controller executes

Ceres is a **polishing step**.

---

## 12. Takeaway

> Ceres does not replace planning.
> It replaces hand-tuned heuristics.

It is a powerful *refinement* tool when used in the right place.

---

## 13. What Comes Next

Now that we understand:

* Planning
* Timing
* Smoothing

The next question is:

> How do we enforce constraints and execute trajectories?

Continue with:

[Seidel and Linear Programming](09_siedel_and_linear_programming.md)