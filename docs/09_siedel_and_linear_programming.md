# Seidel Solver and LP-Based Planning

> If sampling-based planners decide *where* to go, LP solvers decide *how fast you are allowed to go*.

This document explains **Seidel’s LP solver**, why it appears inside motion planners, and how linear programming quietly underpins time-parameterization and feasibility checks in robotics.

---

## 1. Where LP Fits in the Motion Stack

Recall the pipeline:

```
C-space → Collision-free path → Time-parameterized trajectory → Controller
```

Sampling-based planners (RRT, PRM):

* Ignore time
* Ignore dynamics
* Only care about *geometry*

LP solvers appear **after** a geometric path exists.

Their job:

> Enforce velocity, acceleration, and dynamic constraints *along* a fixed path.

---

## 2. What Is a Linear Program (LP)?

An LP has the form:

```math
\text{minimize } c^T x
```

subject to:

```math
Ax \le b
```

Key properties:

* Constraints are linear
* Feasible region is convex
* Global optimum is guaranteed

In motion planning, LPs are used for **feasibility**, not optimality.

---

## 3. What Is Seidel’s Algorithm?

Seidel’s algorithm is a **randomized incremental LP solver** with:

* Expected linear time in low dimensions
* Extremely fast performance for small constraint counts

Why it matters:

* Time-parameterization problems produce **many small LPs**
* Deterministic solvers are overkill

> Seidel is fast because robotics LPs are *small but frequent*.

---

## 4. Where Seidel Is Used in Practice

### TOPP-RA

* Each path segment induces linear constraints
* Velocity bounds become LP feasibility problems
* Seidel solves these repeatedly

### Velocity Scaling

* Emergency slowdowns
* Dynamic limit enforcement
* Online feasibility checks

### Convex Subproblems

* Inside nonlinear optimizers
* Used as inner feasibility checks

---

## 5. Why Not Use Ceres or IPOPT?

| Solver | Purpose                 | Cost     |
| ------ | ----------------------- | -------- |
| Seidel | Fast feasibility        | Very low |
| QP     | Optimal control         | Medium   |
| Ceres  | Nonlinear least squares | High     |

LP solvers are chosen when:

* Constraints are linear
* Speed matters more than optimality

---

## 6. Mental Model

Think of LP solvers as:

> *Traffic cops* that limit how fast you can move along a road.

They don’t plan the road.
They just enforce rules.

---

## 7. Takeaway

* Seidel is not a planner
* It is not an optimizer of motion
* It enforces **dynamic feasibility** efficiently

This is why you see it buried deep in trajectory timing code.

---

## 8. What Comes Next

Now that constraints are enforced, the trajectory must still be *executed*.

That means controllers.

Continue with:

👉 [Controllers and Execution](`docs/10_controllers_and_execution.md`)
