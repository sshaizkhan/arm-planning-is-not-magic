# Time-Optimal Path Parameterization (TOPP-RA)

> Once the path is fixed, timing is a **one-dimensional optimization problem**.

This document provides a math-level walkthrough of **TOPP-RA (Time-Optimal Path Parameterization via Reachability Analysis)** and explains why it fits so naturally into the modern motion-planning stack.

---

## 1. Problem Statement (Precisely)

Given:

* A geometric path :
    ```math
    (q(s),; s \in [0,1])
    ```
* Joint velocity limits:
    ```math
    (|\dot{q}*i| \le \dot{q}*{i,max})
    ```
* Joint acceleration limits:
    ```math
    (|\ddot{q}*i| \le \ddot{q}*{i,max})
    ```

Find:

> The **fastest possible** timing law (s(t)) such that all constraints are respected.

The geometry is fixed.
Only **time** is optimized.

---

## 2. Path Derivatives Revisited

Recall:

```math
\dot{q} = q'(s) \dot{s}
]
```

```math
[
\ddot{q} = q''(s) \dot{s}^2 + q'(s) \ddot{s}
]
```

Joint limits translate into constraints on:

```math
(\dot{s}) and (\ddot{s}).
```

This is the key reduction:

> A high-dimensional robot becomes a **1D system** along the path.

---

## 3. Velocity Constraints → Upper Envelope

From velocity limits:

```math
[
|q'*i(s)| \dot{s} \le \dot{q}*{i,max}
]
```

This yields:

```math
[
\dot{s} \le \dot{s}_{max}(s)
]
```

This defines a **velocity limit curve** along the path.

No timing may exceed it.

---

## 4. Acceleration Constraints → Feasible Regions

Acceleration limits impose:

```math
[
\ddot{s}*{min}(s, \dot{s}) \le \ddot{s} \le \ddot{s}*{max}(s, \dot{s})
]
```

These bounds depend on:

* Path curvature
* Current speed

This makes naive forward integration unsafe.

---

## 5. Why Greedy Integration Fails

A tempting idea:

* Go as fast as possible
* Brake when needed

This fails because:

* Future curvature matters
* Local decisions can cause dead ends

TOPP-RA solves this globally.

---

## 6. Reachability Analysis (Core Idea)

TOPP-RA asks:

> From this state `((s, \dot{s}))`, can I still reach the goal *without violating constraints*?

It computes:

* Forward reachable sets
* Backward reachable sets

And intersects them.

This guarantees feasibility.

---

## 7. Discretization Along the Path

The path is discretized into stages:

```math
[
s_0, s_1, ..., s_N
]
```

At each stage, TOPP-RA computes:

* Admissible velocity intervals

This turns the problem into a **sequence of convex constraints**.

---

## 8. Why This Becomes Linear Programming

Acceleration constraints can be written as:

```math
[
A_k \ddot{s}_k \le b_k
]
```

This yields:

* Small LPs at each stage
* Solved efficiently

This is where **Seidel’s LP solver** fits.

---

## 9. Algorithm Outline (High Level)

1. Discretize path
2. Compute velocity limits
3. Forward pass (reachability)
4. Backward pass (braking feasibility)
5. Integrate (s(t))

Result:

* Time-optimal
* Constraint-satisfying trajectory

---

## 10. What TOPP-RA Guarantees

TOPP-RA produces:

* Globally time-optimal timing
* Guaranteed constraint satisfaction

It does **not**:

* Change the path
* Handle dynamic obstacles
* Run cheaply online

---

## 11. Why TOPP-RA Is Mostly Offline

TOPP-RA:

* Requires full path knowledge
* Uses global reasoning
* Is relatively heavy

Thus it is best suited for:

* Offline planning
* Precomputed trajectories

---

## 12. Takeaway

> TOPP-RA is optimal because it looks **ahead**.

It treats time parameterization as a global feasibility problem, not a local control problem.

---

## 13. What Comes Next

TOPP-RA is mathematically elegant.

But industry often prefers something else.

Why?

Continue with:

👉 [Ruckig vs TOPP-RA (Why Industry Chooses Ruckig)](`docs/07_ruckig_vs_toppra.md`)
