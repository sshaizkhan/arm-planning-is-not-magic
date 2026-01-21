# Path vs Trajectory (Where Time Finally Enters)

> A robot does **not** execute a path.
> It executes a **trajectory**.

This document explains the critical distinction between a **geometric path** and a **time-parameterized trajectory**, and why confusing the two leads to broken robots, violated limits, and unsafe motion.

---

## 1. What a Planner Actually Outputs

From the previous chapter, sampling-based planners (RRT, PRM, etc.) output:

* A sequence of configurations
* Connected by straight-line segments in C-space

Formally, this is a **path**:

$$q(s), \quad s \in [0, 1]$$

Where:

* $s$ is a *path parameter*
* $s$ is **not time**

> This distinction is everything.

---

## 2. Definition: Geometric Path

A **path** is:

* A curve in configuration space
* Collision-free
* Independent of time

Properties:

* Has length
* Has curvature
* Has no velocity
* Has no acceleration

Visual intuition:

```
q2 ↑
   |        ______
   |   ____/      \
   |__/             \
   |__________________→ q1
        s ∈ [0,1]
```

The planner’s job ends here.

---

## 3. Definition: Trajectory

A **trajectory** is:

$$q(t), \quad t \in [0, T]$$

Where:

* Time is explicit
* Motion constraints apply

A trajectory must satisfy:

* Velocity limits: $|\dot{q}| \le \dot{q}_{max}$
* Acceleration limits: $|\ddot{q}| \le \ddot{q}_{max}$
* Sometimes jerk limits

> Trajectories are *physical*. Paths are *geometric*.

---

## 4. The Missing Link: Time Parameterization

To go from path → trajectory, we introduce:

$$s = s(t)$$

Then:

$$q(t) = q(s(t))$$

This is called **path parameterization**.

The entire problem now becomes:

> How fast can we move along the path *without violating limits*?

---

## 5. Why “Just Scale the Speed” Is Wrong

A common intuition:

> “If the robot moves too fast, just reduce the speed.”

This fails because:

* Different joints move different amounts
* Velocity limits couple through the Jacobian
* Acceleration depends on curvature of the path

### Counterexample

* A sharp bend in joint space
* Even slow motion can violate acceleration limits

Speed scaling is **not constraint-aware**.

---

## 6. Waypoints Do Not Equal Time

A common misconception:

> “If I have 50 waypoints and set speed to 50%, motion slows down.”

Reality:

* Waypoints define geometry
* Controllers interpolate internally
* Timing is applied *after* interpolation

Increasing speed does **not**:

* Remove waypoints
* Change path topology

> Waypoints are not a timing mechanism.

---

## 7. Path Derivatives and Constraints

Given a path $q(s)$:

$$\dot{q} = q'(s) \cdot \dot{s}$$

$$\ddot{q} = q''(s) \cdot \dot{s}^2 + q'(s) \cdot \ddot{s}$$

Joint limits impose constraints on $\dot{s}$ and $\ddot{s}$.

This turns timing into a **1D constrained optimization problem**.

---

## 8. Time-Optimal Path Parameterization (TOPP)

Given:

* A fixed path
* Velocity and acceleration limits

Find:

> The fastest valid $s(t)$

This is the **TOPP problem**.

Key insight:

* Geometry first
* Timing second

---

## 9. Why This Separation Is Intentional

Separating planning and timing:

* Simplifies planners
* Enables reuse of paths
* Allows swapping timing algorithms

This is why:

* OMPL ignores dynamics
* TOPP-RA exists
* Ruckig exists

Each layer solves one problem well.

---

## 10. What Path Parameterization Does *Not* Do

Path parameterization:

* ❌ Does not change the path
* ❌ Does not fix collisions
* ❌ Does not optimize geometry

If the path is bad:

* Timing cannot save it

---

## 11. Takeaway (Critical Insight)

> Motion planning is a **two-stage problem**:
>
> 1. Find a collision-free path
> 2. Time-parameterize it safely

Confusing these stages causes:

* Violated limits
* Jerky motion
* Unstable control

---

## 12. What Comes Next

Now we are finally ready for the math.

The next question:

> How do we compute the *best possible* timing along a path?

Continue with:

[Time-Optimal Path Parameterization (TOPP-RA)](06_toppra.md)
