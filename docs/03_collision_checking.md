# Collision Checking (The Real Core of Motion Planning)

> Planning is easy.
> **Collision checking is the hard part.**

This document explains what collision checking *actually* is, why it dominates planning time, and how real systems (MoveIt, OMPL, industrial stacks) reason about collisions in configuration space.

---

## 1. What Does “Collision” Mean for a Robot Arm?

A robot arm is not a point.
It is a **collection of rigid bodies** connected by joints.

A configuration (q) is **valid** if:

* No link intersects the environment
* No link intersects another link (self-collision)
* No joint limit is violated

Formally:

```math
[
q \in \mathcal{C}_{free}
]
```
Otherwise:

```math
[
q \in \mathcal{C}_{obs}
]
```
> Collision checking is a *predicate*:
>
> **Is this configuration allowed? Yes or no.**

---

## 2. Collision Checking Happens in C-Space (Always)

Recall from the previous document:

* A **single point** in C-space represents the *entire robot*
* Obstacles carve out **regions** in C-space

So collision checking answers:

> “Does this point in C-space lie inside a forbidden region?”

### Visual intuition

```
Workspace:                        C-space:

   y ↑                             θ2 ↑
      |                                |
      |   ◯ obstacle                   |   ███████
      |                                | ████  ███
      |                                | ████  ███
      |                                |   ███████
      |________→ x                    |________→ θ1
```

A simple obstacle becomes a complex shape in joint space.

---

## 3. Why Collision Checking Is the Bottleneck

A planner does **not** check collisions once.

It checks collisions:

* Thousands of times
* Sometimes millions
* For every sampled configuration
* For every edge between configurations

Typical breakdown:

| Component              | Time Spent   |
| ---------------------- | ------------ |
| Sampling               | small        |
| Nearest neighbor       | moderate     |
| **Collision checking** | **dominant** |

> Fast collision checking matters more than clever planning logic.

---

## 4. Types of Collision Checking

### 4.1 Self-Collision

* Link–link intersections
* Often precomputed as a **self-collision matrix**
* Adjacent links are usually ignored

Why this matters:

* Self-collision constraints are *configuration dependent*
* They dramatically shrink:
```math
(\mathcal{C}_{free})
```

---

### 4.2 Environment Collision

* Robot vs world
* Requires a geometric model of:

  * Robot links
  * Environment objects

Common representations:

* Meshes (accurate, slow)
* Convex hulls (faster)
* Primitive shapes (fastest)

---

## 5. Discrete vs Continuous Collision Checking

### Discrete Collision Checking

Check only the endpoints:

```math
[
q_1, q_2
]
```
Problem:

* Robot may collide *between* them

```
q1 ●────────● q2   (collision missed)
```

---

### Continuous Collision Checking (CCD)

Check the **swept volume** between configurations:

```math
[
q(t) = (1 - t) q_1 + t q_2
]
```
This guarantees:

* No missed collisions
* Higher computational cost

> Industrial systems increasingly rely on CCD.

---

## 6. Edge Validity vs State Validity

Planners check two things:

### State validity

```math
[
\text{isValid}(q)
]
```
Used for:

* Sampling
* Tree expansion

### Edge validity

```math
[
\text{isValid}(q_1 \rightarrow q_2)
]
```
Used for:

* Connecting nodes
* Path smoothing

This distinction matters later for trajectory generation.

---

## 7. Collision Checking APIs in Real Systems

### MoveIt

* Uses FCL (Flexible Collision Library)
* Maintains a planning scene
* Separates:

  * Robot state
  * World state

### OMPL

* Planner-agnostic
* Expects a user-defined validity checker:

```cpp
bool isStateValid(q)
```

> OMPL does not know what a robot is.
> You teach it.

---

## 8. Resolution, Interpolation, and Lies

Collision checking is **never exact**.

Tradeoffs:

* More interpolation → safer → slower
* Less interpolation → faster → riskier

This leads to:

* False negatives (missed collisions)
* Rarely, false positives

> Planning is probabilistically correct, not perfect.

---

## 9. Why This Shapes Planner Design

Collision checking cost determines:

* Step size
* Planner choice (RRT vs PRM)
* Shortcutting aggressiveness

Planners are designed to:

* Minimize collision checks
* Reuse validity results

This is why:

* Sampling-based planners win
* Exact planners lose

---

## 10. What Collision Checking Does *Not* Do

Collision checking does **not**:

* Enforce smoothness
* Enforce velocity limits
* Enforce acceleration limits
* Decide timing

It only answers:

> “Is this configuration or motion allowed geometrically?”

---

## 11. Takeaway

> Motion planning is a search problem.
> Collision checking defines the search space.

If collision checking is slow or wrong:

* Planning fails
* Or worse — looks correct but isn’t

---

## 12. What Comes Next

Now that we know:

* What space we plan in
* How we mark forbidden regions

The next question is inevitable:

> How do we move through this space efficiently?

## Exercises

1. **5-link blind spot.** Open `core/collision_manager.py`. The `ShapeCollisionManager` checks positions of 5 key link frames. Name a robot posture where all 5 link positions clear every obstacle yet the robot is still in collision. (Hint: think about the volume swept between checked frames — a long forearm link between two checked points can arc through an obstacle without either endpoint being inside it.)

2. **Add a Capsule shape.** A capsule is a cylinder with hemispherical caps — commonly used for arm links because it tightly fits cylindrical geometry. Design the interface: what constructor arguments does a `Capsule` need? What methods must it implement to plug into the existing collision system (`contains_point()`, or distance-based)? Write a stub class and a unit test that verifies a point at the capsule center is in collision and a point 10 meters away is not.

3. **Why return True on FK failure.** In `core/collision_manager.py`, when forward kinematics throws an exception, the manager returns `True` (assume collision) rather than `False` (assume free). Explain the safety reasoning behind this decision. Give a concrete scenario where returning `False` on an FK error would cause a physical robot to execute an unsafe motion. Now give a scenario where returning `True` causes the planner to spuriously fail. Which failure mode is preferable in a real system?

4. **Resolution and false negatives.** The edge validity check interpolates between `q1` and `q2` at a fixed step size. Construct a test case (two configs and an obstacle) where a step size of `0.1` rad misses a collision that a step size of `0.01` rad catches. What is the maximum obstacle "width" (in C-space radians) that can be missed at a given step size, and how does this relate to the robot's link geometry in workspace?

---

Continue with:

[Sampling-Based Planning](04_sampling_based_planning.md)
