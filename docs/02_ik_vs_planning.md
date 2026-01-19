# Inverse Kinematics (IK) vs Motion Planning

> One of the most common confusions in robotics is believing that inverse kinematics *is* motion planning.
>
> It is not.

This document explains **what inverse kinematics actually does**, what it **cannot do**, and how it fits into the broader motion‑planning pipeline.

---

## 1. The Core Question IK Answers

Inverse kinematics answers a **single, static question**:

> **Which joint configurations place the end‑effector at a desired pose?**

Mathematically:

```math
[
f(q) = x_{ee}
]
```
IK solves:

```math
[
q ;\text{such that}; f(q) = x_{ee}
]
```
That’s it.

No motion. No time. No obstacles.

---

## 2. IK Is a Mapping, Not a Path

Forward kinematics maps:

```math
[
q \rightarrow x
]
```
Inverse kinematics maps:

```math
[
x \rightarrow {q_1, q_2, ..., q_n}
]
```
### Visual intuition

```
Task space (pose goal)

      ●  target pose

            ↓ IK

C-space (joint solutions)

   θ2 ↑
      |   ● q₁
      |   ● q₂
      |   ● q₃
      |
      |________→ θ1
```

Key observation:

> IK returns **points**, not **paths**.

---

## 3. Multiple IK Solutions Are the Norm

For most robot arms:

* There are **multiple valid IK solutions**
* Example: elbow‑up vs elbow‑down

### 2‑DOF planar arm example

```math
[
q = [\theta_1, \theta_2]
]
```
A single (x, y) can often be reached by **two** joint configurations.

Industrial 6‑DOF arms often have:

* 4–8 discrete IK solutions

> IK does not choose *which* solution is better.

---

## 4. What IK Completely Ignores

Inverse kinematics does **not** consider:

* ❌ Joint‑space distance from current configuration
* ❌ Collisions (self or environment)
* ❌ Singularities (unless explicitly handled)
* ❌ Velocity or acceleration limits
* ❌ Smoothness or feasibility of motion

IK answers *“Can I be here?”*, not *“How should I get there?”*

---

## 5. Why IK Alone Is Dangerous

### Example failure mode

```
q_start ●───────────────┐
                        │   obstacle
                        │   in C-space
                        │
q_goal  ●───────────────┘
```

IK gives you `q_goal`, but:

* The straight‑line motion in joint space collides
* A different IK solution might work
* Or no safe path may exist at all

IK cannot tell you this.

---

## 6. Motion Planning Solves a Different Problem

Motion planning asks:

> **Is there a continuous, collision‑free path between two configurations?**

Formally:

```math
[
\exists; q(t),; t \in [0,1]
]
```
Such that:

```math
* (q(0) = q_{start})
```
```math
* (q(1) = q_{goal})
```
```math
* (q(t) \in \mathcal{C}_{free}) for all (t)
```
This is a **connectivity problem**, not an inversion problem.

---

## 7. How IK and Planning Work Together

In real systems:

1. IK generates **candidate goal configurations**
2. The planner tries to **connect** start → goal
3. Failed IK solutions are discarded

### Visual intuition

```
Pose goal
   ↓ IK
{ q₁, q₂, q₃ }
      ↓ planning
q_start ───────► q₂   (only one is reachable)
```

Planning *selects* among IK solutions.

---

## 8. Analytical IK vs Numerical IK

### Analytical IK (e.g., OPW Kinematics)

* Closed‑form
* Deterministic
* Extremely fast
* Returns all valid solutions

### Numerical IK

* Iterative
* May fail or converge slowly
* Sensitive to initial guess

> Industrial systems strongly prefer analytical IK when available.

---

## 9. Why OPW Kinematics Fits Here

OPW kinematics:

* Solves IK for common 6‑DOF industrial arms
* Exploits their specific geometry
* Produces up to 8 exact solutions

It fits **here** in the stack:

```
Pose goal
   ↓ OPW IK
Joint candidates
   ↓ OMPL / planner
Collision‑free path
```

OPW does **not** replace planning.

---

## 10. Common Beginner Mistakes

* Using IK in a loop and calling it "planning"
* Interpolating linearly between IK solutions
* Ignoring alternative IK branches
* Blaming planners for IK failures

If you’ve done this — you’re not alone.

---

## 11. Takeaway (Read This Carefully)

> Inverse kinematics answers *where* you can be.
> Motion planning answers *how* you can move.

They solve **orthogonal problems**.

Both are necessary.

---

## 12. What Comes Next

Now that we can:

* Describe configurations (C‑space)
* Generate goal configurations (IK)

The next missing piece is obvious:

> **Which configurations are forbidden?**

That is the job of **collision checking**.

Continue with:

👉 [Collision Checking](`docs/03_collision_checking.md`)
