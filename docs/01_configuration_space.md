# Configuration Space (C-Space)

> If you don’t understand configuration space, motion planning will always feel like magic.

This document explains **configuration space (C-space)** from first principles and shows why *all serious robot arm planning happens there*, not in Cartesian space.

---

## 1. What Is Configuration Space?

A robot’s **configuration** is the *minimum set of variables* needed to completely describe its pose.

For a robot arm, those variables are its **joint values**.

### Example: 2‑DOF Planar Arm

```math
[
q = [\theta_1, \theta_2]
]
```

This vector lives in:

```math
[
\mathcal{C} = \mathbb{R}^2
]
```

This space is called **configuration space**, or **C‑space**.

> A single point in C‑space represents the *entire robot*, not just its end‑effector.

---

## 2. C‑Space vs Task Space (Why This Matters)

### Task (Cartesian) Space

* Describes **end‑effector position and orientation**
* Example:
  ```math
  [
  (x, y) \in \mathbb{R}^2
  ]
  ```

### Configuration Space

* Describes **joint angles**
* Example:
  ```math
  [
  (\theta_1, \theta_2) \in \mathbb{R}^2
  ]
  ```

### Critical Difference

* Many **C‑space points map to the same task‑space point**
* Task‑space paths ignore joint limits, singularities, and collisions

> The robot does **not** move in task space.
> It only moves in configuration space.

This is why planning in Cartesian space alone fails for real robots.

---

## 3. Forward Kinematics Is a Mapping

Forward kinematics (FK) is a function:

```math
[
f : \mathcal{C} \rightarrow \mathcal{W}
]
```

Where:

```math
* (\mathcal{C}) = configuration space
* (\mathcal{W}) = workspace (task space)
```

For a planar 2‑DOF arm:

```math
    [
x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2)
```

```math
[
y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2)
]
```

Important consequences:

* FK is **many‑to‑one**
* C‑space is usually **higher dimensional** than workspace

---

## 4. Joint Limits Define the Shape of C‑Space

Real joints are limited:

```math
[
\theta_i^{min} \le \theta_i \le \theta_i^{max}
]
```

For a 2‑DOF arm:

* C‑space becomes a **bounded square**
* Often visualized as a torus if joints wrap

> C‑space is not infinite — it has structure.

These limits are **hard constraints**.

No planner is allowed to violate them.

---

## 5. Obstacles Live in C‑Space (Not Where You Expect)

An obstacle in workspace becomes a **region of forbidden configurations** in C‑space.

### Key idea

> An obstacle in Cartesian space turns into a *curved, unintuitive shape* in C‑space.

This is why:

* Collision checking is done in C‑space
* Obstacle shapes look "weird" in joint space

Even a simple circular obstacle can carve out complex forbidden regions.

---

## 6. Free Space vs Forbidden Space

C‑space is split into two sets:

```math
* **(\mathcal{C}_{free})** — collision‑free configurations
```
```math
* **(\mathcal{C}_{obs})** — configurations in collision
```

Planning is simply:

Find a continuous path through:
```math
(\mathcal{C}_{free})
```

Nothing more. Nothing less.

---

## 7. Why Shortest Paths Are Not Obvious

In task space:

* A straight line *looks* optimal

In C‑space:

* That same motion may:

  * Violate joint limits
  * Cause self‑collision
  * Require large joint motion

> Shortest Cartesian path ≠ shortest joint‑space path

This is one of the biggest conceptual hurdles in robotics.

---

## 8. Dimensionality Explosion (Why Planning Is Hard)

| Robot              | DOF | C‑Space Dimension |
| ------------------ | --- | ----------------- |
| Planar arm         | 2   | 2                 |
| Industrial arm     | 6   | 6                 |
| Mobile manipulator | 10+ | 10+               |

Each extra DOF:

* Multiplies complexity
* Breaks grid‑based methods

This is why **sampling‑based planners** exist.

---

## 9. What Configuration Space Is *Not*

C‑space does **not**:

* Encode time
* Encode velocity
* Encode acceleration
* Encode smoothness

Those come **later**.

> C‑space only answers: *Which configurations are allowed?*

---

## 10. How This Maps to Real Planners

| Concept         | Real System            |
| --------------- | ---------------------- |
| C‑space         | Joint state space      |
| FK              | Robot model            |
| Joint limits    | URDF / hardware limits |
| C‑free          | Collision checker      |
| Path in C‑space | OMPL output            |

If you understand this mapping, MoveIt stops being mysterious.

---

## 11. Takeaway (Read This Twice)

> Motion planning is **not** about moving an end‑effector.
> It is about moving a **point through a high‑dimensional space**.

Once that clicks, everything else becomes engineering.

---

## 12. What Comes Next

Now that we understand C‑space, the next questions are natural:

1. How do we *invert* FK? (Inverse Kinematics)
2. How do we mark forbidden regions? (Collision checking)
3. How do we move through free space? (Path planning)

Raad more here:

[Configuration Space - Tufts University](https://www.cs.tufts.edu/comp/150IR/hw/cspace.html)

Continue with:

👉 [Inverse Kinematics (IK) vs Motion Planning](`docs/02_ik_vs_planning.md`)
