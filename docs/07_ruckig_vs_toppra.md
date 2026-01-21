# Ruckig vs TOPP-RA (Why Industry Chooses Ruckig)

> TOPP-RA is optimal.
> Ruckig is practical.

This document explains **why modern industrial robots overwhelmingly favor Ruckig-style online trajectory generation**, even though TOPP-RA offers stronger theoretical guarantees.

---

## 1. Different Problems, Same Confusion

Both TOPP-RA and Ruckig deal with:

* Velocity limits
* Acceleration limits
* Jerk limits (Ruckig)

But they solve **different problems**.

---

## 2. TOPP-RA Assumptions

TOPP-RA assumes:

* Full path known in advance
* Static environment
* Offline computation acceptable

It optimizes:

* Total execution time

This is a **global optimization problem**.

---

## 3. Ruckig Assumptions

Ruckig assumes:

* Control loop execution
* New targets arriving every cycle
* Disturbances and replanning

It optimizes:

* Smoothness
* Constraint satisfaction
* Responsiveness

This is a **local, online problem**.

---

## 4. Mathematical Framing Difference

### TOPP-RA

* Optimizes (s(t)) globally
* Uses reachability analysis
* Solves LPs along the path

### Ruckig

* Solves a boundary-value problem
* Polynomial motion profiles
* Closed-form time computation

> Ruckig trades optimality for reactivity.

---

## 5. Jerk Is the Deciding Factor

TOPP-RA:

* Typically limits velocity and acceleration
* Jerk emerges implicitly

Ruckig:

* Explicit jerk constraints
* Much smoother motion

Controllers *love* jerk-bounded trajectories.

---

## 6. Online Replanning Reality

In real systems:

* Goals change
* Sensors update
* Humans interfere

TOPP-RA:

* Must be rerun
* Computationally heavy

Ruckig:

* Recomputes in microseconds
* Designed for this scenario

---

## 7. Safety and Predictability

Ruckig guarantees:

* Continuity up to jerk
* Predictable stopping behavior

This matters for:

* Collaborative robots
* Safety certification

Optimality is secondary.

---

## 8. Architectural Placement

```
Path (OMPL)
   ↓
Optional TOPP-RA (offline)
   ↓
Ruckig (online)
   ↓
Controller
```

Many systems:

* Skip TOPP-RA entirely
* Let Ruckig handle timing

---

## 9. Why Research Loves TOPP-RA

* Clean math
* Strong guarantees
* Benchmark-friendly

Why industry doesn’t:

* Environments change
* Optimality rarely matters

---

## 10. When TOPP-RA Is Still the Right Tool

Use TOPP-RA when:

* Path is long and fixed
* Time truly matters
* Motion is offline

Examples:

* Industrial welding
* Preplanned machining

---

## 11. When Ruckig Is the Right Tool

Use Ruckig when:

* Motion is interactive
* Replanning is frequent
* Smoothness matters

Examples:

* Pick-and-place
* Human–robot collaboration

---

## 12. Final Takeaway

> TOPP-RA answers: *What is the fastest possible motion?*
>
> Ruckig answers: *What is the best motion **right now**?*

Industry chooses the second question.

---

## 13. What Comes Next

We now understand:

* Geometry
* Planning
* Timing

The final layer remains:

> How does this become control signals?

Optional next topics:

* Controllers
* Ceres-based smoothing
* Full pipeline examples

Continue with:

[Ceres and Trajectory Optimization](08_ceres_and_trajectory_optimization.md)