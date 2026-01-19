# Controllers and Execution

> Planning ends when physics begins.

This document explains **how trajectories are actually executed**, why controllers matter more than planners, and why most failures happen *after* planning succeeds.

---

## 1. Planning vs Execution (The Hard Truth)

A planner outputs:

* Joint positions
* Possibly velocities and accelerations

A robot executes:

* Motor torques
* Under noise, delay, and friction

> A perfect trajectory does not guarantee perfect motion.

---

## 2. Types of Controllers

### Position Control

* Commands joint positions
* Controller handles everything else
* Most common in industrial arms

Pros:

* Simple
* Robust

Cons:

* Limited dynamic awareness

---

### Velocity Control

* Commands joint velocities
* Requires smooth trajectories

Used when:

* Online replanning
* Reactive systems

---

### Torque Control

* Commands torques directly
* Full dynamics required

Pros:

* Most expressive

Cons:

* Hard to tune
* Unsafe without care

---

## 3. Why Smoothness Matters

Controllers assume:

* Bounded acceleration
* Bounded jerk

Violations cause:

* Tracking error
* Vibrations
* Hardware wear

This is why:

* Ruckig enforces jerk limits
* Ceres is used for smoothing

---

## 4. Feedforward vs Feedback

### Feedback

* Corrects error
* Always present

### Feedforward

* Uses planned velocity/acceleration
* Reduces error proactively

Good execution requires both.

---

## 5. Why Controllers Fail Even With Valid Plans

Common reasons:

* Discrete waypoints too sparse
* Sudden velocity changes
* Latency between planner and controller
* Mismatch between model and hardware

Planning must respect **controller assumptions**.

---

## 6. Execution in Real Systems

### MoveIt

* Sends trajectories to `ros_control`
* Controller interpolates between points

### Industrial Controllers

* Expect time-parameterized splines
* Reject discontinuities

---

## 7. Mental Model

Think of execution as:

> Driving a car with suspension.

The road (trajectory) must be smooth enough for the suspension (controller) to handle.

---

## 8. Takeaway

* Planners create intent
* Controllers face reality
* Smoothness beats optimality

If execution fails, look *downstream*, not upstream.

---

## 9. What Comes Next

With theory complete, we now move to **code**.

Next phases:

* `core/` → math and primitives
* `planners/` → OMPL-style algorithms
* `timing/` → TOPP-RA and Ruckig
* `demos/` → experiments

