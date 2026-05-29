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

## Exercises

1. **Controller mode mismatch.** A position-control interface receives a trajectory with 500 waypoints evenly spaced in time at 1 ms apart. A velocity-control interface receives the same trajectory. Describe what each controller does between waypoints. Which interface is more sensitive to gaps in the trajectory, and why? What happens to a velocity-controlled robot if the trajectory publisher stops sending commands mid-motion?

2. **Feedforward contribution.** Section 4 describes feedforward as using planned velocity/acceleration to reduce tracking error proactively. Write a one-paragraph explanation of why feedforward helps specifically at the start of a fast motion segment — where feedback (which reacts to error) is least helpful. What would you observe in the joint position error plot if feedforward were disabled on a trajectory with a high initial acceleration?

3. **Sparse waypoints and interpolation artifacts.** Section 5 lists "discrete waypoints too sparse" as a failure mode. Suppose a controller linearly interpolates between waypoints 100 ms apart, but the planned trajectory has a smooth curve that deviates significantly from linear over that interval. Estimate (geometrically) the maximum position error this introduces as a function of path curvature and waypoint spacing. What waypoint spacing would keep this error below 1 mm for a typical industrial arm move?

---

## 9. What Comes Next

With theory complete, we now move to **code**.

Next phases:

* `core/` → math and primitives
* `planners/` → OMPL-style algorithms
* `timing/` → TOPP-RA and Ruckig
* `demos/` → experiments

