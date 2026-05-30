# Path Smoothing (Why Raw Planner Output Is Not Enough)

> Sampling-based planners find *a* path.
> Path smoothing finds *a better* path — without ever leaving C_free.

This document explains why RRT paths are jagged, how shortcutting and spline fitting fix this, and why the tradeoffs matter before you hand a path to a time-parameterizer.

---

## 1. Why Raw RRT Paths Are Jagged

RRT grows a tree by:

1. Sampling a random configuration
2. Extending from the nearest node by a fixed step size

This step-size-driven extension creates two problems:

**Voronoi bias** — the tree expands toward large unexplored regions, not toward a direct goal. Branches zig-zag as the tree fills space rather than driving straight toward the goal.

**Unnecessary waypoints** — when RRT finally reaches the goal, the path is a sequence of fixed-step hops. Many intermediate nodes exist only because the step size forced them, not because they encode any meaningful geometry.

The result: a jagged, indirect path with many more waypoints than the geometry requires.

---

## 2. The Shortcutting Algorithm

Shortcutting removes redundant waypoints by testing whether intermediate nodes can be bypassed entirely.

### Pseudocode

```
function shortcut_smoothing(path, collision_check, max_iterations):
    for _ in range(max_iterations):
        if len(path) <= 2:
            break
        i = random integer in [0, len(path) - 2)
        j = random integer in (i+1, len(path))
        if is_collision_free_segment(path[i], path[j], collision_check):
            path = path[:i+1] + path[j:]   # remove all nodes between i and j
    return path

function is_collision_free_segment(q1, q2, collision_check):
    n_steps = ceil(dist(q1, q2) / step_size)
    for t in linspace(0, 1, n_steps + 1):
        q = q1 + t * (q2 - q1)
        if collision_check(q):
            return False
    return True
```

The key property: we only remove nodes when the straight-line segment in C-space is verified collision-free at the chosen resolution. **We never leave C_free.**

See the implementation in `core/path_smoothing.py` → `shortcut_smoothing()`.

---

## 3. Why Shortcutting Alone Is Not Enough

After shortcutting, you have fewer waypoints — but the path is still **piecewise-linear**. Every retained waypoint is a corner: a sharp change in direction in joint space.

These corners matter because:

* The **velocity at a corner is mathematically zero** if you want to respect acceleration limits — the robot must stop, reverse direction, restart.
* In practice, controllers either ignore corners (causing constraint violations) or interpolate through them (creating trajectories that don't follow the planned geometry).

Shortcutting reduces waypoint count; it does not remove the angular nature of the path.

---

## 4. Spline Fitting

Spline fitting converts the piecewise-linear path into a **smooth curve** that passes through (or near) the waypoints.

In `core/path_smoothing.py`, `spline_smoothing()` uses `scipy.interpolate` B-splines per joint axis. The output is a dense, smooth array of configurations that can be re-sampled at any resolution.

### Why This Matters for Time Parameterization

TOPP-RA computes trajectory timing from path derivatives:

$$\dot{q} = q'(s)\,\dot{s}$$
$$\ddot{q} = q''(s)\,\dot{s}^2 + q'(s)\,\ddot{s}$$

If `q(s)` has sharp corners, `q'(s)` is discontinuous and `q''(s)` is undefined at those corners. The TOPP-RA solver degenerates: it either fails, produces artificially low speeds through corners, or outputs trajectories with extreme acceleration demands.

A smooth spline gives TOPP-RA well-defined first and second derivatives everywhere, so it can compute genuinely time-optimal timing without fighting numerical ill-conditioning.

**Sharp corners → infinite theoretical acceleration demand → TOPP-RA forced to slow down or fail.**

---

## 5. The Tradeoff: Smoother Paths Are Not Always Shorter

Spline fitting pulls the path away from the waypoints. A spline through a tight set of RRT waypoints may:

* Cut corners and shorten the path (good)
* Overshoot into regions that were not checked for collisions (dangerous)
* Create a longer arc when the original waypoints already formed a near-direct line (wasteful)

This is why `path_smoothing.py` includes `_fix_collisions()`: after spline fitting, every point on the smoothed path is re-checked. If the spline introduced collisions, the code falls back to a linear interpolation of the original path at the same density.

Post-processing can create curves where a direct path existed. Smoothing is not free.

---

## 6. Key Insight: Path Smoothing Never Replans

Shortcutting and spline fitting operate **only within C_free**. Every segment tested by `is_collision_free_segment()` is verified before accepting the shortcut. The spline output is re-validated before use.

This means:

* Smoothing cannot find a path that the planner missed.
* Smoothing cannot escape a local minimum in C_free.
* Smoothing cannot fix a topologically wrong path (e.g., a path that goes around the wrong side of an obstacle cannot be smoothed into one that goes the right way).

Path smoothing is geometry refinement. **It is not replanning.**

---

## 7. What Comes Next

A smooth, shortcut path is ready for time parameterization. The sharp-corner problem is gone, TOPP-RA gets clean derivatives, and the resulting trajectory respects velocity and acceleration limits with minimal artificial slowdowns.

Continue with:

[Path vs Trajectory](05_path_vs_trajectory.md)

---

## Exercises

1. **Shortcutting convergence.** Run `shortcut_smoothing()` on a 20-waypoint path with `max_iterations=10` versus `max_iterations=500`. How does waypoint count change? Is there a point of diminishing returns? Why does it depend on the obstacle geometry rather than just the iteration count?

2. **Spline overshoot.** In `core/path_smoothing.py`, `spline_smoothing()` uses `smoothing_factor=0.0` by default, meaning the spline interpolates exactly through every waypoint. Increase `smoothing_factor` to `0.5` for a 5-waypoint path. What happens to path length? What happens to the maximum deviation from the original waypoints? Under what conditions does a nonzero smoothing factor make sense?

3. **Acceleration demand at corners.** Take a path with a 90-degree turn in joint space at a single waypoint. Compute the path-parameter second derivative `q''(s)` analytically at that corner for a piecewise-linear parameterization. What does this imply for the acceleration constraint in TOPP-RA? Now apply spline smoothing and repeat. What changed?

4. **Collision after spline.** Construct a scenario (by hand or in a test) where `spline_smoothing()` produces a path that passes through `C_obs`. Verify that `_fix_collisions()` detects it and falls back. Now explain: if the fallback linear path has the same waypoints as the pre-smoothing input, why might the fallback still be jagged?
