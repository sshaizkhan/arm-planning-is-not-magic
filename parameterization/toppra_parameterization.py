from typing import List, Tuple

import numpy as np

try:
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
except ImportError as e:
    raise ImportError(
        "TOPP-RA is required. Install via: pip install toppra"
    ) from e


class ToppraTimeParameterizer:
    """
    TOPP-RA based time parameterizer with robust solver fallback.

    TOPP-RA (Time-Optimal Path Parameterization via Reachability Analysis)
    computes the time-optimal trajectory along a fixed geometric path while
    respecting joint velocity and acceleration constraints.

    The ``n_samples`` parameter controls how many time steps are used when
    sampling the continuous trajectory back into a discrete array.  A higher
    value produces a smoother visualization and finer playback, but does **not**
    change the underlying physics: the duration and constraint satisfaction are
    determined entirely by the TOPP-RA solver.  For real-time control you
    typically want ``n_samples`` to match your controller rate; for plotting
    or visualization, 100–500 samples is usually sufficient.
    """

    def __init__(self, v_max: np.ndarray, a_max: np.ndarray, n_samples: int = 100):
        assert v_max.shape == a_max.shape, "v_max and a_max must have same shape"
        if np.any(v_max <= 0) or np.any(a_max <= 0):
            raise ValueError("All velocity and acceleration limits must be > 0")
        self.v_max = v_max
        self.a_max = a_max
        self.n_samples = n_samples

    def compute(self, path: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        if len(path) < 2:
            raise ValueError("Path must contain at least two waypoints")

        waypoints = np.vstack(path)
        if np.any(np.all(np.diff(waypoints, axis=0) == 0, axis=1)):
            raise ValueError("Path contains duplicate consecutive waypoints")

        # Spline interpolation
        path_spline = ta.SplineInterpolator(
            np.linspace(0, 1, waypoints.shape[0]),
            waypoints,
        )

        # Constraints
        v_constraint = constraint.JointVelocityConstraint(self.v_max)
        a_constraint = constraint.JointAccelerationConstraint(self.a_max)

        # Try seidel first, fallback to qpoases
        for solver in ["seidel", "qpoases"]:
            try:
                instance = algo.TOPPRA([v_constraint, a_constraint], path_spline, solver_wrapper=solver)
                result = instance.compute_trajectory()
                if result is not None:
                    break
            except Exception as e:
                print(f"TOPP-RA solver {solver} failed: {e}")
                result = None

        if result is None:
            raise RuntimeError(
                "TOPP-RA failed to compute trajectory. "
                "Check your path and velocity/acceleration limits."
            )

        # Sample trajectory uniformly in time
        duration = result.get_duration()  # type: ignore
        time_stamps = np.linspace(0, duration, self.n_samples)
        trajectory = np.zeros((self.n_samples, waypoints.shape[1]))
        for i, t in enumerate(time_stamps):
            trajectory[i, :] = result.eval(t)  # type: ignore

        return time_stamps, trajectory
