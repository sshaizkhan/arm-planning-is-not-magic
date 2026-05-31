"""
Tests for time parameterization (parameterization/).

A motion planner produces a geometric path: an ordered list of joint
configurations with no timing information.  Time parameterization adds a
time dimension — computing how fast to move along the path while respecting
joint velocity and acceleration limits.

Two parameterizers are tested here:
  - ToppraTimeParameterizer  — time-optimal (TOPP-RA algorithm)
  - RuckigTimeParameterizer  — jerk-limited (Ruckig algorithm)

Both accept a path and limits; both return (time_stamps, trajectory).
"""

import numpy as np
import pytest

try:
    from parameterization.toppra_parameterization import ToppraTimeParameterizer
    TOPPRA_AVAILABLE = True
except ImportError:
    TOPPRA_AVAILABLE = False

try:
    from parameterization.ruckig_parameterization import RuckigTimeParameterizer
    RUCKIG_AVAILABLE = True
except ImportError:
    RUCKIG_AVAILABLE = False

skip_toppra = pytest.mark.skipif(not TOPPRA_AVAILABLE, reason="toppra not installed")
skip_ruckig = pytest.mark.skipif(not RUCKIG_AVAILABLE, reason="ruckig not installed")


# ---------------------------------------------------------------------------
# Shared test data
# ---------------------------------------------------------------------------

# A simple 3-waypoint path through 6D joint space
PATH = [
    np.zeros(6),
    np.array([0.3, -0.3, 0.2, -0.4, 0.1, 0.0]),
    np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0]),
]

V_MAX = np.ones(6) * 1.5   # rad/s per joint
A_MAX = np.ones(6) * 3.0   # rad/s² per joint
J_MAX = np.ones(6) * 10.0  # rad/s³ per joint (Ruckig only)


# ---------------------------------------------------------------------------
# TOPP-RA tests
# ---------------------------------------------------------------------------

@skip_toppra
def test_toppra_output_shape():
    """compute() must return (time_stamps, trajectory) with consistent shapes.

    time_stamps is 1-D with length n_samples; trajectory is 2-D with shape
    (n_samples, 6) — one row per time step, one column per joint.
    """
    n = 100
    tp = ToppraTimeParameterizer(V_MAX, A_MAX, n_samples=n)
    time_stamps, trajectory = tp.compute(PATH)

    assert time_stamps.ndim == 1, "time_stamps should be 1-D"
    assert time_stamps.shape[0] == n, f"Expected {n} time stamps, got {time_stamps.shape[0]}"
    assert trajectory.shape == (n, 6), f"Expected trajectory shape ({n}, 6), got {trajectory.shape}"


@skip_toppra
def test_toppra_duration_positive():
    """The trajectory duration must be strictly positive.

    A zero-duration trajectory would mean the robot teleports, which violates
    the velocity and acceleration constraints.
    """
    tp = ToppraTimeParameterizer(V_MAX, A_MAX, n_samples=100)
    time_stamps, _ = tp.compute(PATH)
    duration = time_stamps[-1] - time_stamps[0]
    assert duration > 0.0, f"Trajectory duration must be > 0, got {duration}"


@skip_toppra
def test_toppra_start_end_match():
    """The trajectory must begin at path[0] and end at path[-1].

    The parameterizer must not add or remove waypoints — it only adds timing.
    """
    tp = ToppraTimeParameterizer(V_MAX, A_MAX, n_samples=100)
    time_stamps, trajectory = tp.compute(PATH)

    assert np.allclose(trajectory[0], PATH[0], atol=1e-3), \
        "Trajectory start does not match path start"
    assert np.allclose(trajectory[-1], PATH[-1], atol=1e-3), \
        "Trajectory end does not match path end"


@skip_toppra
def test_toppra_n_samples():
    """Setting n_samples=50 must produce exactly 50 time stamps and trajectory rows.

    n_samples controls the temporal resolution of the output — it does not
    change the duration or the physics.
    """
    n = 50
    tp = ToppraTimeParameterizer(V_MAX, A_MAX, n_samples=n)
    time_stamps, trajectory = tp.compute(PATH)

    assert len(time_stamps) == n, f"Expected {n} samples, got {len(time_stamps)}"
    assert trajectory.shape[0] == n


@skip_toppra
def test_toppra_rejects_short_path():
    """A path with fewer than 2 waypoints must raise ValueError.

    A single waypoint is not a path — there is no segment to parameterize.
    """
    tp = ToppraTimeParameterizer(V_MAX, A_MAX)
    with pytest.raises(ValueError, match="at least two waypoints"):
        tp.compute([np.zeros(6)])


@skip_toppra
def test_toppra_rejects_duplicate_waypoints():
    """A path with two identical consecutive waypoints must raise ValueError.

    Duplicate waypoints create a zero-length segment that cannot be parameterized
    (it implies infinite velocity to advance the spline parameter).
    """
    tp = ToppraTimeParameterizer(V_MAX, A_MAX)
    duplicate_path = [np.zeros(6), np.zeros(6), np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])]
    with pytest.raises(ValueError, match="duplicate"):
        tp.compute(duplicate_path)


# ---------------------------------------------------------------------------
# Ruckig tests
# ---------------------------------------------------------------------------

@skip_ruckig
def test_ruckig_output_shape():
    """compute() must return a 2-D trajectory with one column per DOF.

    Ruckig parameterizes the path from first to last waypoint with jerk limits.
    The number of rows depends on the motion duration and dt, but columns must
    match the robot's DOF (6 for UR5).
    """
    rp = RuckigTimeParameterizer(V_MAX, A_MAX, J_MAX)
    time_stamps, trajectory = rp.compute(PATH)

    assert trajectory.ndim == 2, "trajectory must be 2-D"
    assert trajectory.shape[1] == 6, \
        f"trajectory must have 6 columns (one per joint), got {trajectory.shape[1]}"
    assert len(time_stamps) == trajectory.shape[0], \
        "time_stamps length must match trajectory row count"


@skip_ruckig
def test_ruckig_duration_positive():
    """The Ruckig trajectory must have a strictly positive duration.

    A motion from q_start to q_goal (different configurations) always takes
    nonzero time under finite velocity and acceleration limits.
    """
    rp = RuckigTimeParameterizer(V_MAX, A_MAX, J_MAX)
    time_stamps, _ = rp.compute(PATH)
    duration = time_stamps[-1] - time_stamps[0]
    assert duration > 0.0, f"Ruckig trajectory duration must be > 0, got {duration}"


# ---------------------------------------------------------------------------
# API contract tests — prevent regression on constructor kwargs and return type
# ---------------------------------------------------------------------------

def test_constructor_kwargs():
    """ToppraTimeParameterizer takes v_max and a_max, not velocity_limits/acceleration_limits."""
    import inspect
    from parameterization.toppra_parameterization import ToppraTimeParameterizer
    sig = inspect.signature(ToppraTimeParameterizer.__init__)
    params = list(sig.parameters.keys())
    assert 'v_max' in params, f"Expected v_max param, got: {params}"
    assert 'a_max' in params, f"Expected a_max param, got: {params}"
    assert 'velocity_limits' not in params, "velocity_limits param was renamed to v_max"
    assert 'acceleration_limits' not in params, "acceleration_limits param was renamed to a_max"


@skip_toppra
def test_compute_returns_tuple_of_timestamps_then_trajectory():
    """compute() must return (time_stamps, trajectory) — time_stamps first."""
    import numpy as np
    from parameterization.toppra_parameterization import ToppraTimeParameterizer
    v = np.ones(6) * 2.0
    a = np.ones(6) * 1.5
    param = ToppraTimeParameterizer(v_max=v, a_max=a)
    path = [np.zeros(6), np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])]
    result = param.compute(path)
    assert isinstance(result, tuple) and len(result) == 2, "compute() must return a 2-tuple"
    time_stamps, trajectory = result
    assert time_stamps[0] == pytest.approx(0.0), "time_stamps must start at 0"
    assert time_stamps[-1] > 0, "time_stamps must end > 0"
    assert trajectory.shape[1] == 6, "trajectory must have 6 joints"
    assert time_stamps.ndim == 1, "first return value must be time_stamps (1D)"
    assert trajectory.ndim == 2, "second return value must be trajectory (2D)"


# ---------------------------------------------------------------------------
# API contract tests — prevent regression on constructor kwargs and return type
# ---------------------------------------------------------------------------

def test_constructor_kwargs():
    """ToppraTimeParameterizer takes v_max and a_max, not velocity_limits/acceleration_limits."""
    import inspect
    from parameterization.toppra_parameterization import ToppraTimeParameterizer
    sig = inspect.signature(ToppraTimeParameterizer.__init__)
    params = list(sig.parameters.keys())
    assert 'v_max' in params, f"Expected v_max param, got: {params}"
    assert 'a_max' in params, f"Expected a_max param, got: {params}"
    assert 'velocity_limits' not in params, "velocity_limits param was renamed to v_max"
    assert 'acceleration_limits' not in params, "acceleration_limits param was renamed to a_max"


@skip_toppra
def test_compute_returns_tuple_of_timestamps_then_trajectory():
    """compute() must return (time_stamps, trajectory) — time_stamps first."""
    import numpy as np
    from parameterization.toppra_parameterization import ToppraTimeParameterizer
    v = np.ones(6) * 2.0
    a = np.ones(6) * 1.5
    param = ToppraTimeParameterizer(v_max=v, a_max=a)
    path = [np.zeros(6), np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])]
    result = param.compute(path)
    assert isinstance(result, tuple) and len(result) == 2, "compute() must return a 2-tuple"
    time_stamps, trajectory = result
    assert time_stamps[0] == pytest.approx(0.0), "time_stamps must start at 0"
    assert time_stamps[-1] > 0, "time_stamps must end > 0"
    assert trajectory.shape[1] == 6, "trajectory must have 6 joints"
    assert time_stamps.ndim == 1, "first return value must be time_stamps (1D)"
    assert trajectory.ndim == 2, "second return value must be trajectory (2D)"
