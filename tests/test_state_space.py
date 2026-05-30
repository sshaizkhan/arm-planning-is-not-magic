"""
Tests for JointStateSpace (core/state_space.py).

JointStateSpace is the C-space layer that sits between the robot model and
the planner.  It answers three questions:
  1. Is a configuration valid? (within limits AND not in collision)
  2. How far apart are two configurations?
  3. How do we move between them? (interpolate / discretize)

Planners only talk to JointStateSpace — they don't touch the robot directly.
"""

import numpy as np
import pytest

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import NullCollisionManager


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def robot():
    """UR5 with NullCollisionManager — every configuration is collision-free."""
    null_cm = NullCollisionManager()
    return UR5RobotModel(use_opw=True, collision_manager=null_cm)


@pytest.fixture
def space(robot):
    """JointStateSpace built around the UR5."""
    return JointStateSpace(robot)


@pytest.fixture
def q1():
    return np.zeros(6)


@pytest.fixture
def q2():
    return np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])


# ---------------------------------------------------------------------------
# Dimension
# ---------------------------------------------------------------------------

def test_dim(space, robot):
    """State space dimension must equal the robot's DOF.

    dim is the number of independent joint variables — the dimension of C-space.
    """
    assert space.dim == robot.dof()


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------

def test_sample_within_limits(space):
    """Every uniform sample must lie inside the joint limits.

    If even one sample violates limits, the sampler is broken and the planner
    will generate invalid states.
    """
    lower, upper = space.robot.joint_limits()
    for _ in range(100):
        q = space.sample_uniform()
        assert np.all(q >= lower), "Sample below lower joint limit"
        assert np.all(q <= upper), "Sample above upper joint limit"


# ---------------------------------------------------------------------------
# Validity
# ---------------------------------------------------------------------------

def test_within_limits_valid(space):
    """The zero configuration must be within joint limits for UR5."""
    q_zero = np.zeros(6)
    assert space.within_limits(q_zero) is True


def test_within_limits_violation(space):
    """A configuration that exceeds joint limits must fail the limits check."""
    # Put joint 0 way outside any reasonable limit
    q_bad = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert space.within_limits(q_bad) is False


def test_is_valid_no_collision(space):
    """With NullCollisionManager, the zero config is fully valid.

    is_valid combines limits check AND collision check.  Since NullCollisionManager
    never reports a collision, any in-limits config should pass.
    """
    q_zero = np.zeros(6)
    assert space.is_valid(q_zero) is True


def test_is_valid_outside_limits(space):
    """A configuration outside joint limits is invalid regardless of collision.

    Limits are checked first; no FK or collision query is needed if limits fail.
    """
    q_bad = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert space.is_valid(q_bad) is False


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------

def test_interpolate_endpoints(space, q1, q2):
    """interp(q1, q2, 0) must equal q1; interp(q1, q2, 1) must equal q2.

    Alpha=0 means 'stay at start'; alpha=1 means 'reach goal'.
    """
    assert np.allclose(space.interpolate(q1, q2, 0.0), q1)
    assert np.allclose(space.interpolate(q1, q2, 1.0), q2)


def test_interpolate_midpoint(space, q1, q2):
    """interp(q1, q2, 0.5) must be exactly halfway between q1 and q2.

    Linear interpolation in joint space: (q1 + q2) / 2 at alpha=0.5.
    """
    mid = space.interpolate(q1, q2, 0.5)
    expected = (q1 + q2) / 2.0
    assert np.allclose(mid, expected)


# ---------------------------------------------------------------------------
# Distance
# ---------------------------------------------------------------------------

def test_distance_zero(space, q1):
    """Distance from a configuration to itself must be 0."""
    assert space.distance(q1, q1) == 0.0


# ---------------------------------------------------------------------------
# Discretization
# ---------------------------------------------------------------------------

def test_discretize_includes_endpoints(space, q1, q2):
    """Discretized path must include both q1 and q2 as the first and last points.

    This ensures the collision checker always tests the exact start and goal.
    """
    step_size = 0.1
    path = space.discretize(q1, q2, step_size)
    assert len(path) >= 2
    assert np.allclose(path[0], q1), "First point must be q1"
    assert np.allclose(path[-1], q2), "Last point must be q2"


def test_discretize_step_size(space, q1, q2):
    """Consecutive samples in the discretized path must be within step_size of each other.

    If any step is larger than step_size, we might skip over a narrow obstacle.
    """
    step_size = 0.1
    path = space.discretize(q1, q2, step_size)
    for i in range(len(path) - 1):
        d = space.distance(path[i], path[i + 1])
        assert d <= step_size + 1e-9, \
            f"Step {i}->{i+1} has distance {d:.4f} > step_size {step_size}"
