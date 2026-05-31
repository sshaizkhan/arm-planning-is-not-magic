"""
Tests for the UR5 robot model (core/robot_model.py).

The RobotModel ABC defines the minimal interface a motion planner needs:
  - dof()             — how many joints?
  - joint_limits()    — what are the valid ranges?
  - fk(q)             — where is the end-effector?
  - distance(q1, q2)  — how far apart are two configs in C-space?
  - link_positions(q) — where are all the links (for collision checking)?
  - in_collision(q)   — is this configuration in collision?
  - name              — human-readable identifier

These tests verify that UR5RobotModel satisfies the contract correctly.
"""

import numpy as np
import pytest

from core.robot_model import UR5RobotModel


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def robot():
    """Default UR5 with OPW kinematics and no collision manager."""
    return UR5RobotModel(use_opw=True, collision_manager=None)


@pytest.fixture
def q_zero():
    """All-zeros joint configuration — the 'home' pose."""
    return np.zeros(6)


@pytest.fixture
def q_test():
    """A non-trivial configuration well within joint limits."""
    return np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])


# ---------------------------------------------------------------------------
# DOF and limits
# ---------------------------------------------------------------------------

def test_dof(robot):
    """UR5 is a 6-DOF robot; dof() must return 6."""
    assert robot.dof() == 6


def test_joint_limits_shape(robot):
    """joint_limits() returns (lower, upper); each must be a length-6 vector."""
    lower, upper = robot.joint_limits()
    assert lower.shape == (6,), "lower limits must have shape (6,)"
    assert upper.shape == (6,), "upper limits must have shape (6,)"


def test_joint_limits_ordering(robot):
    """Every joint's lower limit must be strictly less than its upper limit."""
    lower, upper = robot.joint_limits()
    assert np.all(lower < upper), "lower limits must be less than upper limits for every joint"


def test_name(robot):
    """name property must return a non-empty string that identifies the robot."""
    assert isinstance(robot.name, str)
    assert len(robot.name) > 0


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def test_fk_home_position(robot, q_zero):
    """FK at the zero configuration must return a valid 4x4 homogeneous matrix."""
    T = robot.fk(q_zero)
    assert T.shape == (4, 4), "FK result must be a 4x4 transformation matrix"


def test_fk_returns_valid_rotation(robot, q_test):
    """The top-left 3x3 block of FK must be a proper rotation matrix (R @ R.T = I)."""
    T = robot.fk(q_test)
    R = T[:3, :3]
    # A valid rotation matrix satisfies R @ R.T == I (up to floating-point error)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6), \
        "Rotation part of FK is not orthogonal — FK may be incorrect"


# ---------------------------------------------------------------------------
# Distance metric
# ---------------------------------------------------------------------------

def test_distance_zero(robot, q_test):
    """Distance from a configuration to itself must be exactly 0."""
    assert robot.distance(q_test, q_test) == 0.0


def test_distance_symmetric(robot, q_zero, q_test):
    """Distance must be symmetric: d(q1, q2) == d(q2, q1)."""
    assert np.isclose(robot.distance(q_zero, q_test), robot.distance(q_test, q_zero))


def test_distance_triangle_inequality(robot):
    """Distance must satisfy the triangle inequality: d(a, c) <= d(a, b) + d(b, c)."""
    a = np.zeros(6)
    b = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
    c = np.array([0.3, -0.3, 0.1, -0.4, 0.1, 0.0])
    assert robot.distance(a, c) <= robot.distance(a, b) + robot.distance(b, c) + 1e-9


# ---------------------------------------------------------------------------
# Link positions
# ---------------------------------------------------------------------------

def test_link_positions_shape(robot, q_zero):
    """link_positions(q) must return a (5, 3) array — 5 key frames in 3D space.

    The UR5 OPW model tracks: base, shoulder, elbow, wrist, and end-effector.
    Having all of them (not just the EE) lets the collision manager check the
    full robot body.
    """
    positions = robot.link_positions(q_zero)
    assert positions.shape == (5, 3), \
        f"Expected (5, 3) link positions, got {positions.shape}"


def test_link_positions_base_at_origin(robot, q_zero):
    """The base link position must be at the world origin [0, 0, 0]."""
    positions = robot.link_positions(q_zero)
    assert np.allclose(positions[0], [0.0, 0.0, 0.0], atol=1e-6), \
        "Base link should be at the origin"


def test_link_positions_ee_matches_fk(robot, q_test):
    """The last link position must match the FK end-effector translation.

    link_positions and fk must agree on where the end-effector is —
    they use the same OPW kinematics under the hood.
    """
    positions = robot.link_positions(q_test)
    T = robot.fk(q_test)
    ee_from_fk = T[:3, 3]
    assert np.allclose(positions[-1], ee_from_fk, atol=1e-4), \
        "Last link position does not match FK end-effector position"


# ---------------------------------------------------------------------------
# Collision
# ---------------------------------------------------------------------------

def test_in_collision_no_manager(robot, q_zero):
    """A robot with no collision manager must report False for any configuration.

    This is the 'open world' assumption: if you haven't told the robot about
    any obstacles, it assumes it's collision-free.
    """
    # robot fixture has collision_manager=None
    assert robot.in_collision(q_zero) is False


# ---------------------------------------------------------------------------
# URDFKinematics link_transforms
# ---------------------------------------------------------------------------

from core.kinematics.urdf_kinematics import URDFKinematics


@pytest.fixture
def urdf_kin():
    return URDFKinematics()


def test_link_transforms_returns_all_links(urdf_kin):
    q = np.zeros(6)
    transforms = urdf_kin.link_transforms(q)
    expected_links = [
        'base_link', 'shoulder_link', 'upper_arm_link',
        'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link',
    ]
    for name in expected_links:
        assert name in transforms, f"Missing link: {name}"
        T = transforms[name]
        assert T.shape == (4, 4), f"{name} transform is not 4x4"
        assert np.allclose(T[3], [0, 0, 0, 1]), f"{name} last row not [0,0,0,1]"


def test_link_transforms_base_is_identity(urdf_kin):
    transforms = urdf_kin.link_transforms(np.zeros(6))
    assert np.allclose(transforms['base_link'], np.eye(4))


def test_link_transforms_ee_matches_forward_kinematics(urdf_kin):
    """EE transform from link_transforms must match forward_kinematics."""
    q = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.1])
    transforms = urdf_kin.link_transforms(q)
    T_fk = urdf_kin.forward_kinematics(q)
    assert np.allclose(transforms['ee_link'], T_fk, atol=1e-10)
