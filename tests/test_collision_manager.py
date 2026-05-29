"""
Tests for collision shapes and managers (core/collision_manager.py).

Collision checking in this repo is separated from planning:
  - CollisionShape (Box, Sphere, Cylinder) — geometric primitives
  - CollisionManager — delegates collision queries to shapes
  - NullCollisionManager — always returns False (open world)
  - ShapeCollisionManager — checks ALL link positions against all shapes

The key architectural insight: ShapeCollisionManager checks every link frame
(not just the EE), so the robot body is fully accounted for.
"""

import numpy as np
import pytest

from core.collision_manager import (
    Box,
    Sphere,
    Cylinder,
    NullCollisionManager,
    ShapeCollisionManager,
)
from core.robot_model import UR5RobotModel


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def robot():
    """UR5 without a collision manager (FK only, no collision)."""
    return UR5RobotModel(use_opw=True, collision_manager=None)


# ---------------------------------------------------------------------------
# Box
# ---------------------------------------------------------------------------

def test_box_contains_center():
    """A point at the center of a box must be inside it.

    The center is the most obviously interior point.
    """
    box = Box(center=np.array([1.0, 2.0, 3.0]), size=np.array([1.0, 1.0, 1.0]))
    assert box.check_point(np.array([1.0, 2.0, 3.0]))


def test_box_excludes_far_point():
    """A point 10 m away from the box center must not be inside it."""
    box = Box(center=np.array([0.0, 0.0, 0.0]), size=np.array([1.0, 1.0, 1.0]))
    assert not box.check_point(np.array([10.0, 0.0, 0.0]))


# ---------------------------------------------------------------------------
# Sphere
# ---------------------------------------------------------------------------

def test_sphere_contains_center():
    """A point at the sphere center must be inside (distance = 0 <= radius)."""
    sphere = Sphere(center=np.array([0.0, 0.0, 0.5]), radius=0.3)
    assert sphere.check_point(np.array([0.0, 0.0, 0.5]))


def test_sphere_boundary():
    """A point exactly at the radius boundary is inside; just outside is not.

    The check uses <=, so the surface itself counts as interior (solid sphere).
    """
    center = np.array([0.0, 0.0, 0.0])
    radius = 0.5
    sphere = Sphere(center=center, radius=radius)

    # Exactly at the boundary: distance == radius, so check_point returns True
    on_boundary = np.array([radius, 0.0, 0.0])
    assert sphere.check_point(on_boundary), "Surface point should be inside"

    # Just outside: distance slightly > radius
    just_outside = np.array([radius + 1e-4, 0.0, 0.0])
    assert not sphere.check_point(just_outside), "Point just outside should not be inside"


# ---------------------------------------------------------------------------
# Cylinder
# ---------------------------------------------------------------------------

def test_cylinder_contains_center():
    """A point at the cylinder center must be inside it."""
    cyl = Cylinder(center=np.array([0.0, 0.0, 0.0]), radius=0.3, height=1.0)
    assert cyl.check_point(np.array([0.0, 0.0, 0.0]))


def test_cylinder_excludes_outside_radius():
    """A point beyond the cylinder's radial boundary must not be inside."""
    cyl = Cylinder(center=np.array([0.0, 0.0, 0.0]), radius=0.3, height=1.0)
    # At the right Z but too far in X
    assert not cyl.check_point(np.array([1.0, 0.0, 0.0]))


def test_cylinder_excludes_outside_height():
    """A point above z_max must not be inside the cylinder.

    The cylinder is axis-aligned along Z, so height bounds are the z_min/z_max
    of the cylinder centered at `center`.
    """
    cyl = Cylinder(center=np.array([0.0, 0.0, 0.0]), radius=0.3, height=1.0)
    # z_max = 0.0 + 0.5 = 0.5, so z=1.0 is outside
    assert not cyl.check_point(np.array([0.0, 0.0, 1.0]))


# ---------------------------------------------------------------------------
# NullCollisionManager
# ---------------------------------------------------------------------------

def test_null_collision_manager_always_false(robot):
    """NullCollisionManager must always report no collision.

    This manager is used when there are no obstacles in the scene.
    Any configuration — including extreme ones — must return False.
    """
    null_cm = NullCollisionManager()
    q_zero = np.zeros(6)
    q_extreme = np.array([2.9, -1.5, 2.5, -3.0, 2.5, 3.0])
    assert not null_cm.in_collision(q_zero)
    assert not null_cm.in_collision(q_extreme)


# ---------------------------------------------------------------------------
# ShapeCollisionManager
# ---------------------------------------------------------------------------

def test_shape_collision_manager_no_shapes(robot):
    """A ShapeCollisionManager with no shapes must report no collision.

    With an empty scene, every configuration is collision-free — same as Null.
    """
    cm = ShapeCollisionManager(robot_model=robot)
    assert not cm.in_collision(np.zeros(6))


def test_shape_collision_manager_detects_link_collision(robot):
    """Placing a sphere at the UR5 EE position must trigger a collision.

    Strategy:
      1. Use FK to find where the EE is for q_test.
      2. Place a sphere centered exactly there.
      3. The ShapeCollisionManager checks ALL link positions, so when the EE
         link position falls inside the sphere, in_collision returns True.
    """
    q_test = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
    T = robot.fk(q_test)
    ee_pos = T[:3, 3]

    # Sphere centered at EE with generous radius to avoid float edge cases
    sphere = Sphere(center=ee_pos, radius=0.05)
    cm = ShapeCollisionManager(robot_model=robot, shapes=[sphere])
    robot.set_collision_manager(cm)

    assert cm.in_collision(q_test), \
        "Expected collision when sphere is placed at EE link position"


def test_collision_log_populated(robot):
    """After a collision is detected, the collision log must be non-empty.

    The log stores metadata about each collision event (config, link, shape type),
    which is useful for debugging why a path was rejected.
    """
    q_test = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
    T = robot.fk(q_test)
    ee_pos = T[:3, 3]

    sphere = Sphere(center=ee_pos, radius=0.05)
    cm = ShapeCollisionManager(robot_model=robot, shapes=[sphere])

    cm.in_collision(q_test)  # trigger the collision

    log = cm.get_collision_log()
    assert len(log) > 0, "Collision log should be populated after a collision"


def test_collision_log_cleared(robot):
    """After clear_log(), the collision log must be empty.

    Logs should be clearable between planning attempts so they don't
    accumulate across unrelated queries.
    """
    q_test = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
    T = robot.fk(q_test)
    ee_pos = T[:3, 3]

    sphere = Sphere(center=ee_pos, radius=0.05)
    cm = ShapeCollisionManager(robot_model=robot, shapes=[sphere])

    cm.in_collision(q_test)  # populate the log
    cm.clear_log()

    assert cm.get_collision_log() == [], "Log should be empty after clear_log()"
