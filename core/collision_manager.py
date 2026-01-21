"""
Collision management abstraction.

This module isolates collision checking from:
- Planning
- State spaces
- Timing

This mirrors real systems where collision checking is a service,
not a planner responsibility.
"""

from abc import ABC, abstractmethod
from typing import List, Optional
from core.robot_model import RobotModel

import numpy as np


class CollisionShape(ABC):
    """Abstract base class for collision shapes."""

    @abstractmethod
    def check_point(self, point: np.ndarray) -> bool:
        """Check if a point is inside the shape."""
        pass

    @abstractmethod
    def get_type(self) -> str:
        """Return the type of shape."""
        pass


class Box(CollisionShape):
    """Axis-aligned box collision shape."""

    def __init__(self, center: np.ndarray, size: np.ndarray):
        """
        Initialize box.

        Args:
            center: Center position (3,)
            size: Size along each axis (3,)
        """
        self.center = np.array(center)
        self.size = np.array(size)
        self.min_corner = self.center - self.size / 2.0
        self.max_corner = self.center + self.size / 2.0

    def check_point(self, point: np.ndarray) -> bool:
        """Check if point is inside box."""
        return bool(np.all(point >= self.min_corner) and np.all(point <= self.max_corner))

    def get_type(self) -> str:
        return "box"


class Sphere(CollisionShape):
    """Sphere collision shape."""

    def __init__(self, center: np.ndarray, radius: float):
        """
        Initialize sphere.

        Args:
            center: Center position (3,)
            radius: Sphere radius
        """
        self.center = np.array(center)
        self.radius = radius

    def check_point(self, point: np.ndarray) -> bool:
        """Check if point is inside sphere."""
        return bool(np.linalg.norm(point - self.center) <= self.radius)

    def get_type(self) -> str:
        return "sphere"


class Cylinder(CollisionShape):
    """Cylinder collision shape (axis-aligned along Z)."""

    def __init__(self, center: np.ndarray, radius: float, height: float):
        """
        Initialize cylinder.

        Args:
            center: Center position (3,)
            radius: Cylinder radius
            height: Cylinder height (along Z axis)
        """
        self.center = np.array(center)
        self.radius = radius
        self.height = height
        self.z_min = self.center[2] - height / 2.0
        self.z_max = self.center[2] + height / 2.0

    def check_point(self, point: np.ndarray) -> bool:
        """Check if point is inside cylinder."""
        if not (self.z_min <= point[2] <= self.z_max):
            return False
        radial_dist = np.sqrt((point[0] - self.center[0])**2 + (point[1] - self.center[1])**2)
        return radial_dist <= self.radius

    def get_type(self) -> str:
        return "cylinder"


class CollisionManager:
    """
    Abstract collision manager.

    The robot model delegates collision queries here.
    """

    def in_collision(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        raise NotImplementedError('Collision manager must implement in_collision method')


class NullCollisionManager(CollisionManager):
    """
    Collision-free world.

    Useful for:
    - Early planner development
    - Algorithm validation
    - Educational demos
    """

    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision."""
        return False


class ShapeCollisionManager(CollisionManager):
    """
    Collision manager with geometric shapes.

    Checks robot links against collision shapes using forward kinematics.
    """

    def __init__(self, robot_model: RobotModel, shapes: Optional[List[CollisionShape]] = None):
        """
        Initialize shape collision manager.

        Args:
            robot_model: Robot model with FK capability
            shapes: List of collision shapes
        """
        self.robot = robot_model
        self.shapes: List[CollisionShape] = shapes if shapes is not None else []
        self.collision_log = []

    def add_shape(self, shape: CollisionShape):
        """Add a collision shape to the scene."""
        self.shapes.append(shape)

    def clear_shapes(self):
        """Remove all collision shapes."""
        self.shapes.clear()
        self.collision_log.clear()

    def in_collision(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        if not self.shapes:
            return False

        # Get end-effector position via FK
        try:
            pose = self.robot.fk(q)
            if pose is None:
                return False

            # Extract position (assuming pose is 4x4 matrix or has translation)
            if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
                ee_position = pose[:3, 3]
            elif isinstance(pose, dict):
                ee_position = np.array(pose['translation'])
            else:
                return False

            # Check against all shapes
            for shape in self.shapes:
                if shape.check_point(ee_position):
                    self.collision_log.append({
                        'config': q.copy(),
                        'ee_position': ee_position.copy(),
                        'shape_type': shape.get_type(),
                    })
                    return True

        except Exception:
            # If FK fails, assume collision to be safe
            return True

        return False

    def get_collision_log(self) -> List[dict]:
        """Get log of all detected collisions."""
        return self.collision_log.copy()

    def clear_log(self):
        """Clear the collision log."""
        self.collision_log.clear()


class SimpleSelfCollisionManager(CollisionManager):
    """
    Placeholder for self-collision logic.

    This will later be replaced by FCL-based checking.
    """

    def __init__(self, robot_model: RobotModel):
        """Initialize the simple self-collision manager."""
        self.robot = robot_model

    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision."""
        return False
