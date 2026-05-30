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

import numpy as np

from core.robot_model import RobotModel


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

    def check_capsule(self, p1: np.ndarray, p2: np.ndarray, radius: float) -> bool:
        """Check if a capsule (segment p1→p2 with given radius) intersects this box."""
        closest_dist_sq = self._segment_box_distance_squared(p1, p2)
        return closest_dist_sq <= radius * radius

    def _segment_box_distance_squared(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """Squared distance from line segment to this axis-aligned box (sampled approximation)."""
        def clamp(val, lo, hi):
            return max(lo, min(val, hi))

        min_dist_sq = float('inf')
        for i in range(11):
            t = i / 10.0
            pt = p1 + t * (p2 - p1)
            closest = np.array([
                clamp(pt[0], self.min_corner[0], self.max_corner[0]),
                clamp(pt[1], self.min_corner[1], self.max_corner[1]),
                clamp(pt[2], self.min_corner[2], self.max_corner[2]),
            ])
            min_dist_sq = min(min_dist_sq, float(np.sum((pt - closest) ** 2)))
        return min_dist_sq

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

    def check_capsule(self, p1: np.ndarray, p2: np.ndarray, capsule_radius: float) -> bool:
        """Check if a capsule intersects this sphere."""
        closest = self._closest_point_on_segment(p1, p2, self.center)
        dist = float(np.linalg.norm(closest - self.center))
        return dist <= (self.radius + capsule_radius)

    def _closest_point_on_segment(self, p1: np.ndarray, p2: np.ndarray, point: np.ndarray) -> np.ndarray:
        """Return the closest point on segment p1→p2 to the given point."""
        seg = p2 - p1
        seg_len_sq = float(np.dot(seg, seg))
        if seg_len_sq < 1e-10:
            return p1.copy()
        t = float(np.dot(point - p1, seg)) / seg_len_sq
        t = max(0.0, min(1.0, t))
        return p1 + t * seg

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

    def check_capsule(self, p1: np.ndarray, p2: np.ndarray, capsule_radius: float) -> bool:
        """Check if a capsule intersects this cylinder (axis-aligned along Z)."""
        combined_radius = self.radius + capsule_radius
        z_lo = self.z_min - capsule_radius
        z_hi = self.z_max + capsule_radius

        for i in range(11):
            t = i / 10.0
            pt = p1 + t * (p2 - p1)
            if not (z_lo <= pt[2] <= z_hi):
                continue
            radial = float(np.sqrt((pt[0] - self.center[0])**2 + (pt[1] - self.center[1])**2))
            if radial <= combined_radius:
                return True
        return False

    def get_type(self) -> str:
        return "cylinder"


class CollisionManager(ABC):
    """
    Abstract collision manager.

    The robot model delegates collision queries here.
    """

    @abstractmethod
    def in_collision(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        pass


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
        # Approximate UR5 link radii (meters) for capsule collision checking
        self._link_radii = [0.065, 0.050, 0.045, 0.035]  # base→shoulder, shoulder→elbow, elbow→wrist, wrist→EE

    def add_shape(self, shape: CollisionShape):
        """Add a collision shape to the scene."""
        self.shapes.append(shape)

    def clear_shapes(self):
        """Remove all collision shapes."""
        self.shapes.clear()
        self.collision_log.clear()

    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision using capsule-based link checking."""
        if not self.shapes:
            return False
        try:
            # Get 5 key link positions: base, shoulder, elbow, wrist, EE
            link_pos = self.robot.link_positions(q)
            # Build capsule segments from consecutive positions with link radii
            segments = [
                (link_pos[i], link_pos[i + 1], self._link_radii[i])
                for i in range(len(self._link_radii))
            ]
            for p1, p2, radius in segments:
                for shape in self.shapes:
                    if hasattr(shape, 'check_capsule'):
                        hit = shape.check_capsule(p1, p2, radius)
                    else:
                        hit = shape.check_point(p1) or shape.check_point(p2)
                    if hit:
                        self.collision_log.append({
                            'config': q.copy(),
                            'segment': (p1.copy(), p2.copy()),
                            'radius': radius,
                            'shape_type': shape.get_type(),
                        })
                        return True
        except Exception:
            return True
        return False

    def get_collision_log(self) -> List[dict]:
        """Get log of all detected collisions. Each entry contains 'config', 'segment' (p1, p2),
        'radius', and 'shape_type' keys."""
        return self.collision_log.copy()

    def clear_log(self):
        """Clear the collision log."""
        self.collision_log.clear()


