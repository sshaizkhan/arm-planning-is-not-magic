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

    def check_capsule(self, p1: np.ndarray, p2: np.ndarray, radius: float) -> bool:
        """
        Check if a capsule (cylinder with hemispherical ends) intersects the box.

        A capsule is defined by a line segment (p1 to p2) and a radius.
        This checks if the swept sphere along the segment intersects the box.

        Args:
            p1: Start point of line segment (3,)
            p2: End point of line segment (3,)
            radius: Radius of the capsule

        Returns:
            True if capsule intersects box, False otherwise
        """
        # Expand the box by the capsule radius for the distance check
        expanded_min = self.min_corner - radius
        expanded_max = self.max_corner + radius

        # Find closest point on line segment to the box center
        # Then check if that point is within the expanded box
        segment = p2 - p1
        segment_length_sq = np.dot(segment, segment)

        if segment_length_sq < 1e-10:
            # Degenerate case: segment is a point
            return self.check_point(p1) or np.linalg.norm(
                np.maximum(self.min_corner - p1, 0) +
                np.maximum(p1 - self.max_corner, 0)
            ) <= radius

        # Find the closest point on the segment to the box
        # We need to find the point on segment closest to the box
        closest_dist_sq = self._segment_box_distance_squared(p1, p2)
        return closest_dist_sq <= radius * radius

    def _segment_box_distance_squared(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """
        Compute squared distance from line segment to axis-aligned box.

        Uses the separating axis theorem approach.
        """
        # Clamp helper
        def clamp(val, min_val, max_val):
            return max(min_val, min(val, max_val))

        # Sample multiple points along segment and find minimum distance
        # This is an approximation but works well for robot collision checking
        num_samples = 10
        min_dist_sq = float('inf')

        for i in range(num_samples + 1):
            t = i / num_samples
            point = p1 + t * (p2 - p1)

            # Compute distance from point to box
            # Closest point on box to the point
            closest = np.array([
                clamp(point[0], self.min_corner[0], self.max_corner[0]),
                clamp(point[1], self.min_corner[1], self.max_corner[1]),
                clamp(point[2], self.min_corner[2], self.max_corner[2]),
            ])

            dist_sq = np.sum((point - closest) ** 2)
            min_dist_sq = min(min_dist_sq, dist_sq)

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
        """
        Check if a capsule intersects the sphere.

        Args:
            p1: Start point of line segment (3,)
            p2: End point of line segment (3,)
            capsule_radius: Radius of the capsule

        Returns:
            True if capsule intersects sphere, False otherwise
        """
        # Find closest point on segment to sphere center
        closest = self._closest_point_on_segment(p1, p2, self.center)
        dist = np.linalg.norm(closest - self.center)
        return dist <= (self.radius + capsule_radius)

    def _closest_point_on_segment(self, p1: np.ndarray, p2: np.ndarray, point: np.ndarray) -> np.ndarray:
        """Find closest point on line segment p1-p2 to the given point."""
        segment = p2 - p1
        length_sq = np.dot(segment, segment)
        if length_sq < 1e-10:
            return p1.copy()
        t = max(0, min(1, np.dot(point - p1, segment) / length_sq))
        return p1 + t * segment

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
        """
        Check if a capsule intersects the cylinder.

        Args:
            p1: Start point of line segment (3,)
            p2: End point of line segment (3,)
            capsule_radius: Radius of the capsule

        Returns:
            True if capsule intersects cylinder, False otherwise
        """
        # Sample points along the segment and check each
        num_samples = 10
        combined_radius = self.radius + capsule_radius
        z_min_expanded = self.z_min - capsule_radius
        z_max_expanded = self.z_max + capsule_radius

        for i in range(num_samples + 1):
            t = i / num_samples
            point = p1 + t * (p2 - p1)

            # Check height bounds (expanded by capsule radius)
            if not (z_min_expanded <= point[2] <= z_max_expanded):
                continue

            # Check radial distance (expanded by capsule radius)
            radial_dist = np.sqrt(
                (point[0] - self.center[0])**2 +
                (point[1] - self.center[1])**2
            )
            if radial_dist <= combined_radius:
                return True

        return False

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
    Uses capsule-based collision checking for entire link segments, not just points.
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

        # UR5 link parameters for collision checking (approximate)
        # These are key positions along the kinematic chain
        self._ur5_params = {
            'c1': 0.089159,   # Base height
            'a2': -0.425,     # Upper arm length
            'a3': -0.39225,   # Forearm length
            'c4': 0.10915,    # Wrist 1 length
        }

        # UR5 link radii for capsule collision checking (meters)
        # These approximate the robot's physical dimensions
        self._link_radii = {
            'base': 0.08,       # Base cylinder
            'shoulder': 0.075,  # Shoulder joint
            'upper_arm': 0.06,  # Upper arm link
            'forearm': 0.05,    # Forearm link
            'wrist': 0.045,     # Wrist section
            'ee': 0.04,         # End-effector
        }

    def add_shape(self, shape: CollisionShape):
        """Add a collision shape to the scene."""
        self.shapes.append(shape)

    def clear_shapes(self):
        """Remove all collision shapes."""
        self.shapes.clear()
        self.collision_log.clear()

    def _compute_link_segments(self, q: np.ndarray) -> List[tuple]:
        """
        Compute link segments (capsules) for collision checking.

        Returns a list of (start_point, end_point, radius) tuples representing
        each link as a capsule for collision detection.

        Args:
            q: Joint configuration, shape (6,)

        Returns:
            List of (p1, p2, radius) tuples for each link segment
        """
        segments = []

        c1, c2 = np.cos(q[0]), np.cos(q[1])
        s1, s2 = np.sin(q[0]), np.sin(q[1])

        # Key positions along the kinematic chain
        base = np.array([0.0, 0.0, 0.0])
        shoulder = np.array([0.0, 0.0, self._ur5_params['c1']])

        a2 = abs(self._ur5_params['a2'])
        elbow = np.array([
            a2 * c1 * s2,
            a2 * s1 * s2,
            self._ur5_params['c1'] + a2 * c2
        ])

        a3 = abs(self._ur5_params['a3'])
        wrist = np.array([
            c1 * (a2 * s2 + a3 * np.sin(q[1] + q[2])),
            s1 * (a2 * s2 + a3 * np.sin(q[1] + q[2])),
            self._ur5_params['c1'] + a2 * c2 + a3 * np.cos(q[1] + q[2])
        ])

        # Get end-effector position from full FK
        ee_position = wrist.copy()  # Default to wrist if FK fails
        try:
            pose = self.robot.fk(q)
            if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
                ee_position = pose[:3, 3]
        except Exception:
            pass

        # Define link segments as capsules (start, end, radius)
        # Base to shoulder (vertical link)
        segments.append((base, shoulder, self._link_radii['base']))

        # Shoulder to elbow (upper arm)
        segments.append((shoulder, elbow, self._link_radii['upper_arm']))

        # Elbow to wrist (forearm)
        segments.append((elbow, wrist, self._link_radii['forearm']))

        # Wrist to end-effector
        segments.append((wrist, ee_position, self._link_radii['wrist']))

        return segments

    def _compute_link_positions(self, q: np.ndarray) -> List[np.ndarray]:
        """
        Compute positions of key points along the robot arm.

        Returns a list of 3D positions for collision checking:
        - Base (fixed at origin)
        - Shoulder
        - Elbow
        - Wrist
        - End-effector

        Args:
            q: Joint configuration, shape (6,)

        Returns:
            List of position arrays, each shape (3,)
        """
        positions = []

        # Base position (always at origin, slightly elevated)
        positions.append(np.array([0.0, 0.0, self._ur5_params['c1'] / 2]))

        c1, c2 = np.cos(q[0]), np.cos(q[1])
        s1, s2 = np.sin(q[0]), np.sin(q[1])
        c12 = np.cos(q[0] + q[1])
        s12 = np.sin(q[0] + q[1])
        c123 = np.cos(q[0] + q[1] + q[2])
        s123 = np.sin(q[0] + q[1] + q[2])

        # Shoulder position (after joint 1)
        shoulder = np.array([0.0, 0.0, self._ur5_params['c1']])
        positions.append(shoulder)

        # Elbow position (after joints 1-2) - upper arm
        # Upper arm extends from shoulder
        a2 = abs(self._ur5_params['a2'])
        elbow = np.array([
            a2 * c1 * s2,
            a2 * s1 * s2,
            self._ur5_params['c1'] + a2 * c2
        ])
        positions.append(elbow)

        # Mid-forearm position (midpoint between elbow and wrist)
        a3 = abs(self._ur5_params['a3'])
        wrist = np.array([
            c1 * (a2 * s2 + a3 * np.sin(q[1] + q[2])),
            s1 * (a2 * s2 + a3 * np.sin(q[1] + q[2])),
            self._ur5_params['c1'] + a2 * c2 + a3 * np.cos(q[1] + q[2])
        ])

        # Mid-forearm
        mid_forearm = (elbow + wrist) / 2
        positions.append(mid_forearm)

        # Wrist position
        positions.append(wrist)

        # End-effector position from full FK
        try:
            pose = self.robot.fk(q)
            if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
                ee_position = pose[:3, 3]
                positions.append(ee_position)

                # Add mid-point between wrist and EE
                mid_wrist_ee = (wrist + ee_position) / 2
                positions.append(mid_wrist_ee)
        except Exception:
            pass

        return positions

    def in_collision(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is in collision.

        Uses capsule-based collision checking for entire link segments,
        accounting for link radius/thickness.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        if not self.shapes:
            return False

        try:
            # Get link segments as capsules (start, end, radius)
            link_segments = self._compute_link_segments(q)

            # Check each link segment against all shapes
            for p1, p2, radius in link_segments:
                for shape in self.shapes:
                    # Use capsule collision if available, otherwise fall back to point check
                    if hasattr(shape, 'check_capsule'):
                        if shape.check_capsule(p1, p2, radius):
                            self.collision_log.append({
                                'config': q.copy(),
                                'collision_segment': (p1.copy(), p2.copy()),
                                'link_radius': radius,
                                'shape_type': shape.get_type(),
                            })
                            return True
                    else:
                        # Fallback: check endpoints
                        if shape.check_point(p1) or shape.check_point(p2):
                            self.collision_log.append({
                                'config': q.copy(),
                                'collision_point': p1.copy(),
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
