"""Pose and transformation utilities for robot motion planning.

This module provides utilities for working with poses, rotations, and
transformations commonly used in robotics.
"""

import numpy as np
from typing import Union, Optional, List, Callable


def quaternion_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix.

    Args:
        quat: Quaternion as [x, y, z, w] (PyBullet/ROS convention)

    Returns:
        3x3 rotation matrix
    """
    q = np.array(quat, dtype=float)
    if len(q) != 4:
        raise ValueError("Quaternion must have 4 elements")

    # Normalize
    q = q / np.linalg.norm(q)

    # PyBullet uses [x, y, z, w] format
    x, y, z, w = q

    # Convert to rotation matrix
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert 3x3 rotation matrix to quaternion.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Quaternion as [x, y, z, w] (PyBullet/ROS convention)
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w])


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles (ZYX convention) to 3x3 rotation matrix.

    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)

    Returns:
        3x3 rotation matrix
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    return R


def make_transform(
    position: Union[list, np.ndarray],
    rotation: Optional[Union[list, np.ndarray]] = None,
) -> np.ndarray:
    """
    Create 4x4 homogeneous transformation matrix.

    Args:
        position: [x, y, z] position
        rotation: Can be:
            - None: Identity rotation
            - [x, y, z, w]: Quaternion (PyBullet convention)
            - [roll, pitch, yaw]: Euler angles (radians)
            - 3x3 array: Rotation matrix

    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = np.array(position)

    if rotation is None:
        return T

    rotation = np.array(rotation)

    if rotation.shape == (3, 3):
        T[:3, :3] = rotation
    elif rotation.shape == (4,):
        T[:3, :3] = quaternion_to_rotation_matrix(rotation)
    elif rotation.shape == (3,):
        T[:3, :3] = euler_to_rotation_matrix(*rotation)
    else:
        raise ValueError(f"Invalid rotation shape: {rotation.shape}")

    return T


def transform_to_pose(T: np.ndarray) -> tuple:
    """
    Extract position and quaternion from 4x4 transform.

    Args:
        T: 4x4 transformation matrix

    Returns:
        (position, quaternion) where position is [x,y,z] and quaternion is [x,y,z,w]
    """
    position = T[:3, 3].copy()
    quaternion = rotation_matrix_to_quaternion(T[:3, :3])
    return position, quaternion


class IKSolver:
    """Wrapper for inverse kinematics solving with solution selection."""

    def __init__(self, robot, ik_solver):
        """
        Args:
            robot: Robot model with joint_limits() method
            ik_solver: IK solver with inverse_kinematics() and filter_joint_limits() methods
        """
        self.robot = robot
        self.ik_solver = ik_solver

    def solve(
        self,
        target_pose: np.ndarray,
        reference_q: Optional[np.ndarray] = None,
        collision_check: Optional[Callable[[np.ndarray], bool]] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve IK and return best solution.

        Args:
            target_pose: 4x4 target transformation matrix
            reference_q: Reference joint config for selecting closest solution
            collision_check: Optional collision checker, returns True if in collision

        Returns:
            Best joint configuration or None if no valid solution
        """
        # Get all IK solutions
        solutions = self.ik_solver.inverse_kinematics(target_pose)

        # Filter valid (non-NaN) solutions
        valid = [s for s in solutions if not np.any(np.isnan(s))]
        if not valid:
            return None

        # Filter by joint limits
        within_limits = self.ik_solver.filter_joint_limits(valid)
        if within_limits:
            valid = within_limits

        # Filter by collision
        if collision_check is not None:
            collision_free = [q for q in valid if not collision_check(q)]
            if collision_free:
                valid = collision_free

        if not valid:
            return None

        # Select closest to reference
        if reference_q is not None:
            distances = [np.linalg.norm(q - reference_q) for q in valid]
            return valid[np.argmin(distances)]

        return valid[0]

    def solve_from_position(
        self,
        position: np.ndarray,
        rotation: Optional[np.ndarray] = None,
        reference_q: Optional[np.ndarray] = None,
        collision_check: Optional[Callable[[np.ndarray], bool]] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve IK from position and optional rotation.

        Args:
            position: [x, y, z] target position
            rotation: Optional rotation (quaternion, euler, or 3x3 matrix)
            reference_q: Reference joint config
            collision_check: Optional collision checker

        Returns:
            Best joint configuration or None
        """
        target_pose = make_transform(position, rotation)
        return self.solve(target_pose, reference_q, collision_check)
