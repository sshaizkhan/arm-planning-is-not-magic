"""Robot model abstraction.

This file defines the *minimal interface* required by any motion planner.
Everything else in the repo (OMPL, timing, demos) talks ONLY to this API.


Key idea:
----------
Planning does NOT care about:
- Robot brand
- IK method
- Controller type


It only cares about:
- C-space dimension
- Joint limits
- FK for collision checking
- A distance metric
"""

from abc import ABC, abstractmethod

import numpy as np


class RobotModel(ABC):
    """Abstract base class for all robot models."""

    @abstractmethod
    def dof(self) -> int:
        """Return number of joints (dimensions of C-space)."""
        pass

    @abstractmethod
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return joint limits as (lower, upper) each shape (dof,)."""
        pass

    @abstractmethod
    def fk(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            End-effector pose
        """
        pass

    @abstractmethod
    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        pass

    @abstractmethod
    def distance(self, q1: np.ndarray, q2: np.ndarray) -> np.floating:
        """Compute the distance between two joint configurations.

        Args:
            q1: Joint configuration, shape (dof,)
            q2: Joint configuration, shape (dof,)

        Returns:
            Distance between the two configurations
        """
        return np.linalg.norm(q1 - q2)


class UR5RobotModel(RobotModel):
    """Robot model for the UR5 robot."""

    def __init__(self):
        """Initialize the robot model."""
        self._dof = 6
        self._lower = np.array([-2*np.pi] * 6)
        self._upper = np.array([2*np.pi] * 6)

    def dof(self) -> int:
        """Return number of joints (dimensions of C-space)."""
        return 6

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return joint limits as (lower, upper) each shape (dof,)."""
        return self._lower, self._upper

    def fk(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            End-effector pose
        """
        raise NotImplementedError('Forward kinematics is not implemented for the UR5 robot model.')

    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        return False

    def distance(self, q1: np.ndarray, q2: np.ndarray) -> np.floating:
        """Compute the distance between two joint configurations.

        Args:
            q1: Joint configuration, shape (dof,)
            q2: Joint configuration, shape (dof,)

        Returns:
            Distance between the two configurations
        """
        return np.linalg.norm(q1 - q2)
