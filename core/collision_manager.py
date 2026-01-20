"""
Collision management abstraction.

This module isolates collision checking from:
- Planning
- State spaces
- Timing

This mirrors real systems where collision checking is a service,
not a planner responsibility.
"""

from core.robot_model import RobotModel

import numpy as np


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
        return bool(False)


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
        # Stub: always collision-free
        return False
