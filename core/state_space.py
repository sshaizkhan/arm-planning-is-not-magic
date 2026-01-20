"""
State space abstraction (Configuration Space).

This module defines how joint-space states are:
- Sampled
- Validated
- Interpolated

This mirrors how OMPL thinks about state spaces, but remains
library-agnostic and explicit for learning purposes.
"""

from core.robot_model import RobotModel

import numpy as np


class JointStateSpace:
    """
    Joint-space (C-space) for an n-DOF robot.

    This class intentionally:
    - Knows NOTHING about planning algorithms
    - Knows NOTHING about timing or dynamics
    - Only reasons about valid configurations
    """

    def __init__(self, robot_model: RobotModel):
        """Initialize the joint state space."""
        self.robot = robot_model
        self.dim = robot_model.dof()
        self.lower, self.upper = robot_model.joint_limits()

    # ------------------------------------------------------------------
    # Sampling
    # ------------------------------------------------------------------

    def sample_uniform(self) -> np.ndarray:
        """Uniform random sample in joint limits."""
        return np.random.uniform(self.lower, self.upper)

    # ------------------------------------------------------------------
    # Validity
    # ------------------------------------------------------------------

    def within_limits(self, q: np.ndarray) -> bool:
        """Check joint limits only (no collision)."""
        return bool(np.all(q >= self.lower) and np.all(q <= self.upper))

    def is_valid(self, q: np.ndarray) -> bool:
        """Full state validity check."""
        if not self.within_limits(q):
            return False
        return not self.robot.in_collision(q)

    # ------------------------------------------------------------------
    # Distance & Interpolation
    # ------------------------------------------------------------------

    def distance(self, q1: np.ndarray, q2: np.ndarray) -> np.floating:
        """Compute the distance between two joint configurations."""
        return self.robot.distance(q1, q2)

    def interpolate(self, q1: np.ndarray, q2: np.ndarray, alpha: float) -> np.ndarray:
        """
        Linear interpolation in joint space.

        alpha in [0, 1]
        """
        return (1.0 - alpha) * q1 + alpha * q2

    def discretize(self, q1: np.ndarray, q2: np.ndarray, step_size: float):
        """
        Discretize a segment for collision checking.

        Args:
            q1, q2: endpoints
            step_size: max joint-space distance between samples
        """
        dist = self.distance(q1, q2)
        if dist == 0:
            return [q1]

        steps = int(np.ceil(dist / step_size))
        return [self.interpolate(q1, q2, i / steps) for i in range(steps + 1)]
