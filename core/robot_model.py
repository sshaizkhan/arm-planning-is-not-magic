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
    """Robot model for the UR5 robot with OPW kinematics."""

    def __init__(self, use_opw: bool = True, collision_manager=None):
        """
        Initialize the robot model.

        Args:
            use_opw: If True, use OPW kinematics for FK (requires OPW parameters)
            collision_manager: Optional CollisionManager instance for collision checking.
                              If None, in_collision always returns False.
        """
        self._dof = 6
        # UR5 actual joint limits from URDF (in radians)
        # Joint 1 (base): ±170° = ±2.967 rad
        # Joint 2 (shoulder): -90° to 155° = -1.571 to 2.705 rad
        # Joint 3 (elbow): -85° to 150° = -1.484 to 2.618 rad
        # Joint 4 (wrist 1): ±200° = ±3.491 rad
        # Joint 5 (wrist 2): ±150° = ±2.618 rad
        # Joint 6 (wrist 3): ±455° = ±7.941 rad
        self._lower = np.array([
            -2.9670597283903604,  # Joint 1
            -1.5707963267948966,  # Joint 2
            -1.4835298641951802,  # Joint 3
            -3.490658503988659,   # Joint 4
            -2.6179938779914944,  # Joint 5
            -7.941248096574199,   # Joint 6
        ])
        self._upper = np.array([
            2.9670597283903604,   # Joint 1
            2.705260340591211,    # Joint 2
            2.6179938779914944,   # Joint 3
            3.490658503988659,    # Joint 4
            2.6179938779914944,   # Joint 5
            7.941248096574199,    # Joint 6
        ])
        self._use_opw = use_opw
        self._opw_kinematics = None
        self._collision_manager = collision_manager

        if use_opw:
            try:
                from core.kinematics.opw import OPWKinematics
                from core.kinematics.opw_parameters import OPWParameters
                params = OPWParameters.make_ur5()
                self._opw_kinematics = OPWKinematics(self, params)
            except ImportError:
                self._use_opw = False

    def set_collision_manager(self, collision_manager):
        """
        Set or update the collision manager.

        Args:
            collision_manager: CollisionManager instance
        """
        self._collision_manager = collision_manager

    def dof(self) -> int:
        """Return number of joints (dimensions of C-space)."""
        return 6

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return joint limits as (lower, upper) each shape (dof,)."""
        return self._lower, self._upper

    def fk(self, q: np.ndarray) -> np.ndarray:
        """
        Forward kinematics.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            End-effector pose as 4x4 transformation matrix
        """
        if self._use_opw and self._opw_kinematics is not None:
            return self._opw_kinematics.forward_kinematics(q)
        else:
            raise NotImplementedError(
                'Forward kinematics requires OPW kinematics. '
                'Install dependencies or set use_opw=False.'
            )

    def in_collision(self, q: np.ndarray) -> bool:
        """Check if a joint configuration is in collision.

        Args:
            q: Joint configuration, shape (dof,)

        Returns:
            True if in collision, False otherwise
        """
        if self._collision_manager is not None:
            return self._collision_manager.in_collision(q)
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
