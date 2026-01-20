"""
OPW Kinematics wrapper.

OPW provides closed-form inverse kinematics for industrial 6R arms.

This module intentionally:
- Does NOT do planning
- Does NOT do collision checking
- Does NOT choose solutions

It only answers:
"Given a pose, what joint configurations are kinematically valid?"
"""

from core.robot_model import RobotModel

import numpy as np


class OPWKinematics:
    """
    Thin abstraction over OPW-style closed-form IK.

    This is written as a conceptual wrapper.
    In practice, this would bind to an actual OPW implementation.
    """

    def __init__(self, robot_model: RobotModel):
        """Initialize the OPW kinematics solver."""
        self.robot = robot_model
        if self.robot.dof() not in [5, 6]:
            raise ValueError('OPW kinematics only supports 5DOF and 6DOF arms')

    def solve(self, pose):
        """
        Compute all IK solutions for a desired end-effector pose.

        Args:
            pose: End-effector pose (SE(3)-like object)

        Returns:
            List of joint vectors, each shape (6,)
        """
        # Placeholder for real OPW computation
        solutions = []

        # In a real implementation, this would return up to 8 solutions
        return solutions

    def filter_joint_limits(self, solutions):
        """Discard solutions outside joint limits."""
        lower, upper = self.robot.joint_limits()
        valid = []
        for q in solutions:
            if np.all(q >= lower) and np.all(q <= upper):
                valid.append(q)
        return valid
