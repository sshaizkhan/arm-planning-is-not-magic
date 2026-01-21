"""
OPW kinematic parameters for industrial 6R robots.

Based on the paper:
"An Analytical Solution of the Inverse Kinematics Problem of Industrial
Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist"
by Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.
"""

import numpy as np


class OPWParameters:
    """
    OPW kinematic parameters for 6DOF industrial robots.

    Parameters:
        a1, a2, b, c1, c2, c3, c4: Geometric parameters from robot spec sheet
        offsets: Joint zero offsets (6 elements)
        sign_corrections: Sign corrections for joint axes (-1 or 1, 6 elements)
    """

    def __init__(self):
        """Initialize with default values."""
        self.a1 = 0.0
        self.a2 = 0.0
        self.b = 0.0
        self.c1 = 0.0
        self.c2 = 0.0
        self.c3 = 0.0
        self.c4 = 0.0
        self.offsets = np.zeros(6)
        self.sign_corrections = np.ones(6, dtype=np.int8)

    @classmethod
    def make_ur5(cls):
        """Create OPW parameters for UR5 robot (example values)."""
        params = cls()
        # These are example values - should be calibrated for actual robot
        params.a1 = 0.089159
        params.a2 = -0.42500
        params.b = 0.0
        params.c1 = 0.8173
        params.c2 = 0.8173
        params.c3 = 0.8173
        params.c4 = 0.093
        params.offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        params.sign_corrections = np.array([1, 1, 1, 1, 1, 1], dtype=np.int8)
        return params
