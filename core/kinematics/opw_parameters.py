"""
OPW kinematic parameters for industrial 6R robots.

Based on the paper:
"An Analytical Solution of the Inverse Kinematics Problem of Industrial
Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist"
by Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.

OPW (Ortho-Parallel-Wrist) robots have:
- First three axes that are either parallel or orthogonal
- A spherical wrist (axes 4, 5, 6 intersect at a point)

This covers most industrial 6-DOF robots: UR, KUKA, ABB, FANUC, etc.

Geometric parameters:
    a1: Offset from base to first joint along X
    a2: Link 2 offset along X (often negative)
    b:  Offset along Y (wrist offset)
    c1: Height from base to joint 2
    c2: Length of link 2
    c3: Length of link 3
    c4: Wrist to tool flange distance
"""

import numpy as np


class OPWParameters:
    """
    OPW kinematic parameters for 6DOF industrial robots.

    Parameters:
        a1, a2, b, c1, c2, c3, c4: Geometric parameters (meters)
        offsets: Joint zero offsets (6 elements, radians)
        sign_corrections: Sign corrections for joint axes (-1 or 1, 6 elements)

    Available factory methods:
        make_ur5()   - Universal Robots UR5
        make_ur10()  - Universal Robots UR10
        make_ur3()   - Universal Robots UR3
        make_kuka_kr6_r900() - KUKA KR6 R900
        make_abb_irb2400()   - ABB IRB 2400
        make_fanuc_m10ia()   - FANUC M-10iA
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

    def __repr__(self):
        return (
            f"OPWParameters(a1={self.a1:.4f}, a2={self.a2:.4f}, b={self.b:.4f}, "
            f"c1={self.c1:.4f}, c2={self.c2:.4f}, c3={self.c3:.4f}, c4={self.c4:.4f})"
        )

    # =========================================================================
    # Universal Robots
    # =========================================================================

    @classmethod
    def make_ur5(cls):
        """
        Create OPW parameters for UR5 robot.

        Specs: 850mm reach, 5kg payload
        """
        params = cls()
        params.a1 = 0.089159
        params.a2 = -0.425
        params.b = 0.0
        params.c1 = 0.089159
        params.c2 = 0.425
        params.c3 = 0.39225
        params.c4 = 0.09465
        params.offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        params.sign_corrections = np.array([1, 1, 1, 1, 1, 1], dtype=np.int8)
        return params

    @classmethod
    def make_ur10(cls):
        """
        Create OPW parameters for UR10 robot.

        Specs: 1300mm reach, 10kg payload
        """
        params = cls()
        params.a1 = 0.1180
        params.a2 = -0.6120
        params.b = 0.0
        params.c1 = 0.1273
        params.c2 = 0.6120
        params.c3 = 0.5723
        params.c4 = 0.1157
        params.offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        params.sign_corrections = np.array([1, 1, 1, 1, 1, 1], dtype=np.int8)
        return params

    @classmethod
    def make_ur3(cls):
        """
        Create OPW parameters for UR3 robot.

        Specs: 500mm reach, 3kg payload
        """
        params = cls()
        params.a1 = 0.0
        params.a2 = -0.24365
        params.b = 0.0
        params.c1 = 0.15185
        params.c2 = 0.24365
        params.c3 = 0.21325
        params.c4 = 0.08535
        params.offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        params.sign_corrections = np.array([1, 1, 1, 1, 1, 1], dtype=np.int8)
        return params

    # =========================================================================
    # KUKA Robots
    # =========================================================================

    @classmethod
    def make_kuka_kr6_r900(cls):
        """
        Create OPW parameters for KUKA KR6 R900 robot.

        Specs: 901mm reach, 6kg payload
        """
        params = cls()
        params.a1 = 0.025
        params.a2 = -0.560
        params.b = 0.0
        params.c1 = 0.400
        params.c2 = 0.455
        params.c3 = 0.035
        params.c4 = 0.080
        params.offsets = np.array([0.0, -np.pi/2, 0.0, 0.0, 0.0, np.pi])
        params.sign_corrections = np.array([-1, 1, 1, -1, 1, -1], dtype=np.int8)
        return params

    # =========================================================================
    # ABB Robots
    # =========================================================================

    @classmethod
    def make_abb_irb2400(cls):
        """
        Create OPW parameters for ABB IRB 2400 robot.

        Specs: 1550mm reach, 12kg payload
        """
        params = cls()
        params.a1 = 0.100
        params.a2 = -0.705
        params.b = 0.0
        params.c1 = 0.615
        params.c2 = 0.705
        params.c3 = 0.135
        params.c4 = 0.135
        params.offsets = np.array([0.0, 0.0, -np.pi/2, 0.0, 0.0, 0.0])
        params.sign_corrections = np.array([1, 1, -1, 1, 1, 1], dtype=np.int8)
        return params

    # =========================================================================
    # FANUC Robots
    # =========================================================================

    @classmethod
    def make_fanuc_m10ia(cls):
        """
        Create OPW parameters for FANUC M-10iA robot.

        Specs: 1420mm reach, 10kg payload
        """
        params = cls()
        params.a1 = 0.150
        params.a2 = -0.600
        params.b = 0.0
        params.c1 = 0.525
        params.c2 = 0.625
        params.c3 = 0.200
        params.c4 = 0.110
        params.offsets = np.array([0.0, 0.0, -np.pi/2, 0.0, 0.0, np.pi])
        params.sign_corrections = np.array([1, 1, -1, 1, 1, -1], dtype=np.int8)
        return params

    # =========================================================================
    # Generic Robot Creation
    # =========================================================================

    @classmethod
    def from_dh_like(cls, a1, a2, b, c1, c2, c3, c4, offsets=None, signs=None):
        """
        Create OPW parameters from explicit values.

        This is useful for robots not in the preset list.

        Args:
            a1, a2, b, c1, c2, c3, c4: Geometric parameters in meters
            offsets: Optional joint offsets (default: zeros)
            signs: Optional sign corrections (default: ones)

        Returns:
            OPWParameters instance
        """
        params = cls()
        params.a1 = a1
        params.a2 = a2
        params.b = b
        params.c1 = c1
        params.c2 = c2
        params.c3 = c3
        params.c4 = c4
        if offsets is not None:
            params.offsets = np.array(offsets, dtype=np.float64)
        if signs is not None:
            params.sign_corrections = np.array(signs, dtype=np.int8)
        return params
