"""Kinematics implementations for robot arms.

This module provides:
- OPWKinematics: Closed-form IK for 6DOF industrial robots
- OPWParameters: Robot-specific kinematic parameters
"""

from core.kinematics.opw import OPWKinematics
from core.kinematics.opw_parameters import OPWParameters

__all__ = [
    "OPWKinematics",
    "OPWParameters",
]
