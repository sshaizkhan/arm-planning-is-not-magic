"""Time parameterization for converting paths to trajectories.

This module provides two approaches:
- ToppraTimeParameterizer: Time-optimal (offline, global optimization)
- RuckigTimeParameterizer: Jerk-limited (online, reactive)

Key distinction:
- Path: geometric, q(s) where s in [0,1]
- Trajectory: time-explicit, q(t) with velocity/acceleration constraints
"""

from parameterization.toppra_parameterization import ToppraTimeParameterizer
from parameterization.ruckig_parameterization import RuckigTimeParameterizer

__all__ = [
    "ToppraTimeParameterizer",
    "RuckigTimeParameterizer",
]
