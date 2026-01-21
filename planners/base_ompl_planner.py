"""
Base OMPL planner abstraction.

This module provides a common base class for all OMPL-based planners,
eliminating code duplication and making the OMPL integration pattern explicit.

Key insight:
-----------
All OMPL planners share the same setup:
1. Create state space from joint limits
2. Define validity checker (collision + limits)
3. Create problem definition
4. Solve and extract path

Only the specific planner algorithm differs.
"""

from abc import ABC, abstractmethod
from typing import List, Optional

import numpy as np

try:
    from ompl import base as ob  # type: ignore
    from ompl import geometric as og  # type: ignore
except ImportError:
    raise ImportError(
        "OMPL Python bindings are required. "
        "Install via your ROS distro or OMPL build."
    )


class BaseOMPLPlanner(ABC):
    """
    Abstract base class for OMPL-based planners.

    This class handles:
    - OMPL state space setup
    - Validity checking delegation
    - Path extraction from OMPL solution

    Subclasses only need to implement:
    - _create_planner(): Return the specific OMPL planner instance
    - _configure_planner(): Set planner-specific parameters
    """

    def __init__(self, state_space, step_size: Optional[float] = None):
        """
        Initialize the OMPL planner base.

        Args:
            state_space: JointStateSpace instance
            step_size: Optional step size for planners that support it
        """
        self.state_space = state_space
        self.robot = state_space.robot
        self.dim = state_space.dim
        self.step_size = step_size

        # Create OMPL state space
        self.ompl_space = ob.RealVectorStateSpace(self.dim)
        bounds = ob.RealVectorBounds(self.dim)
        lower, upper = self.robot.joint_limits()

        for i in range(self.dim):
            bounds.setLow(i, lower[i])
            bounds.setHigh(i, upper[i])

        self.ompl_space.setBounds(bounds)

        # Create space information with validity checker
        self.si = ob.SpaceInformation(self.ompl_space)

        def is_state_valid(state):
            q = np.array([state[i] for i in range(self.dim)])
            return self.state_space.is_valid(q)

        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
        self.si.setup()

    @abstractmethod
    def _create_planner(self):
        """
        Create and return the specific OMPL planner instance.

        Returns:
            OMPL planner object (e.g., og.RRT, og.RRTConnect, etc.)
        """
        pass

    def _configure_planner(self, planner, pdef):
        """
        Configure planner-specific parameters.

        Override in subclasses to set parameters like range, goal bias, etc.

        Args:
            planner: OMPL planner instance
            pdef: OMPL problem definition
        """
        pass

    def _create_problem_definition(self, q_start: np.ndarray, q_goal: np.ndarray):
        """
        Create OMPL problem definition.

        Args:
            q_start: Start configuration
            q_goal: Goal configuration

        Returns:
            OMPL ProblemDefinition instance
        """
        start = ob.State(self.ompl_space)
        goal = ob.State(self.ompl_space)

        for i in range(self.dim):
            start[i] = q_start[i]
            goal[i] = q_goal[i]

        pdef = ob.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(start, goal)

        return pdef

    def _extract_path(self, pdef) -> List[np.ndarray]:
        """
        Extract path from OMPL solution.

        Args:
            pdef: OMPL problem definition with solution

        Returns:
            List of joint configurations
        """
        path_geometric = pdef.getSolutionPath()
        path_geometric.interpolate()

        path = []
        for state in path_geometric.getStates():
            q = np.array([state[i] for i in range(self.dim)])
            path.append(q)

        return path

    def plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        timeout: float = 1.0
    ) -> Optional[List[np.ndarray]]:
        """
        Plan a collision-free joint-space path.

        Args:
            q_start: Start configuration, shape (dof,)
            q_goal: Goal configuration, shape (dof,)
            timeout: Planning time limit in seconds

        Returns:
            List of joint configurations (path), or None if planning failed
        """
        pdef = self._create_problem_definition(q_start, q_goal)

        planner = self._create_planner()
        self._configure_planner(planner, pdef)

        planner.setProblemDefinition(pdef)
        planner.setup()

        solved = planner.solve(timeout)

        if not solved:
            return None

        return self._extract_path(pdef)

    @property
    def name(self) -> str:
        """Return the planner name for logging/display."""
        return self.__class__.__name__
