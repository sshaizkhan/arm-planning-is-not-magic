"""
OMPL LBTRRT (Lazy Bidirectional Transition-based RRT) planner.

Bidirectional variant that grows trees from both start and goal.
"""

import numpy as np

try:
    from ompl import base as ob  # type: ignore
    from ompl import geometric as og  # type: ignore
except ImportError:
    raise ImportError(
        "OMPL Python bindings are required. "
        "Install via your ROS distro or OMPL build."
    )


class OMPLBiTRRTPlanner:
    """
    OMPL LBTRRT (Lazy Bidirectional Transition-based RRT) planner.
    """

    def __init__(self, state_space, step_size=0.1):
        self.state_space = state_space
        self.robot = state_space.robot
        self.dim = state_space.dim

        # ------------------------------------------------------------------
        # OMPL state space definition
        # ------------------------------------------------------------------
        self.ompl_space = ob.RealVectorStateSpace(self.dim)  # type: ignore
        bounds = ob.RealVectorBounds(self.dim)  # type: ignore
        lower, upper = self.robot.joint_limits()

        for i in range(self.dim):
            bounds.setLow(i, lower[i])
            bounds.setHigh(i, upper[i])

        self.ompl_space.setBounds(bounds)

        # ------------------------------------------------------------------
        # Space information
        # ------------------------------------------------------------------
        self.si = ob.SpaceInformation(self.ompl_space)  # type: ignore

        def is_state_valid(state):
            q = np.array([state[i] for i in range(self.dim)])
            return self.state_space.is_valid(q)

        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))  # type: ignore
        self.si.setup()

        self.step_size = step_size

    def plan(self, q_start, q_goal, timeout=1.0):
        """
        Plan a collision-free joint-space path.

        Args:
            q_start: start configuration (dof,)
            q_goal: goal configuration (dof,)
            timeout: planning time in seconds

        Returns:
            List of joint configurations (path), or None if failed.
        """
        start = ob.State(self.ompl_space)  # type: ignore
        goal = ob.State(self.ompl_space)  # type: ignore
        for i in range(self.dim):
            start[i] = q_start[i]
            goal[i] = q_goal[i]

        pdef = ob.ProblemDefinition(self.si)  # type: ignore
        pdef.setStartAndGoalStates(start, goal)

        planner = og.LBTRRT(self.si)  # type: ignore
        planner.setRange(self.step_size)  # type: ignore
        planner.setProblemDefinition(pdef)
        planner.setup()

        solved = planner.solve(timeout)

        if not solved:
            return None

        path_geometric = pdef.getSolutionPath()
        path_geometric.interpolate()

        path = []
        for state in path_geometric.getStates():
            q = np.array([state[i] for i in range(self.dim)])
            path.append(q)
        return path
