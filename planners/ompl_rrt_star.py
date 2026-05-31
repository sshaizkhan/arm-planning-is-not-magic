"""
OMPL RRT* (RRT-Star) planner.

Extends RRT with a rewiring step: after adding a node, it checks whether
neighboring nodes are better reached through the new node. Over time, this
converges to the optimal path.

Guarantee: as samples → ∞, solution → optimal (asymptotically optimal).
Cost: more collision checks per iteration than RRT.

Caveat — single-tree planning in constrained spaces:
RRT* grows a single tree from the start and spends much of its budget rewiring
for optimality rather than aggressively connecting to the goal. In tight,
narrow-passage scenes this makes it far slower to find *any* feasible solution
than bidirectional planners (RRT-Connect, BiTRRT) that grow from both ends.
To improve goal connection we raise the goal-sampling bias and set a generous
step range. RRT* still needs a longer timeout than bidirectional planners here.
"""

from ompl import base as ob  # type: ignore
from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner

# Goal-sampling probability. OMPL default is 0.05; we raise it so the single
# tree reaches the goal region more often in constrained spaces.
_GOAL_BIAS = 0.2
# Default max edge length (rad) when no explicit step_size is given. A larger
# range lets the tree cover distance faster toward the goal.
_DEFAULT_RANGE = 1.0


class OMPLRRTStarPlanner(BaseOMPLPlanner):
    """Asymptotically optimal RRT with tree rewiring."""

    def _configure_planner(self, planner, pdef):
        pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(self.si))

    def _create_planner(self):
        planner = og.RRTstar(self.si)
        planner.setGoalBias(_GOAL_BIAS)
        planner.setRange(self.step_size if self.step_size is not None else _DEFAULT_RANGE)
        return planner
