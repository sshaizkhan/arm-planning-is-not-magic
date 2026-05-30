"""
OMPL RRT* (RRT-Star) planner.

Extends RRT with a rewiring step: after adding a node, it checks whether
neighboring nodes are better reached through the new node. Over time, this
converges to the optimal path.

Guarantee: as samples → ∞, solution → optimal (asymptotically optimal).
Cost: more collision checks per iteration than RRT.

In practice: run with a longer timeout than RRT for a meaningful improvement.
"""

from ompl import base as ob  # type: ignore
from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLRRTStarPlanner(BaseOMPLPlanner):
    """Asymptotically optimal RRT with tree rewiring."""

    def _configure_planner(self, planner, pdef):
        pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(self.si))

    def _create_planner(self):
        planner = og.RRTstar(self.si)
        if self.step_size is not None:
            planner.setRange(self.step_size)
        return planner
