"""
OMPL RRT-Connect planner.

Bidirectional variant: grows trees from BOTH start and goal simultaneously,
connecting when they meet. Typically 2-5x faster than single-tree RRT.

Use this when:
- No path cost optimization needed
- Speed matters more than path quality
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLRRTConnectPlanner(BaseOMPLPlanner):
    """Bidirectional RRT. Fastest for most point-to-point problems."""

    def _create_planner(self):
        planner = og.RRTConnect(self.si)
        if self.step_size is not None:
            planner.setRange(self.step_size)
        return planner
