"""
OMPL LBTRRT (Lower Bound Tree RRT) planner.

A bidirectional variant that uses lazy collision checking and transition-based
expansion. The bidirectional structure (trees from start and goal) typically
outperforms single-tree RRT in time-to-solution.

Use this when:
- Bidirectional search is beneficial (long paths, complex topology)
- You want a cost-aware bidirectional planner
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLBiTRRTPlanner(BaseOMPLPlanner):
    """Lazy bidirectional transition-based RRT."""

    def _create_planner(self):
        planner = og.LBTRRT(self.si)
        if self.step_size is not None:
            planner.setRange(self.step_size)
        return planner
