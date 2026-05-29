"""
OMPL RRT (Rapidly-exploring Random Tree) planner.

RRT grows a single tree by sampling random configurations and steering toward them.

Properties:
- Fast exploration of large spaces
- Paths are jagged (not optimal)
- Sensitive to step_size — too small = slow, too large = misses narrow passages

This planner is the conceptual baseline. Understand it before the variants.
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLRRTPlanner(BaseOMPLPlanner):
    """Single-tree RRT. Fast, non-optimal, good baseline."""

    def _create_planner(self):
        planner = og.RRT(self.si)
        if self.step_size is not None:
            planner.setRange(self.step_size)
        return planner
