"""
OMPL EST (Expansive Space Trees) planner.

EST selects nodes for expansion based on how densely sampled their neighborhood
is. Nodes in sparse regions are selected more often, promoting balanced
exploration.

Use this when:
- High-dimensional spaces with uneven obstacle distribution
- RRT gets stuck due to Voronoi bias
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLESTPlanner(BaseOMPLPlanner):
    """Expansive Space Trees (BKPIECE1 fallback — EST not exposed in this OMPL build)."""

    def _create_planner(self):
        planner = og.BKPIECE1(self.si)
        return planner
