"""
OMPL PRM (Probabilistic Roadmap Method) planner.

PRM operates in two phases:
1. Learning: sample valid configs, connect neighbors into a roadmap graph
2. Query: find paths through the prebuilt roadmap

Key advantage: the roadmap is reusable. If you need many paths in the same
environment, PRM amortizes its construction cost over queries.

Use this when:
- Environment is mostly static
- You run many planning queries
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLPRMPlanner(BaseOMPLPlanner):
    """Probabilistic Roadmap. Best for multi-query planning in static environments."""

    def _create_planner(self):
        return og.PRM(self.si)
