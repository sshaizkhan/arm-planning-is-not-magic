"""
OMPL KPIECE1 (Kinodynamic Planning by Interior-Exterior Cell Exploration) planner.

KPIECE1 divides C-space into cells and guides exploration toward cells with
low coverage. This helps it find paths through narrow passages where uniform
sampling fails.

Use this when:
- High-dimensional spaces
- Narrow passages or tight constraints
"""

from ompl import geometric as og  # type: ignore

from planners.base_ompl_planner import BaseOMPLPlanner


class OMPLKPiece1Planner(BaseOMPLPlanner):
    """Cell-based exploration. Good for narrow passages."""

    def _create_planner(self):
        return og.KPIECE1(self.si)
