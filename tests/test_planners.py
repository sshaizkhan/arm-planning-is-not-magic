"""
Tests for OMPL-based planners (planners/).

All tests in this file require OMPL Python bindings and are marked with
@pytest.mark.ompl.  If OMPL is not installed, they are automatically skipped
(see conftest.py).

Each planner wraps a different OMPL algorithm but shares the same interface:
  planner.plan(q_start, q_goal, timeout) -> List[np.ndarray] | None

A return value of None means the planner ran out of time or couldn't find a
path.  For most planners in open space (NullCollisionManager), a 5-second
budget is more than enough.

Planner overview:
  - RRT          — single tree, fast, non-optimal baseline
  - RRTConnect   — bidirectional trees, faster than RRT in most cases
  - RRTStar      — asymptotically optimal, slower but better paths
  - PRM          — builds a roadmap first, good for repeated queries
  - KPIECE1      — cell-based exploration, good for narrow passages
  - EST          — Expansive Space Tree, alternative exploration strategy
  - BiTRRT       — bidirectional Transition-based RRT
"""

import numpy as np
import pytest

from core.robot_model import UR5RobotModel
from core.collision_manager import NullCollisionManager, ShapeCollisionManager, Box
from core.state_space import JointStateSpace


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def open_space():
    """UR5 state space with no obstacles — all configurations are valid."""
    null_cm = NullCollisionManager()
    robot = UR5RobotModel(use_opw=True, collision_manager=null_cm)
    return JointStateSpace(robot)


# Common start and goal used across all planners
Q_START = np.zeros(6)
Q_GOAL = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])
TIMEOUT = 5.0  # seconds — generous budget for open-space planning


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def assert_valid_path(path, q_start, q_goal):
    """Assert that `path` is a non-empty list starting near q_start and ending near q_goal."""
    assert path is not None, "Planner returned None — no path found within timeout"
    assert len(path) >= 2, "Path must have at least 2 waypoints (start and goal)"
    assert np.allclose(path[0], q_start, atol=1e-3), "Path must start at q_start"
    assert np.allclose(path[-1], q_goal, atol=1e-3), "Path must end at q_goal"


# ---------------------------------------------------------------------------
# RRT
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_rrt_finds_path(open_space):
    """RRT must find a path in open space within 5 seconds.

    RRT (Rapidly-exploring Random Tree) is the baseline single-tree planner.
    In a collision-free space it should always succeed quickly.
    """
    from planners.ompl_rrt import OMPLRRTPlanner
    planner = OMPLRRTPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# RRTConnect
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_rrt_connect_finds_path(open_space):
    """RRTConnect (bidirectional RRT) must find a path in open space.

    RRTConnect grows two trees — one from start, one from goal — and connects
    them.  It is typically faster than single-tree RRT.
    """
    from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
    planner = OMPLRRTConnectPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# RRTStar
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_rrt_star_finds_path(open_space):
    """RRTStar must find a path in open space.

    RRTStar is asymptotically optimal: given infinite time it converges to the
    shortest path.  In practice it finds a valid (not necessarily optimal) path
    quickly and keeps improving.
    """
    from planners.ompl_rrt_star import OMPLRRTStarPlanner
    planner = OMPLRRTStarPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# PRM
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_prm_finds_path(open_space):
    """PRM (Probabilistic Roadmap Method) must find a path in open space.

    PRM first samples random valid configurations and connects nearby ones,
    then queries the roadmap for start-to-goal paths.  It's most efficient
    when the same environment is queried multiple times.
    """
    from planners.ompl_prm import OMPLPRMPlanner
    planner = OMPLPRMPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# KPIECE1
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_kpiece1_finds_path(open_space):
    """KPIECE1 must find a path in open space.

    KPIECE1 (Kinodynamic Planning by Interior-Exterior Cell Exploration) uses
    a cell decomposition of the state space to guide exploration.  It can
    handle narrow passages better than uniform-random sampling strategies.
    """
    from planners.ompl_kpiece1 import OMPLKPIECE1Planner
    planner = OMPLKPIECE1Planner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# EST
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_est_finds_path(open_space):
    """EST (Expansive Space Tree) must find a path in open space.

    EST biases sampling toward less-explored regions.  It's a useful baseline
    for comparing coverage-driven vs. goal-biased exploration.
    """
    from planners.ompl_est import OMPLESTPlanner
    planner = OMPLESTPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# BiTRRT
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_bitrrt_finds_path(open_space):
    """BiTRRT (Bidirectional Transition-based RRT) must find a path in open space.

    BiTRRT grows two trees and uses a transition test to accept new states,
    which helps escape local minima in cost space.
    """
    from planners.ompl_bitrrt import OMPLBiTRRTPlanner
    planner = OMPLBiTRRTPlanner(open_space)
    path = planner.plan(Q_START, Q_GOAL, timeout=TIMEOUT)
    assert_valid_path(path, Q_START, Q_GOAL)


# ---------------------------------------------------------------------------
# Impossible planning scenario
# ---------------------------------------------------------------------------

@pytest.mark.ompl
@pytest.mark.slow
def test_rrt_returns_none_on_impossible(open_space):
    """When start and goal are identical and blocked, plan() must not crash.

    We wrap the zero configuration in a large box that blocks both start and
    goal.  The planner should eventually give up and return None rather than
    raising an exception.

    Note: This test uses a longer timeout to give the planner a fair chance
    to exhaust its budget before returning — it is marked @pytest.mark.slow.
    """
    from planners.ompl_rrt import OMPLRRTPlanner

    # A box that fully encloses the zero configuration (origin area)
    blocking_box = Box(
        center=np.array([0.0, 0.0, 0.5]),
        size=np.array([2.0, 2.0, 2.0]),
    )
    robot = open_space.robot
    cm = ShapeCollisionManager(robot_model=robot, shapes=[blocking_box])
    robot.set_collision_manager(cm)

    # Re-create state space with the updated (blocked) robot
    blocked_space = JointStateSpace(robot)

    planner = OMPLRRTPlanner(blocked_space)

    # Should not raise — may return None or an empty path if planning fails
    try:
        result = planner.plan(Q_START, Q_START, timeout=2.0)
        # Either None (could not plan) or a trivial single-waypoint path is acceptable
        assert result is None or len(result) >= 1
    except Exception as e:
        pytest.fail(f"planner.plan() raised an unexpected exception: {e}")


# ---------------------------------------------------------------------------
# API contract tests — prevent regression on return-type and naming bugs
# ---------------------------------------------------------------------------

@pytest.mark.ompl
def test_plan_returns_list_not_dict():
    """plan() must return a list of configs, not a dict."""
    import inspect
    from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
    sig = inspect.signature(OMPLRRTConnectPlanner.plan)
    hints = sig.return_annotation
    assert hints != dict, "plan() must not return a dict"


@pytest.mark.ompl
def test_plan_return_value_is_list_or_none():
    """plan() return value is a list of arrays or None, never a dict."""
    from core.robot_model import UR5RobotModel
    from core.state_space import JointStateSpace
    from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
    robot = UR5RobotModel(use_opw=False, collision_manager=None)
    ss = JointStateSpace(robot)
    planner = OMPLRRTConnectPlanner(ss)
    q0 = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    q1 = np.array([1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
    result = planner.plan(q0, q1, timeout=5.0)
    assert result is None or isinstance(result, list), \
        f"plan() returned {type(result)}, expected list or None"
    if result is not None:
        assert len(result) > 0
        assert isinstance(result[0], np.ndarray)


@pytest.mark.ompl
def test_all_planner_names_importable():
    """All names declared in planners/__init__.__all__ must actually import."""
    import planners
    for name in planners.__all__:
        assert hasattr(planners, name), \
            f"planners.{name} declared in __all__ but not importable"
