"""
Smoke tests for demo scripts — verify they import cleanly and define main().

These tests catch:
- Wrong API calls (wrong kwargs, renamed methods, missing args)
- Missing imports
- Syntax errors

We do NOT run main() since demos are interactive. We import the module
with heavy deps (OMPL, meshcat, pybullet) mocked out.

Implementation note on ruckig:
  ruckig is a C extension. C extensions cannot be safely evicted from
  sys.modules and re-imported in the same process (CPython aborts on the
  second PyInit call). We therefore mock ruckig *before* any demo imports
  it, and never evict the mock. All other local packages (demos, planners,
  parameterization, core) are evicted and freshly imported per test so that
  cross-test state pollution is avoided.
"""
import importlib
import sys
import types
from unittest.mock import MagicMock, patch
import pytest


# ---------------------------------------------------------------------------
# Heavy-dep mocks
# ---------------------------------------------------------------------------

def _make_ompl_mock():
    """Build a minimal ompl mock that satisfies planner imports."""
    ompl = types.ModuleType("ompl")
    ompl.base = MagicMock()
    ompl.geometric = MagicMock()
    ompl.util = MagicMock()
    return ompl


def _make_meshcat_mock():
    mock = MagicMock()
    mock.Visualizer.return_value = MagicMock()
    mock.Visualizer.return_value.url.return_value = "http://127.0.0.1:7000/static/"
    return mock


def _make_ruckig_mock():
    """Mock the ruckig C extension so it is never imported as a real .so."""
    mock = MagicMock()
    # Ruckig classes accessed by RuckigTimeParameterizer
    mock.Ruckig = MagicMock()
    mock.InputParameter = MagicMock()
    mock.OutputParameter = MagicMock()
    mock.Result = MagicMock()
    mock.Result.Working = MagicMock()
    return mock


# Seed ruckig as a mock in sys.modules immediately at collection time,
# before any import of parameterization can load the real C extension.
if "ruckig" not in sys.modules:
    sys.modules["ruckig"] = _make_ruckig_mock()


# ---------------------------------------------------------------------------
# Module eviction and demo import helper
# ---------------------------------------------------------------------------

# Prefixes of *local* packages to evict before each demo import so that
# cross-test state doesn't bleed. C extensions (ruckig) are excluded.
_EVICT_PREFIXES = ("demos.", "planners", "parameterization", "core")


def _evict_local_modules():
    """Remove locally-owned modules from sys.modules for a clean re-import."""
    to_remove = [
        key for key in list(sys.modules)
        if any(key == p or key.startswith(p + ".") for p in _EVICT_PREFIXES)
    ]
    for key in to_remove:
        del sys.modules[key]


def _import_demo(module_name: str):
    """Import a demo module with heavy deps mocked. Returns the module."""
    _evict_local_modules()

    mocks = {
        "ompl": _make_ompl_mock(),
        "ompl.base": MagicMock(),
        "ompl.geometric": MagicMock(),
        "ompl.util": MagicMock(),
        "meshcat": _make_meshcat_mock(),
        "meshcat.geometry": MagicMock(),
        "meshcat.transformations": MagicMock(),
        "yourdfpy": MagicMock(),
        "pybullet": MagicMock(),
        "pybullet_data": MagicMock(),
        # ruckig is already seeded as a mock in sys.modules above;
        # re-applying it here ensures it stays mocked within the patch context.
        "ruckig": sys.modules["ruckig"],
    }
    with patch.dict(sys.modules, mocks):
        mod = importlib.import_module(module_name)
    return mod


# ---------------------------------------------------------------------------
# Demo list
# ---------------------------------------------------------------------------

DEMOS = [
    "demos.01_plan_and_time",
    "demos.02_toppra_vs_ruckig",
    "demos.03_compare_planners",
    "demos.04_collision_demo",
    "demos.05_visualization_demo",
    "demos.06_pybullet_visualization",
    "demos.07_multi_planner_visualization",
    "demos.08_meshcat_visualization",
    "demos.09_meshcat_collision_planning",
]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("demo", DEMOS)
def test_demo_imports_cleanly(demo):
    """Each demo must import without errors when heavy deps are mocked."""
    mod = _import_demo(demo)
    assert mod is not None


@pytest.mark.parametrize("demo", DEMOS)
def test_demo_has_main(demo):
    """Each demo must define a main() function."""
    mod = _import_demo(demo)
    assert hasattr(mod, "main"), f"{demo} must define main()"
    assert callable(mod.main), f"{demo}.main must be callable"
