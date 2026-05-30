import pytest


def pytest_configure(config):
    config.addinivalue_line("markers", "ompl: tests requiring OMPL Python bindings")
    config.addinivalue_line("markers", "slow: tests that take more than 1 second")


def pytest_collection_modifyitems(config, items):
    try:
        from ompl import base  # noqa
    except ImportError:
        skip_ompl = pytest.mark.skip(reason="OMPL not installed")
        for item in items:
            if "ompl" in item.keywords:
                item.add_marker(skip_ompl)
