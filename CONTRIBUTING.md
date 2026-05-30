# Contributing

## Setup

```bash
pip install -e ".[dev]"
pre-commit install
```

## Running Tests

Quick run (no OMPL required):

```bash
pytest tests/ -m "not ompl"
```

Full suite (requires OMPL Python bindings — see `docs/00_setup.md`):

```bash
pytest tests/
```

## How to Add a New Planner

1. Subclass `BaseOMPLPlanner`, implement `_create_planner()`, and optionally `_configure_planner()`.
2. Add the new class to `planners/__init__.py`.
3. See any existing planner for a ~20-line example.

```python
class MyPlanner(BaseOMPLPlanner):
    def _create_planner(self):
        return og.MyAlgorithm(self.si)

    def _configure_planner(self):
        self.planner.setRange(0.1)
```

## How to Add a New Collision Shape

1. Subclass `CollisionShape`, implement `check_point(point)` and `get_type()`.
2. See `Box`, `Sphere`, `Cylinder` in `core/collision_manager.py` for reference.

```python
class Capsule(CollisionShape):
    def check_point(self, point: np.ndarray) -> bool:
        ...

    def get_type(self) -> str:
        return "capsule"
```

## Code Style

- Formatter/linter: [ruff](https://docs.astral.sh/ruff/)
- Line length: 100
- Run `ruff check --fix .` before committing, or let the pre-commit hook handle it.

## Documentation

Every major new concept should have a matching doc in `docs/`. Follow the existing numbered format (e.g. `docs/12_my_concept.md`) and include an **Exercises** section at the end.
