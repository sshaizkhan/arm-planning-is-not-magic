# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Changed
- Refactored all 7 OMPL planners to inherit `BaseOMPLPlanner` — each planner is now ~20 lines focused on the algorithm, not boilerplate
- `CollisionManager` is now a proper ABC with `@abstractmethod`
- `ShapeCollisionManager` now checks all 5 link positions (base, shoulder, elbow, wrist, EE), not just the end-effector
- Removed `SimpleSelfCollisionManager` placeholder
- Fixed monkey-patching anti-pattern in all demos — use `robot.set_collision_manager()` consistently
- Added `n_samples` parameter to `ToppraTimeParameterizer` (default: 100)
- Added `link_positions(q)` to `RobotModel` interface and UR5 implementation via OPW

### Added
- `tests/` — pytest suite with 5 test files covering all major modules
- `notebooks/` — 4 interactive Jupyter notebooks
- `docs/00_setup.md` — installation guide
- `docs/11_path_smoothing.md` — explains shortcutting and spline fitting
- Exercises sections in all 10 concept docs
- `CONTRIBUTING.md`, `CHANGELOG.md`
- GitHub Actions CI (lint + test without OMPL)
- `.pre-commit-config.yaml`
