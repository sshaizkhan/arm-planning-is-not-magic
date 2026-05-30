# Changelog

All notable changes to this project will be documented in this file.

## [0.2.1] - 2026-05-30

- Fix post-merge workflow: retry PR detection up to 5x with 3s delay to handle GitHub API lag
- Backfill PR6 FK fix changelog entries into [0.2.0] section


## [0.2.0] - 2026-05-30

- Fix UR5 FK: replace OPW approximation with URDFKinematics that walks the exact URDF joint chain
- Add core/kinematics/urdf_kinematics.py with URDFKinematics class
- UR5RobotModel.fk() and link_positions() now match PyBullet at floating-point precision (0.0000mm error)
- Correct OPW parameters: a1=0.0, b=0.01615, offsets=[0,-π/2,0,0,0,0], sign_corrections=[1,1,-1,1,1,1]
- OPW retained as _opw_kinematics for IKSolver use in demos
- Add .github/PULL_REQUEST_TEMPLATE.md with description, type checkbox, changelog block, and testing checklist
- Add .github/workflows/post-merge.yml: auto-bumps version in pyproject.toml and appends changelog entries on merge to master
- Post-merge workflow supports manual retrigger via workflow_dispatch


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
