# Setup Guide

This guide gets you from a fresh checkout to running all demos.

---

## Prerequisites

* Python 3.10 or newer
* `pip`
* Git

---

## Quick Install

```bash
pip install -e .
```

This installs the package in editable mode, so edits to `core/`, `planners/`, etc. are immediately reflected without reinstalling.

---

## OMPL — Not pip-installable

OMPL (Open Motion Planning Library) cannot be installed with `pip`. It requires compiled C++ binaries with Python bindings. You have two options:

### Option 1: Docker (Fastest)

A pre-built image with OMPL already compiled and configured:

```bash
cd docker && make run
```

See [docker/README.md](../docker/README.md) for the full workflow including build steps, display passthrough for visualization, and development tips.

### Option 2: Build From Source

OMPL requires a CMake build with Python bindings enabled (`-DOMPL_BUILD_PYBINDINGS=ON`). Compilation takes 10–20 minutes depending on hardware.

Full instructions and system requirements are on the official OMPL site: https://ompl.kavrakilab.org/installation.html

---

## Optional Dependencies

`pybullet` is required for demo `06_pybullet_visualization.py`. Install it with:

```bash
pip install pybullet
```

All other demos work without it.

---

## Verify the Install

Run this one-liner to confirm the core robot model loads correctly:

```bash
python -c "from core.robot_model import UR5RobotModel; r = UR5RobotModel(); print(r.name, r.dof(), 'DOF')"
```

Expected output:

```
UR5 6 DOF
```

If you see an import error, check that you ran `pip install -e .` from the repo root.

---

## Running Tests

```bash
pytest tests/ -m "not ompl"
```

The `ompl` marker gates tests that require OMPL Python bindings. If you have OMPL installed (Docker or from source), run the full suite:

```bash
pytest tests/
```

---

## Running the Demos

Demos are numbered to be read and run in order. Each builds on concepts from the previous one.

```bash
python demos/01_plan_and_time.py
python demos/02_toppra_vs_ruckig.py
python demos/03_compare_planners.py
python demos/04_collision_demo.py
python demos/05_visualization_demo.py
python demos/06_pybullet_visualization.py   # requires pybullet
```

Start with `01` and work forward. The concepts compound.
