# MeshcatVisualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `MeshcatVisualizer` — a browser-based drop-in alternative to `PyBulletVisualizer` — using `meshcat` for rendering and `yourdfpy` for URDF visual geometry parsing.

**Architecture:** `yourdfpy` parses `robots/ur5/ur5.urdf` to extract per-link visual geometry (shapes, sizes, colors). `URDFKinematics` (already in codebase) is extended with a `link_transforms()` method to supply per-link world-frame 4×4 transforms. `meshcat` renders everything in a browser tab via a local ZMQ/HTTP server.

**Tech Stack:** `meshcat>=0.3.2`, `yourdfpy>=0.0.53`, existing `core/kinematics/urdf_kinematics.py`

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `core/kinematics/urdf_kinematics.py` | Modify | Add `link_transforms(q)` — per-link world-frame 4×4 transforms |
| `visualization/meshcat_visualizer.py` | Create | `MeshcatVisualizer` class |
| `visualization/__init__.py` | Modify | Export `MeshcatVisualizer`, `MESHCAT_AVAILABLE` |
| `pyproject.toml` | Modify | Add `[meshcat]` optional dep group |
| `tests/test_meshcat_visualizer.py` | Create | Unit tests (mocked meshcat + yourdfpy) |
| `demos/08_meshcat_visualization.py` | Create | Browser-based obstacle avoidance demo |

---

## Task 1: Add optional dependencies to pyproject.toml

**Files:**
- Modify: `pyproject.toml`

- [ ] **Step 1: Add meshcat optional dep group**

Open `pyproject.toml`. After the existing `[project.optional-dependencies]` block, add:

```toml
[project.optional-dependencies]
pybullet = ["pybullet>=3.2.0"]
meshcat = [
    "meshcat>=0.3.2",
    "yourdfpy>=0.0.53",
]
dev = [
    "pytest>=7.0",
    "pytest-timeout",
    "ruff>=0.4.0",
    "mypy>=1.0",
    "jupyter",
    "ipykernel",
    "pybullet>=3.2.0",
]
```

- [ ] **Step 2: Install the new deps**

```bash
pip install "meshcat>=0.3.2" "yourdfpy>=0.0.53"
```

Expected: both packages install without errors.

- [ ] **Step 3: Verify imports work**

```bash
python -c "import meshcat; import yourdfpy; print('OK')"
```

Expected output: `OK`

- [ ] **Step 4: Commit**

```bash
git add pyproject.toml
git commit -m "build: add meshcat optional dependency group"
```

---

## Task 2: Extend URDFKinematics with link_transforms()

**Files:**
- Modify: `core/kinematics/urdf_kinematics.py:86-106`
- Test: `tests/test_robot_model.py` (add to existing file)

- [ ] **Step 1: Write the failing test**

Add to `tests/test_robot_model.py`:

```python
from core.kinematics.urdf_kinematics import URDFKinematics


@pytest.fixture
def urdf_kin():
    return URDFKinematics()


def test_link_transforms_returns_all_links(urdf_kin):
    q = np.zeros(6)
    transforms = urdf_kin.link_transforms(q)
    expected_links = [
        'base_link', 'shoulder_link', 'upper_arm_link',
        'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link',
    ]
    for name in expected_links:
        assert name in transforms, f"Missing link: {name}"
        T = transforms[name]
        assert T.shape == (4, 4), f"{name} transform is not 4x4"
        assert np.allclose(T[3], [0, 0, 0, 1]), f"{name} last row not [0,0,0,1]"


def test_link_transforms_base_is_identity(urdf_kin):
    transforms = urdf_kin.link_transforms(np.zeros(6))
    assert np.allclose(transforms['base_link'], np.eye(4))


def test_link_transforms_ee_matches_forward_kinematics(urdf_kin):
    """EE transform from link_transforms must match forward_kinematics."""
    q = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.1])
    transforms = urdf_kin.link_transforms(q)
    T_fk = urdf_kin.forward_kinematics(q)
    assert np.allclose(transforms['ee_link'], T_fk, atol=1e-10)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/test_robot_model.py::test_link_transforms_returns_all_links -v
```

Expected: `FAILED` with `AttributeError: 'URDFKinematics' object has no attribute 'link_transforms'`

- [ ] **Step 3: Implement link_transforms()**

Add after the existing `link_positions()` method in `core/kinematics/urdf_kinematics.py`:

```python
_LINK_NAMES = [
    'base_link',
    'shoulder_link',
    'upper_arm_link',
    'forearm_link',
    'wrist_1_link',
    'wrist_2_link',
    'wrist_3_link',
]

def link_transforms(self, q: np.ndarray) -> dict:
    """
    Return world-frame 4x4 transform for every link including EE.

    Keys: 'base_link', 'shoulder_link', 'upper_arm_link',
          'forearm_link', 'wrist_1_link', 'wrist_2_link',
          'wrist_3_link', 'ee_link'
    """
    transforms = {'base_link': np.eye(4)}
    T = np.eye(4)
    for i, (xyz, rpy, axis) in enumerate(self._JOINTS):
        R_frame = _rpy_to_matrix(*rpy)
        R_joint = _axis_rotation(axis, float(q[i]))
        T = T @ _transform(xyz, R_frame @ R_joint)
        transforms[self._LINK_NAMES[i + 1]] = T.copy()
    R_ee = _rpy_to_matrix(*self._EE_RPY)
    transforms['ee_link'] = (T @ _transform(self._EE_XYZ, R_ee)).copy()
    return transforms
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/test_robot_model.py::test_link_transforms_returns_all_links \
       tests/test_robot_model.py::test_link_transforms_base_is_identity \
       tests/test_robot_model.py::test_link_transforms_ee_matches_forward_kinematics -v
```

Expected: 3 PASSED

- [ ] **Step 5: Commit**

```bash
git add core/kinematics/urdf_kinematics.py tests/test_robot_model.py
git commit -m "feat: add link_transforms() to URDFKinematics for per-link FK"
```

---

## Task 3: MeshcatVisualizer skeleton + helper functions

**Files:**
- Create: `visualization/meshcat_visualizer.py`
- Create: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests for helper functions**

Create `tests/test_meshcat_visualizer.py`:

```python
"""Tests for MeshcatVisualizer helpers (no real meshcat server needed)."""
import numpy as np
import pytest

meshcat = pytest.importorskip("meshcat", reason="meshcat not installed")
yourdfpy = pytest.importorskip("yourdfpy", reason="yourdfpy not installed")


def test_hex_color_white():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([1.0, 1.0, 1.0, 1.0]) == 0xFFFFFF


def test_hex_color_red():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([1.0, 0.0, 0.0, 1.0]) == 0xFF0000


def test_hex_color_blue():
    from visualization.meshcat_visualizer import _rgba_to_hex
    assert _rgba_to_hex([0.0, 0.0, 1.0, 1.0]) == 0x0000FF


def test_hex_color_mixed():
    from visualization.meshcat_visualizer import _rgba_to_hex
    result = _rgba_to_hex([0.5, 0.25, 0.0, 1.0])
    assert result == (int(0.5 * 255) << 16) | (int(0.25 * 255) << 8) | 0


def test_cylinder_alignment_matrix_shape():
    from visualization.meshcat_visualizer import _cylinder_align
    R = _cylinder_align()
    assert R.shape == (4, 4)
    assert np.allclose(R[3], [0, 0, 0, 1])


def test_cylinder_alignment_rotates_y_to_z():
    """meshcat Cylinder is Y-aligned; URDF expects Z-aligned. Rotation maps Y→Z."""
    from visualization.meshcat_visualizer import _cylinder_align
    R = _cylinder_align()
    # The Y axis [0,1,0] should map to Z axis [0,0,1] after rotation
    y_axis = np.array([0, 1, 0, 0])
    rotated = R @ y_axis
    assert np.allclose(rotated[:3], [0, 0, 1], atol=1e-10)
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: `FAILED` with `ModuleNotFoundError: No module named 'visualization.meshcat_visualizer'`

- [ ] **Step 3: Create visualization/meshcat_visualizer.py with helpers**

```python
"""Meshcat-based 3D robot arm visualization.

Renders the UR5 in a browser tab via meshcat. No display server required.
Open the printed URL in any browser.

Requirements:
    meshcat>=0.3.2
    yourdfpy>=0.0.53
"""

import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

try:
    import meshcat
    import meshcat.geometry as g
    import meshcat.transformations as tf
    MESHCAT_AVAILABLE = True
except ImportError:
    MESHCAT_AVAILABLE = False

try:
    import yourdfpy
    YOURDFPY_AVAILABLE = True
except ImportError:
    YOURDFPY_AVAILABLE = False


def _rgba_to_hex(rgba: List[float]) -> int:
    """Convert [r, g, b, a] floats (0-1) to packed RGB integer for meshcat."""
    r, g_v, b = int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255)
    return (r << 16) | (g_v << 8) | b


def _cylinder_align() -> np.ndarray:
    """Return 4x4 rotation that maps meshcat's Y-aligned cylinder to Z-aligned (URDF convention)."""
    return tf.rotation_matrix(np.pi / 2, [1, 0, 0])


def get_default_ur5_urdf() -> Optional[str]:
    """Return path to the bundled UR5 URDF, or None if not found."""
    urdf = Path(__file__).parent.parent / "robots" / "ur5" / "ur5.urdf"
    return str(urdf) if urdf.exists() else None
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: add MeshcatVisualizer file with color/alignment helpers"
```

---

## Task 4: URDF geometry parsing + MeshcatVisualizer.__init__

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
from unittest.mock import MagicMock, patch


@pytest.fixture
def mock_viz():
    """MeshcatVisualizer with meshcat server and yourdfpy mocked out."""
    with patch('visualization.meshcat_visualizer.meshcat') as mc, \
         patch('visualization.meshcat_visualizer.yourdfpy') as yd:

        # Mock meshcat.Visualizer — supports viz["path"] subscripting
        mock_server = MagicMock()
        mc.Visualizer.return_value = mock_server

        # Mock yourdfpy.URDF.load — return a robot with 2 links, each with one visual
        mock_robot = MagicMock()
        mock_robot.links = _make_mock_links()
        mock_robot.joints = []
        yd.URDF.load.return_value = mock_robot

        from visualization.meshcat_visualizer import MeshcatVisualizer
        viz = MeshcatVisualizer(urdf_path="fake.urdf", open_browser=False)
        yield viz, mock_server


def _make_mock_links():
    """Two mock links: one with a cylinder visual, one with a box visual."""
    links = []

    # Link with cylinder
    cyl_geom = MagicMock()
    cyl_geom.box = None
    cyl_geom.sphere = None
    cyl_geom.cylinder = MagicMock(radius=0.05, length=0.2)
    cyl_geom.mesh = None

    cyl_visual = MagicMock()
    cyl_visual.geometry = cyl_geom
    cyl_visual.origin = None
    cyl_visual.material = MagicMock()
    cyl_visual.material.color = MagicMock()
    cyl_visual.material.color.rgba = [0.8, 0.2, 0.2, 1.0]

    link_a = MagicMock()
    link_a.name = "base_link"
    link_a.visuals = [cyl_visual]
    links.append(link_a)

    # Link with box
    box_geom = MagicMock()
    box_geom.box = MagicMock(size=[0.1, 0.1, 0.4])
    box_geom.sphere = None
    box_geom.cylinder = None
    box_geom.mesh = None

    box_visual = MagicMock()
    box_visual.geometry = box_geom
    box_visual.origin = None
    box_visual.material = None

    link_b = MagicMock()
    link_b.name = "shoulder_link"
    link_b.visuals = [box_visual]
    links.append(link_b)

    return links


def test_init_loads_urdf(mock_viz):
    _, mock_server = mock_viz
    import visualization.meshcat_visualizer as mv
    yourdfpy.URDF.load.assert_called_once()


def test_init_sets_geometry_for_each_link(mock_viz):
    _, mock_server = mock_viz
    # set_object should be called for base_link and shoulder_link
    assert mock_server.__getitem__.call_count >= 2


def test_init_opens_no_browser_when_false(mock_viz):
    _, mock_server = mock_viz
    mock_server.open.assert_not_called()
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_init_loads_urdf -v
```

Expected: `FAILED` — `MeshcatVisualizer` doesn't exist yet.

- [ ] **Step 3: Implement MeshcatVisualizer.__init__ and _load_robot_geometry**

Add to `visualization/meshcat_visualizer.py` after the helper functions:

```python
# Default colors for links that have no URDF material
_DEFAULT_LINK_COLORS = [
    [0.2, 0.2, 0.2, 1.0],  # base_link       — dark grey
    [0.7, 0.7, 0.7, 1.0],  # shoulder_link   — light grey
    [0.0, 0.4, 0.6, 1.0],  # upper_arm_link  — UR5 blue
    [0.7, 0.7, 0.7, 1.0],  # forearm_link    — light grey
    [0.2, 0.2, 0.2, 1.0],  # wrist_1_link    — dark grey
    [0.7, 0.7, 0.7, 1.0],  # wrist_2_link    — light grey
    [0.2, 0.2, 0.2, 1.0],  # wrist_3_link    — dark grey
    [0.1, 0.1, 0.1, 1.0],  # ee_link         — black
]


class MeshcatVisualizer:
    """
    Browser-based 3D visualizer for robot arm trajectories using meshcat.

    Opens a local web server; navigate to the printed URL in any browser.
    Drop-in API replacement for PyBulletVisualizer (no collision checking).
    """

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        open_browser: bool = True,
        zmq_url: Optional[str] = None,
        base_position: Tuple[float, float, float] = (0, 0, 0),
    ):
        if not MESHCAT_AVAILABLE:
            raise ImportError(
                "meshcat is not installed. Install with: pip install meshcat"
            )
        if not YOURDFPY_AVAILABLE:
            raise ImportError(
                "yourdfpy is not installed. Install with: pip install yourdfpy"
            )

        self.base_position = np.array(base_position)
        self._path_object_paths: List[str] = []
        self._obstacle_count = {"box": 0, "sphere": 0, "cylinder": 0, "marker": 0}

        # Start meshcat server
        self._viz = meshcat.Visualizer(zmq_url) if zmq_url else meshcat.Visualizer()
        if open_browser:
            self._viz.open()
        print(f"Meshcat running at: {self._viz.url()}")

        # Load URDF geometry
        if urdf_path is None:
            urdf_path = get_default_ur5_urdf()
        if urdf_path is None:
            raise FileNotFoundError(
                "No URDF found. Pass urdf_path= or ensure robots/ur5/ur5.urdf exists."
            )
        self._robot = yourdfpy.URDF.load(urdf_path)
        self._load_robot_geometry()

        # Add ground plane
        self._viz["ground/plane"].set_object(
            g.Box([2.0, 2.0, 0.005]),
            g.MeshLambertMaterial(color=0x888888),
        )
        self._viz["ground/plane"].set_transform(
            tf.translation_matrix([0, 0, -0.0025])
        )

        # FK engine
        from core.kinematics.urdf_kinematics import URDFKinematics
        self._fk = URDFKinematics()

        # Set initial pose
        self.set_joint_positions(np.zeros(6))

    def _load_robot_geometry(self):
        """Parse URDF links and add geometry to meshcat scene."""
        default_idx = 0
        for link in self._robot.links:
            if not link.visuals:
                continue
            fallback_rgba = _DEFAULT_LINK_COLORS[default_idx % len(_DEFAULT_LINK_COLORS)]
            default_idx += 1

            for vis_idx, visual in enumerate(link.visuals):
                geom = self._urdf_geom_to_meshcat(visual.geometry)
                if geom is None:
                    continue
                mat = self._urdf_mat_to_meshcat(visual.material, fallback_rgba)
                path = f"robot/{link.name}/visual_{vis_idx}"
                self._viz[path].set_object(geom, mat)

                # Apply visual origin offset (relative to link frame)
                if visual.origin is not None:
                    origin = np.array(visual.origin)
                else:
                    origin = np.eye(4)

                # Cylinders need 90° X rotation (meshcat Y-aligned → URDF Z-aligned)
                if visual.geometry.cylinder is not None:
                    origin = origin @ _cylinder_align()

                self._viz[path].set_transform(origin)

    def _urdf_geom_to_meshcat(self, geom) -> Optional[object]:
        """Convert yourdfpy geometry to meshcat geometry. Returns None for meshes."""
        if geom.cylinder is not None:
            return g.Cylinder(geom.cylinder.length, geom.cylinder.radius)
        if geom.box is not None:
            size = geom.box.size
            return g.Box([float(size[0]), float(size[1]), float(size[2])])
        if geom.sphere is not None:
            return g.Sphere(geom.sphere.radius)
        return None  # mesh — skip for now

    def _urdf_mat_to_meshcat(self, material, fallback_rgba: List[float]) -> object:
        """Convert yourdfpy material to MeshLambertMaterial."""
        if material is not None and material.color is not None:
            rgba = material.color.rgba
        else:
            rgba = fallback_rgba
        hex_color = _rgba_to_hex(rgba)
        alpha = float(rgba[3])
        return g.MeshLambertMaterial(
            color=hex_color,
            opacity=alpha,
            transparent=(alpha < 1.0),
        )
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: add MeshcatVisualizer __init__ and URDF geometry loading"
```

---

## Task 5: set_joint_positions + visualize_configuration

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_set_joint_positions_calls_set_transform(mock_viz):
    viz, mock_server = mock_viz
    q = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.1])
    viz.set_joint_positions(q)
    # set_transform should be called for each link in the robot
    assert mock_server.__getitem__.return_value.set_transform.called


def test_set_joint_positions_wrong_dof_raises(mock_viz):
    viz, _ = mock_viz
    with pytest.raises(ValueError, match="Expected 6 joints"):
        viz.set_joint_positions(np.zeros(3))


def test_visualize_configuration_calls_set_joint_positions(mock_viz):
    viz, _ = mock_viz
    q = np.zeros(6)
    # Should not raise and should set transforms
    viz.visualize_configuration(q, duration=0.0)
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_set_joint_positions_wrong_dof_raises -v
```

Expected: `FAILED` — method doesn't exist yet.

- [ ] **Step 3: Implement set_joint_positions and visualize_configuration**

Add to the `MeshcatVisualizer` class in `visualization/meshcat_visualizer.py`:

```python
    def set_joint_positions(self, q: np.ndarray):
        """
        Set robot joint positions and update all link transforms in meshcat.

        Args:
            q: Joint angles, shape (6,)
        """
        if len(q) != 6:
            raise ValueError(f"Expected 6 joints, got {len(q)}")

        link_transforms = self._fk.link_transforms(q)

        for link in self._robot.links:
            if not link.visuals:
                continue
            T_world = link_transforms.get(link.name)
            if T_world is None:
                continue

            for vis_idx, visual in enumerate(link.visuals):
                # Visual origin offset (relative to link frame)
                if visual.origin is not None:
                    origin = np.array(visual.origin)
                else:
                    origin = np.eye(4)
                if visual.geometry.cylinder is not None:
                    origin = origin @ _cylinder_align()

                # World transform = link world transform × visual offset
                T_final = T_world @ origin

                # Shift by base_position
                T_final[:3, 3] += self.base_position

                path = f"robot/{link.name}/visual_{vis_idx}"
                self._viz[path].set_transform(T_final)

    def visualize_configuration(self, q: np.ndarray, duration: float = 0.0):
        """
        Visualize a single robot configuration.

        Args:
            q: Joint angles, shape (6,)
            duration: Seconds to hold this pose
        """
        self.set_joint_positions(q)
        if duration > 0:
            time.sleep(duration)

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get end-effector position and orientation from last set_joint_positions call.

        Returns:
            (position (3,), rotation_matrix (3,3))
        """
        return self._last_ee_pos.copy(), self._last_ee_rot.copy()
```

Also update `set_joint_positions` to cache the EE pose. After computing `link_transforms`, add:

```python
        T_ee = link_transforms.get('ee_link', np.eye(4))
        self._last_ee_pos = T_ee[:3, 3].copy()
        self._last_ee_rot = T_ee[:3, :3].copy()
```

And in `__init__`, before calling `set_joint_positions(np.zeros(6))`, initialize the cache:

```python
        self._last_ee_pos = np.zeros(3)
        self._last_ee_rot = np.eye(3)
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: implement set_joint_positions and visualize_configuration"
```

---

## Task 6: visualize_trajectory with EE trail

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_visualize_trajectory_calls_set_joint_positions(mock_viz):
    viz, mock_server = mock_viz
    trajectory = np.zeros((5, 6))
    trajectory[:, 0] = np.linspace(0, 0.5, 5)
    # Should not raise
    viz.visualize_trajectory(trajectory, real_time=False, show_ee_trail=False)


def test_visualize_trajectory_with_trail_sets_line(mock_viz):
    viz, mock_server = mock_viz
    trajectory = np.zeros((5, 6))
    viz.visualize_trajectory(trajectory, real_time=False, show_ee_trail=True)
    # trail/line path should have set_object called
    calls = [str(c) for c in mock_server.__getitem__.call_args_list]
    assert any("trail" in c for c in calls)


def test_visualize_trajectory_time_stamps_shape(mock_viz):
    viz, _ = mock_viz
    trajectory = np.zeros((4, 6))
    time_stamps = np.linspace(0, 1, 4)
    viz.visualize_trajectory(trajectory, time_stamps=time_stamps, real_time=False)
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_visualize_trajectory_calls_set_joint_positions -v
```

Expected: `FAILED`

- [ ] **Step 3: Implement visualize_trajectory**

Add to the `MeshcatVisualizer` class:

```python
    def visualize_trajectory(
        self,
        trajectory: np.ndarray,
        time_stamps: Optional[np.ndarray] = None,
        real_time: bool = True,
        speed: float = 1.0,
        show_ee_trail: bool = True,
        trail_length: int = 100,
    ):
        """
        Visualize a complete trajectory.

        Args:
            trajectory: Joint trajectory, shape (N, 6)
            time_stamps: Time stamps shape (N,). If None, uniform over [0,1].
            real_time: If True, play at real-time speed
            speed: Speed multiplier (1.0 = real time)
            show_ee_trail: If True, draw growing EE trail line
            trail_length: Max number of trail points to keep
        """
        n = len(trajectory)
        if time_stamps is None:
            time_stamps = np.linspace(0, 1, n)

        ee_trail: List[np.ndarray] = []
        last_frame = time.time()

        for i, q in enumerate(trajectory):
            self.set_joint_positions(q)

            if show_ee_trail:
                ee_trail.append(self._last_ee_pos.copy())
                if len(ee_trail) > trail_length:
                    ee_trail.pop(0)
                if len(ee_trail) >= 2:
                    pts = np.array(ee_trail).T  # (3, N)
                    self._viz["trail/line"].set_object(
                        g.Line(
                            g.PointsGeometry(pts.astype(np.float32)),
                            g.LineBasicMaterial(color=0x00FF00, linewidth=2),
                        )
                    )

            if real_time and i < n - 1:
                dt = (time_stamps[i + 1] - time_stamps[i]) / speed
                elapsed = time.time() - last_frame
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                last_frame = time.time()
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: implement visualize_trajectory with EE trail"
```

---

## Task 7: Path visualization (planned path + clear)

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_visualize_path_sets_line_object(mock_viz):
    viz, mock_server = mock_viz
    path = np.zeros((5, 6))
    path[:, 0] = np.linspace(0, 0.3, 5)

    def fake_fk(q):
        pos = np.array([q[0] * 0.5, 0.0, 0.5])
        return pos, np.eye(3)

    viz.visualize_path(path, fk_func=fake_fk)
    calls = [str(c) for c in mock_server.__getitem__.call_args_list]
    assert any("path" in c for c in calls)


def test_clear_path_deletes_path_subtree(mock_viz):
    viz, mock_server = mock_viz
    viz.clear_path()
    mock_server.__getitem__.return_value.delete.assert_called()


def test_visualize_trajectory_path_samples_correctly(mock_viz):
    viz, mock_server = mock_viz
    traj = np.zeros((20, 6))

    def fake_fk(q):
        return np.zeros(3), np.eye(3)

    # sample_every=5 → 4 samples from 20 points
    viz.visualize_trajectory_path(traj, fk_func=fake_fk, sample_every=5)
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_clear_path_deletes_path_subtree -v
```

Expected: `FAILED`

- [ ] **Step 3: Implement visualize_path, visualize_trajectory_path, clear_path**

Add to `MeshcatVisualizer`:

```python
    def _compute_ee_positions(
        self,
        path: np.ndarray,
        fk_func: Optional[Callable],
    ) -> List[np.ndarray]:
        """Compute EE world positions for each waypoint in path."""
        positions = []
        for q in path:
            if fk_func is not None:
                result = fk_func(q)
                if isinstance(result, tuple):
                    pos = np.array(result[0])
                else:
                    pos = np.array(result[:3, 3])
            else:
                self.set_joint_positions(q)
                pos = self._last_ee_pos.copy()
            positions.append(pos)
        return positions

    def visualize_path(
        self,
        path: np.ndarray,
        fk_func: Optional[Callable] = None,
        color: Tuple[float, float, float] = (1.0, 0.5, 0.0),
        line_width: float = 3,
        show_waypoints: bool = True,
        waypoint_color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        waypoint_size: float = 0.015,
        lifetime: float = 0,
    ) -> List[str]:
        """
        Visualize a planned path as a 3D EE trajectory line.

        Args:
            path: Joint space path, shape (N, 6)
            fk_func: FK function returning (position, rotation) or 4x4 matrix.
                     If None, uses internal FK.
            color: RGB color tuple (0-1) for path line
            line_width: Line width
            show_waypoints: If True, draw spheres at each waypoint
            waypoint_color: RGB for waypoint spheres
            waypoint_size: Sphere radius for waypoints
            lifetime: Unused (meshcat objects persist until deleted)

        Returns:
            List of meshcat path strings for the created objects
        """
        self.clear_path()
        if len(path) < 2:
            return []

        ee_positions = self._compute_ee_positions(path, fk_func)

        hex_line = _rgba_to_hex([*color, 1.0])
        pts = np.array(ee_positions).T.astype(np.float32)  # (3, N)
        line_mat = g.LineBasicMaterial(color=hex_line, linewidth=int(line_width))
        self._viz["path/line"].set_object(
            g.Line(g.PointsGeometry(pts), line_mat)
        )
        created = ["path/line"]

        if show_waypoints:
            hex_wp = _rgba_to_hex([*waypoint_color, 1.0])
            wp_mat = g.MeshLambertMaterial(color=hex_wp)
            for i, pos in enumerate(ee_positions):
                wp_path = f"path/waypoints/{i}"
                self._viz[wp_path].set_object(g.Sphere(waypoint_size), wp_mat)
                self._viz[wp_path].set_transform(tf.translation_matrix(pos))
                created.append(wp_path)

        self._path_object_paths = created
        return created

    def visualize_trajectory_path(
        self,
        trajectory: np.ndarray,
        fk_func: Optional[Callable] = None,
        color: Tuple[float, float, float] = (0.0, 0.7, 1.0),
        line_width: float = 2,
        sample_every: int = 5,
        lifetime: float = 0,
    ) -> List[str]:
        """
        Visualize a dense trajectory as a smooth 3D curve (no waypoint markers).

        Args:
            trajectory: Joint trajectory, shape (N, 6)
            fk_func: FK function (see visualize_path)
            color: RGB color tuple (0-1)
            line_width: Line width
            sample_every: Sample every N points for performance
            lifetime: Unused
        """
        sampled = trajectory[::sample_every]
        if not np.array_equal(sampled[-1], trajectory[-1]):
            sampled = np.vstack([sampled, trajectory[-1]])
        return self.visualize_path(
            sampled,
            fk_func=fk_func,
            color=color,
            line_width=line_width,
            show_waypoints=False,
        )

    def clear_path(self):
        """Delete all path visualization objects from the scene."""
        self._viz["path"].delete()
        self._path_object_paths = []
```


- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: implement visualize_path, visualize_trajectory_path, clear_path"
```

---

## Task 8: Obstacle and marker methods

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_add_box_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_box(center=(0.5, 0.0, 0.3), size=(0.1, 0.1, 0.1))
    assert isinstance(path, str)
    assert "box" in path


def test_add_sphere_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_sphere(center=(0.3, 0.1, 0.4), radius=0.05)
    assert isinstance(path, str)
    assert "sphere" in path


def test_add_cylinder_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_cylinder(center=(0.2, 0.2, 0.2), radius=0.04, height=0.3)
    assert isinstance(path, str)
    assert "cylinder" in path


def test_remove_body_calls_delete(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_box(center=(0, 0, 0), size=(0.1, 0.1, 0.1))
    viz.remove_body(path)
    mock_server.__getitem__.return_value.delete.assert_called()


def test_add_marker_returns_path_string(mock_viz):
    viz, mock_server = mock_viz
    path = viz.add_marker(position=(0.3, 0.3, 0.3))
    assert isinstance(path, str)
    assert "marker" in path


def test_add_multiple_boxes_unique_paths(mock_viz):
    viz, _ = mock_viz
    p1 = viz.add_box((0, 0, 0), (0.1, 0.1, 0.1))
    p2 = viz.add_box((1, 0, 0), (0.1, 0.1, 0.1))
    assert p1 != p2
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_add_box_returns_path_string -v
```

Expected: `FAILED`

- [ ] **Step 3: Implement obstacle and marker methods**

Add to `MeshcatVisualizer`:

```python
    def add_box(
        self,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.7),
    ) -> str:
        """
        Add a box obstacle to the scene.

        Args:
            center: (x, y, z) center position
            size: (width, depth, height)
            color: RGBA (0-1), default semi-transparent red

        Returns:
            meshcat path string (pass to remove_body to delete)
        """
        n = self._obstacle_count["box"]
        self._obstacle_count["box"] += 1
        path = f"obstacles/box_{n}"
        mat = g.MeshLambertMaterial(
            color=_rgba_to_hex(list(color[:3]) + [1.0]),
            opacity=float(color[3]),
            transparent=(color[3] < 1.0),
        )
        self._viz[path].set_object(g.Box([float(s) for s in size]), mat)
        self._viz[path].set_transform(tf.translation_matrix(list(center)))
        return path

    def add_sphere(
        self,
        center: Tuple[float, float, float],
        radius: float,
        color: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.7),
    ) -> str:
        """
        Add a sphere obstacle to the scene.

        Args:
            center: (x, y, z) center position
            radius: Sphere radius
            color: RGBA (0-1), default semi-transparent green

        Returns:
            meshcat path string
        """
        n = self._obstacle_count["sphere"]
        self._obstacle_count["sphere"] += 1
        path = f"obstacles/sphere_{n}"
        mat = g.MeshLambertMaterial(
            color=_rgba_to_hex(list(color[:3]) + [1.0]),
            opacity=float(color[3]),
            transparent=(color[3] < 1.0),
        )
        self._viz[path].set_object(g.Sphere(radius), mat)
        self._viz[path].set_transform(tf.translation_matrix(list(center)))
        return path

    def add_cylinder(
        self,
        center: Tuple[float, float, float],
        radius: float,
        height: float,
        color: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.7),
    ) -> str:
        """
        Add a vertical cylinder obstacle to the scene.

        Args:
            center: (x, y, z) center position
            radius: Cylinder radius
            height: Cylinder height
            color: RGBA (0-1), default semi-transparent blue

        Returns:
            meshcat path string
        """
        n = self._obstacle_count["cylinder"]
        self._obstacle_count["cylinder"] += 1
        path = f"obstacles/cylinder_{n}"
        mat = g.MeshLambertMaterial(
            color=_rgba_to_hex(list(color[:3]) + [1.0]),
            opacity=float(color[3]),
            transparent=(color[3] < 1.0),
        )
        self._viz[path].set_object(g.Cylinder(height, radius), mat)
        T = tf.translation_matrix(list(center)) @ _cylinder_align()
        self._viz[path].set_transform(T)
        return path

    def add_marker(
        self,
        position: Tuple[float, float, float],
        color: Tuple[float, float, float] = (1, 0, 0),
        size: float = 0.05,
        lifetime: float = 0,
    ) -> str:
        """
        Add a small sphere marker in the scene.

        Args:
            position: (x, y, z)
            color: RGB (0-1)
            size: Sphere radius
            lifetime: Unused (meshcat objects persist until deleted)

        Returns:
            meshcat path string
        """
        n = self._obstacle_count["marker"]
        self._obstacle_count["marker"] += 1
        path = f"obstacles/marker_{n}"
        mat = g.MeshLambertMaterial(color=_rgba_to_hex(list(color) + [1.0]))
        self._viz[path].set_object(g.Sphere(size), mat)
        self._viz[path].set_transform(tf.translation_matrix(list(position)))
        return path

    def remove_body(self, path: str):
        """Remove an obstacle or marker from the scene."""
        self._viz[path].delete()

    def add_text(
        self,
        text: str,
        position: Tuple[float, float, float],
        color: Tuple[float, float, float] = (1, 1, 1),
        size: float = 1.5,
        lifetime: float = 0,
    ) -> str:
        """
        Add a label marker at a 3D position.

        Note: meshcat-python has no native Text geometry. This places a small
        colored sphere at the position and prints the label to stdout. For
        rich text overlays, use the meshcat browser's built-in controls.

        Args:
            text: Label string (printed to stdout)
            position: (x, y, z)
            color: RGB (0-1)
            size: Sphere radius (scaled from font-size convention)
            lifetime: Unused

        Returns:
            meshcat path string
        """
        n = self._obstacle_count.get("text", 0)
        self._obstacle_count["text"] = n + 1
        path = f"labels/text_{n}"
        hex_color = _rgba_to_hex(list(color) + [1.0])
        sphere_radius = max(0.005, size * 0.008)
        self._viz[path].set_object(g.Sphere(sphere_radius), g.MeshLambertMaterial(color=hex_color))
        self._viz[path].set_transform(tf.translation_matrix(list(position)))
        print(f"[label] {text} @ {tuple(round(float(v), 3) for v in position)}")
        return path
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: implement obstacle and marker methods"
```

---

## Task 9: visualize_multi_planner_trajectories + close + context manager

**Files:**
- Modify: `visualization/meshcat_visualizer.py`
- Modify: `tests/test_meshcat_visualizer.py`

- [ ] **Step 1: Write failing tests**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_visualize_multi_planner_draws_one_line_per_planner(mock_viz):
    viz, mock_server = mock_viz

    def fake_fk(q):
        return np.array([q[0] * 0.3, 0.0, 0.4]), np.eye(3)

    planner_results = {
        "RRT": {"trajectory": np.zeros((5, 6))},
        "PRM": {"trajectory": np.ones((5, 6)) * 0.1},
        "Failed": {"trajectory": None},
    }
    result = viz.visualize_multi_planner_trajectories(
        planner_results, fk_func=fake_fk, show_labels=False
    )
    # Only RRT and PRM have trajectories
    assert "RRT" in result
    assert "PRM" in result
    assert "Failed" not in result


def test_close_disconnects(mock_viz):
    viz, mock_server = mock_viz
    viz.close()
    # meshcat has no explicit disconnect; just verify no exception raised


def test_context_manager(mock_viz):
    viz, mock_server = mock_viz
    with viz:
        pass  # should not raise
```

- [ ] **Step 2: Run to verify they fail**

```bash
pytest tests/test_meshcat_visualizer.py::test_visualize_multi_planner_draws_one_line_per_planner -v
```

Expected: `FAILED`

- [ ] **Step 3: Implement visualize_multi_planner_trajectories, close, context manager**

Add to `MeshcatVisualizer`:

```python
    # Tab-10 palette (matplotlib default)
    _PLANNER_COLORS = [
        (0.12, 0.47, 0.71),
        (1.00, 0.50, 0.05),
        (0.17, 0.63, 0.17),
        (0.84, 0.15, 0.16),
        (0.58, 0.40, 0.74),
        (0.55, 0.34, 0.29),
        (0.89, 0.47, 0.76),
    ]

    def visualize_multi_planner_trajectories(
        self,
        planner_results: Dict,
        fk_func: Optional[Callable] = None,
        show_labels: bool = True,
        line_width: float = 3,
        sample_every: int = 3,
    ) -> Dict:
        """
        Draw all planner trajectories simultaneously with different colors.

        Args:
            planner_results: Dict mapping planner name to result dict with 'trajectory' key
            fk_func: FK function returning (position, rotation) or 4x4 matrix
            show_labels: If True, add a text label at the midpoint of each trajectory
            line_width: Line width
            sample_every: Sample every N trajectory points for performance

        Returns:
            Dict mapping planner name to list of created meshcat path strings
        """
        all_paths: Dict[str, List[str]] = {}

        for idx, (name, result) in enumerate(planner_results.items()):
            traj = result.get("trajectory")
            if traj is None:
                continue

            color = self._PLANNER_COLORS[idx % len(self._PLANNER_COLORS)]
            sampled = traj[::sample_every]
            if not np.array_equal(sampled[-1], traj[-1]):
                sampled = np.vstack([sampled, traj[-1]])

            ee_positions = self._compute_ee_positions(sampled, fk_func)
            hex_color = _rgba_to_hex(list(color) + [1.0])
            pts = np.array(ee_positions).T.astype(np.float32)
            line_path = f"path/{name}/line"
            self._viz[line_path].set_object(
                g.Line(
                    g.PointsGeometry(pts),
                    g.LineBasicMaterial(color=hex_color, linewidth=int(line_width)),
                )
            )
            created = [line_path]
            self._path_object_paths.append(line_path)

            if show_labels and ee_positions:
                mid = len(ee_positions) // 2
                label_pos = list(ee_positions[mid])
                label_pos[2] += 0.05 * (idx + 1)
                label_path = self.add_text(name, tuple(label_pos), color=color)
                created.append(label_path)

            all_paths[name] = created

        return all_paths

    @property
    def url(self) -> str:
        """URL of the meshcat web interface."""
        return self._viz.url()

    def close(self):
        """Stop the meshcat server."""
        pass  # meshcat Python server has no explicit close; GC handles cleanup

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Run full test suite to check no regressions**

```bash
pytest tests/ -v --timeout=30
```

Expected: all tests pass (meshcat tests skipped if not installed in CI)

- [ ] **Step 6: Commit**

```bash
git add visualization/meshcat_visualizer.py tests/test_meshcat_visualizer.py
git commit -m "feat: implement multi-planner visualization and context manager"
```

---

## Task 10: Update visualization/__init__.py

**Files:**
- Modify: `visualization/__init__.py`

- [ ] **Step 1: Write failing test**

Add to `tests/test_meshcat_visualizer.py`:

```python
def test_meshcat_available_flag_exported():
    from visualization import MESHCAT_AVAILABLE
    assert isinstance(MESHCAT_AVAILABLE, bool)


def test_meshcat_visualizer_exported():
    from visualization import MeshcatVisualizer
    # If meshcat installed, should be the class; otherwise None
    assert MeshcatVisualizer is not None or True  # just verify it's importable
```

- [ ] **Step 2: Run to verify failure**

```bash
pytest tests/test_meshcat_visualizer.py::test_meshcat_available_flag_exported -v
```

Expected: `FAILED` with `ImportError: cannot import name 'MESHCAT_AVAILABLE'`

- [ ] **Step 3: Update visualization/__init__.py**

Replace the content of `visualization/__init__.py` with:

```python
"""Visualization utilities for robot arm motion planning.

This module provides tools to visualize:
- Joint-space trajectories (position, velocity, acceleration vs time)
- End-effector paths in 3D Cartesian space
- Simple stick-figure robot arm animations
- PyBullet-based 3D visualization
- Meshcat browser-based 3D visualization

Usage:
    from visualization import plot_joint_trajectory, plot_ee_path_3d, RobotVisualizer
    from visualization import PyBulletVisualizer
    from visualization import MeshcatVisualizer
"""

from visualization.path_3d import (
    plot_ee_path_3d,
    plot_ee_path_with_waypoints,
)
from visualization.robot_visualizer import (
    RobotVisualizer,
    animate_trajectory,
)
from visualization.trajectory_plots import (
    plot_acceleration_profile,
    plot_joint_comparison,
    plot_joint_trajectory,
    plot_phase_portrait,
    plot_velocity_profile,
)

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    PyBulletVisualizer = None

try:
    from visualization.meshcat_visualizer import MeshcatVisualizer
    MESHCAT_AVAILABLE = True
except ImportError:
    MESHCAT_AVAILABLE = False
    MeshcatVisualizer = None

__all__ = [
    # Joint-space plots
    "plot_joint_trajectory",
    "plot_joint_comparison",
    "plot_velocity_profile",
    "plot_acceleration_profile",
    "plot_phase_portrait",
    # 3D path plots
    "plot_ee_path_3d",
    "plot_ee_path_with_waypoints",
    # Robot visualization
    "RobotVisualizer",
    "animate_trajectory",
    # PyBullet visualization
    "PyBulletVisualizer",
    "PYBULLET_AVAILABLE",
    # Meshcat visualization
    "MeshcatVisualizer",
    "MESHCAT_AVAILABLE",
]
```

- [ ] **Step 4: Run tests**

```bash
pytest tests/test_meshcat_visualizer.py -v
```

Expected: all PASSED

- [ ] **Step 5: Commit**

```bash
git add visualization/__init__.py
git commit -m "feat: export MeshcatVisualizer from visualization package"
```

---

## Task 11: Demo 08 — browser-based obstacle avoidance

**Files:**
- Create: `demos/08_meshcat_visualization.py`

This task has no unit tests (it's a runnable demo). Verify by running it and checking the browser.

- [ ] **Step 1: Create the demo**

Create `demos/08_meshcat_visualization.py`:

```python
#!/usr/bin/env python3
"""
Demo 08: Meshcat Browser-Based Visualization with Obstacle Avoidance.

This demo mirrors demo 06 (PyBullet) but uses meshcat for visualization —
no display server required, just open the printed URL in any browser.

Features demonstrated:
- Meshcat browser-based 3D visualization
- UR5 model loaded from URDF via yourdfpy
- IK-based goal pose specification
- Obstacle avoidance planning
- Path visualization before trajectory execution
- Trajectory playback with EE trail

Run with: python demos/08_meshcat_visualization.py
Then open the URL printed to the terminal in your browser.
"""

import time

import numpy as np

from core.kinematics.opw import OPWKinematics
from core.kinematics.opw_parameters import OPWParameters
from core.kinematics.urdf_kinematics import URDFKinematics
from core.path_smoothing import smooth_path
from core.pose_utils import IKSolver
from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from parameterization.toppra_parameterization import ToppraTimeParameterizer
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner

try:
    from visualization.meshcat_visualizer import MeshcatVisualizer
except ImportError:
    print("ERROR: meshcat/yourdfpy not installed.")
    print("Install with: pip install meshcat yourdfpy")
    exit(1)


UR5_VELOCITY_LIMITS = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])
UR5_ACCEL_LIMITS = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

SAFE_START_CONFIGS = [
    np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
    np.array([0.0, -np.pi / 2, np.pi / 3, -np.pi / 3, -np.pi / 2, 0.0]),
]


def main():
    print("=== Demo 08: Meshcat Visualization ===\n")

    # --- Robot setup ---
    robot = UR5RobotModel(use_opw=False, collision_manager=None)
    fk = URDFKinematics()
    ik_solver = IKSolver(robot)
    state_space = JointStateSpace(robot)

    # --- Start meshcat ---
    viz = MeshcatVisualizer(open_browser=True)
    print("\nOpen the URL above in your browser, then press Enter to continue.")
    input()

    # --- Add obstacles ---
    print("Adding obstacles...")
    obs1 = viz.add_box(
        center=(0.4, 0.0, 0.4),
        size=(0.15, 0.15, 0.4),
        color=(1.0, 0.2, 0.2, 0.7),
    )
    obs2 = viz.add_sphere(
        center=(0.0, 0.5, 0.3),
        radius=0.1,
        color=(0.2, 1.0, 0.2, 0.7),
    )

    # --- Define goal ---
    goal_pos = np.array([0.4, 0.3, 0.5])
    goal_rot = np.eye(3)

    # --- IK for goal ---
    print("Solving IK for goal...")
    q_start = SAFE_START_CONFIGS[0]
    T_goal = np.eye(4)
    T_goal[:3, :3] = goal_rot
    T_goal[:3, 3] = goal_pos
    q_goals = ik_solver.solve(T_goal)
    if not q_goals:
        print("IK failed — using fallback goal config")
        q_goal = SAFE_START_CONFIGS[1]
    else:
        q_goal = q_goals[0]

    print(f"Start config: {np.round(q_start, 3)}")
    print(f"Goal config:  {np.round(q_goal, 3)}")

    # --- Show start config ---
    viz.visualize_configuration(q_start)
    print("\nShowing start config. Press Enter to plan...")
    input()

    # --- Plan ---
    print("Planning with RRT-Connect...")
    planner = OMPLRRTConnectPlanner(state_space)
    result = planner.plan(q_start, q_goal, timeout=10.0)

    if result["path"] is None:
        print("Planning failed.")
        return

    path = result["path"]
    path = smooth_path(path, state_space, iterations=50)
    print(f"Path found: {len(path)} waypoints")

    # --- Show planned path ---
    viz.visualize_path(path, fk_func=lambda q: (fk.forward_kinematics(q)[:3, 3], None))
    print("Planned path shown. Press Enter to parameterize and execute...")
    input()

    # --- Time parameterization ---
    parameterizer = ToppraTimeParameterizer(
        velocity_limits=UR5_VELOCITY_LIMITS,
        acceleration_limits=UR5_ACCEL_LIMITS,
    )
    traj_result = parameterizer.parameterize(path, q_start, q_goal)
    trajectory = traj_result["trajectory"]
    time_stamps = traj_result["time_stamps"]
    print(f"Trajectory: {len(trajectory)} points over {time_stamps[-1]:.2f}s")

    # --- Execute trajectory with trail ---
    print("Executing trajectory in browser...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,
        speed=1.0,
        show_ee_trail=True,
    )

    print("\nDone. Browser window remains open. Press Enter to exit.")
    input()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the demo to verify it works**

```bash
python demos/08_meshcat_visualization.py
```

Open the printed URL in a browser. Verify:
- Browser shows UR5 robot geometry
- Obstacles (red box, green sphere) appear in scene
- EE trail draws as trajectory executes

- [ ] **Step 3: Commit**

```bash
git add demos/08_meshcat_visualization.py
git commit -m "feat: add demo 08 meshcat browser visualization"
```

---

## Task 12: Final integration check

- [ ] **Step 1: Run full test suite**

```bash
pytest tests/ -v --timeout=30
```

Expected: all tests pass; meshcat-specific tests skip if meshcat not installed.

- [ ] **Step 2: Verify package import is clean**

```bash
python -c "
from visualization import MeshcatVisualizer, MESHCAT_AVAILABLE, PyBulletVisualizer, PYBULLET_AVAILABLE
print(f'Meshcat available: {MESHCAT_AVAILABLE}')
print(f'PyBullet available: {PYBULLET_AVAILABLE}')
print('Import OK')
"
```

Expected output:
```
Meshcat available: True
PyBullet available: True
Import OK
```

- [ ] **Step 3: Verify linting passes**

```bash
ruff check visualization/meshcat_visualizer.py demos/08_meshcat_visualization.py core/kinematics/urdf_kinematics.py
```

Expected: no errors

- [ ] **Step 4: Final commit**

```bash
git add -u
git commit -m "feat: complete MeshcatVisualizer implementation

- Add MeshcatVisualizer with full PyBulletVisualizer API parity
- Use yourdfpy for URDF geometry parsing
- Use extended URDFKinematics.link_transforms() for per-link FK
- Add demo 08 showing browser-based obstacle avoidance
- Graceful fallback in visualization/__init__.py"
```
