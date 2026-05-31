# MeshcatVisualizer Design Spec

**Date:** 2026-05-30  
**Status:** Approved

## Goal

Add `MeshcatVisualizer` as a drop-in alternative to `PyBulletVisualizer`. Renders robot arm in a browser tab via `meshcat`. No display server required.

## Dependencies

```
pip install meshcat yourdfpy
```

Added to `pyproject.toml` as optional group `[meshcat]`:
- `meshcat>=0.3.2`
- `yourdfpy>=0.0.53`

## Files Changed

| File | Change |
|------|--------|
| `visualization/meshcat_visualizer.py` | New — `MeshcatVisualizer` class |
| `visualization/__init__.py` | Add `MeshcatVisualizer`, `MESHCAT_AVAILABLE` with graceful fallback |
| `pyproject.toml` | Add `[meshcat]` optional dependency group |
| `demos/08_meshcat_visualization.py` | New — demo mirroring `demos/06_pybullet_visualization.py` |

## Architecture

### MeshcatVisualizer class

```
MeshcatVisualizer
├── __init__(urdf_path=None, open_browser=True, zmq_url=None)
├── set_joint_positions(q: np.ndarray)
├── visualize_configuration(q, duration=0.0)
├── visualize_trajectory(trajectory, time_stamps, real_time, speed, show_ee_trail, trail_length)
├── visualize_path(path, fk_func, color, line_width, show_waypoints, waypoint_color, waypoint_size, lifetime)
├── visualize_trajectory_path(trajectory, fk_func, color, line_width, sample_every, lifetime)
├── visualize_multi_planner_trajectories(planner_results, fk_func, show_labels, line_width, sample_every)
├── add_marker(position, color, size, lifetime)
├── add_box(center, size, color)
├── add_sphere(center, radius, color)
├── add_cylinder(center, radius, height, color)
├── clear_path()
├── close()
├── __enter__ / __exit__
└── [property] url → str
```

All public method signatures are identical to `PyBulletVisualizer`. The only omission is `create_collision_checker` (PyBullet physics feature with no meshcat equivalent).

### Initialization

1. Start `meshcat.Visualizer(zmq_url)` — binds a ZMQ server and HTTP server
2. Optionally call `viz.open()` to open browser tab automatically
3. Load URDF with `yourdfpy.URDF.load(urdf_path)` — falls back to bundled `robots/ur5/ur5.urdf`
4. Call `_load_robot_geometry()` — populate meshcat scene tree
5. Add ground plane as a thin flat `Box`
6. Call `set_joint_positions(np.zeros(6))` to set initial pose

### Scene tree layout

```
meshcat root
├── robot/
│   ├── base_link          ← Box/Cylinder geometry + material from URDF
│   ├── shoulder_link
│   ├── upper_arm_link
│   ├── forearm_link
│   ├── wrist_1_link
│   ├── wrist_2_link
│   └── wrist_3_link
├── ground/
│   └── plane
├── path/
│   ├── line               ← EE path line (planned waypoints)
│   └── waypoints/N        ← sphere per waypoint
├── trail/
│   └── line               ← live EE trail during trajectory playback
└── obstacles/
    ├── box_0
    ├── sphere_0
    └── cylinder_0
```

### FK and transform updates (`set_joint_positions`)

`yourdfpy.URDF` provides `get_transform(link_name, cfg)` where `cfg` is a dict of `{joint_name: angle}`. This returns a 4×4 numpy homogeneous transform (world frame).

```python
joint_names = [j.name for j in self._robot.actuated_joints]
cfg = dict(zip(joint_names, q))
for link in self._robot.links:
    T = self._robot.get_transform(link.name, cfg)
    self._viz[f"robot/{link.name}"].set_transform(T)
```

Each link also has a visual origin offset in the URDF (`visual.origin`). This is composed into the transform before setting.

### Geometry mapping

| URDF primitive | meshcat geometry | Notes |
|----------------|-----------------|-------|
| `<cylinder radius="r" length="l"/>` | `Cylinder(l, r)` | Meshcat Cylinder is Y-aligned; apply 90° X rotation |
| `<box size="x y z"/>` | `Box([x, y, z])` | Direct mapping |
| `<sphere radius="r"/>` | `Sphere(r)` | Direct mapping |
| `<mesh filename="..."/>` | `ObjMeshGeometry` or skip | Skip for now; UR5 URDF uses only primitives |

### Color mapping

URDF material `rgba` → `meshcat.geometry.MeshLambertMaterial`:
```python
r, g, b, a = visual.material.color.rgba
hex_color = int(r * 255) << 16 | int(g * 255) << 8 | int(b * 255)
material = MeshLambertMaterial(color=hex_color, opacity=a, transparent=(a < 1.0))
```

### EE trail (during `visualize_trajectory`)

Accumulate EE world positions each frame. Compute EE position using `fk_func` if provided, else from `yourdfpy.get_transform("wrist_3_link", cfg)[:3, 3]`. Update trail as:

```python
points = np.array(ee_trail).T  # shape (3, N)
self._viz["trail/line"].set_object(
    Line(PointsGeometry(points), LineBasicMaterial(color=0x00ff00))
)
```

### Planned path visualization (`visualize_path`)

Compute EE positions for all waypoints → draw as `Line`. If `show_waypoints=True`, add `Sphere(waypoint_size)` at each waypoint position under `path/waypoints/N`.

`clear_path()` deletes `path/` subtree: `self._viz["path"].delete()`.

### Multi-planner trajectories

Same as `PyBulletVisualizer`: iterate planners, assign colors from palette, draw each as a `Line` under `path/planner_name/line`. Add text label using `meshcat`'s `Text` geometry if `show_labels=True`.

### Obstacles

Each `add_box/sphere/cylinder` call:
1. Increments an internal counter per type
2. Creates geometry + `MeshLambertMaterial` with opacity from alpha channel
3. Sets transform to center position
4. Returns the meshcat path string (analogous to PyBullet body ID)

`remove_body(path_str)` calls `self._viz[path_str].delete()`.

## API Parity Summary

| Method | PyBullet | Meshcat | Notes |
|--------|----------|---------|-------|
| `set_joint_positions` | ✓ | ✓ | |
| `visualize_configuration` | ✓ | ✓ | |
| `visualize_trajectory` | ✓ | ✓ | |
| `visualize_path` | ✓ | ✓ | |
| `visualize_trajectory_path` | ✓ | ✓ | |
| `visualize_multi_planner_trajectories` | ✓ | ✓ | |
| `add_marker` | ✓ | ✓ | |
| `add_box` | ✓ | ✓ | returns path str instead of int ID |
| `add_sphere` | ✓ | ✓ | returns path str instead of int ID |
| `add_cylinder` | ✓ | ✓ | returns path str instead of int ID |
| `remove_body` | ✓ | ✓ | accepts path str |
| `add_text` | ✓ | ✓ | meshcat Text geometry |
| `clear_path` | ✓ | ✓ | |
| `close` | ✓ | ✓ | |
| `create_collision_checker` | ✓ | ✗ | PyBullet physics only — not added |
| `reset_camera` | ✓ | ✗ | meshcat camera controlled in browser |
| `get_end_effector_pose` | ✓ | ✓ | via yourdfpy FK |

## Demo (demos/08_meshcat_visualization.py)

Mirrors `demos/06_pybullet_visualization.py`:
- Same OMPL planner, same IK goal specification
- Adds box obstacles to scene
- Shows planned path before trajectory execution
- Plays trajectory with EE trail
- Prints browser URL so user can open it

## Error Handling

- `meshcat` not installed → `ImportError` with install instructions
- `yourdfpy` not installed → `ImportError` with install instructions
- URDF not found → fall back to bundled `robots/ur5/ur5.urdf`; if still missing, raise `FileNotFoundError`
- `visualization/__init__.py` wraps import in try/except → sets `MESHCAT_AVAILABLE = False`, `MeshcatVisualizer = None`
