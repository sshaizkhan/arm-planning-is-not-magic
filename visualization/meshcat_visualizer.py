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
    r = int(min(1.0, max(0.0, rgba[0])) * 255)
    g_v = int(min(1.0, max(0.0, rgba[1])) * 255)
    b = int(min(1.0, max(0.0, rgba[2])) * 255)
    return (r << 16) | (g_v << 8) | b


def _cylinder_align() -> np.ndarray:
    """Return 4x4 rotation that maps meshcat's Y-aligned cylinder to Z-aligned (URDF convention)."""
    return tf.rotation_matrix(np.pi / 2, [1, 0, 0])


def get_default_ur5_urdf() -> Optional[str]:
    """Return path to the bundled UR5 URDF, or None if not found."""
    urdf = Path(__file__).parent.parent / "robots" / "ur5" / "ur5.urdf"
    return str(urdf) if urdf.exists() else None


_DEFAULT_LINK_COLORS = [
    [0.2, 0.2, 0.2, 1.0],
    [0.7, 0.7, 0.7, 1.0],
    [0.0, 0.4, 0.6, 1.0],
    [0.7, 0.7, 0.7, 1.0],
    [0.2, 0.2, 0.2, 1.0],
    [0.7, 0.7, 0.7, 1.0],
    [0.2, 0.2, 0.2, 1.0],
    [0.1, 0.1, 0.1, 1.0],
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
            raise ImportError("meshcat is not installed. Install with: pip install meshcat")
        if not YOURDFPY_AVAILABLE:
            raise ImportError("yourdfpy is not installed. Install with: pip install yourdfpy")

        self.base_position = np.array(base_position)
        self._path_object_paths: List[str] = []
        self._obstacle_count: Dict[str, int] = {"box": 0, "sphere": 0, "cylinder": 0, "marker": 0, "text": 0}
        self._last_ee_pos = np.zeros(3)
        self._last_ee_rot = np.eye(3)

        self._viz = meshcat.Visualizer(zmq_url) if zmq_url else meshcat.Visualizer()
        if open_browser:
            self._viz.open()
        print(f"Meshcat running at: {self._viz.url()}")

        if urdf_path is None:
            urdf_path = get_default_ur5_urdf()
        if urdf_path is None:
            raise FileNotFoundError("No URDF found. Pass urdf_path= or ensure robots/ur5/ur5.urdf exists.")
        self._robot = yourdfpy.URDF.load(urdf_path)
        self._load_robot_geometry()

        self._viz["ground/plane"].set_object(
            g.Box([2.0, 2.0, 0.005]),
            g.MeshLambertMaterial(color=0x888888),
        )
        self._viz["ground/plane"].set_transform(tf.translation_matrix([0, 0, -0.0025]))

        from core.kinematics.urdf_kinematics import URDFKinematics
        self._fk = URDFKinematics()

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

    def _urdf_geom_to_meshcat(self, geom) -> Optional[object]:
        """Convert yourdfpy geometry to meshcat geometry. Returns None for meshes."""
        if geom.cylinder is not None:
            return g.Cylinder(geom.cylinder.length, geom.cylinder.radius)
        if geom.box is not None:
            size = geom.box.size
            return g.Box([float(size[0]), float(size[1]), float(size[2])])
        if geom.sphere is not None:
            return g.Sphere(geom.sphere.radius)
        return None

    def _urdf_mat_to_meshcat(self, material, fallback_rgba: List[float]) -> object:
        """Convert yourdfpy material to MeshLambertMaterial."""
        if material is not None and material.color is not None:
            rgba = material.color.rgba
        else:
            rgba = fallback_rgba
        hex_color = _rgba_to_hex(rgba)
        alpha = float(rgba[3])
        return g.MeshLambertMaterial(color=hex_color, opacity=alpha, transparent=(alpha < 1.0))

    def set_joint_positions(self, q: np.ndarray):
        """Set robot joint positions and update all link transforms in meshcat."""
        if len(q) != 6:
            raise ValueError(f"Expected 6 joints, got {len(q)}")

        link_transforms = self._fk.link_transforms(q)

        T_ee = link_transforms.get('ee_link', np.eye(4))
        self._last_ee_pos = T_ee[:3, 3].copy()
        self._last_ee_rot = T_ee[:3, :3].copy()

        for link in self._robot.links:
            if not link.visuals:
                continue
            T_world = link_transforms.get(link.name)
            if T_world is None:
                continue
            for vis_idx, visual in enumerate(link.visuals):
                origin = np.array(visual.origin) if visual.origin is not None else np.eye(4)
                if visual.geometry.cylinder is not None:
                    origin = origin @ _cylinder_align()
                T_final = T_world @ origin
                T_final = T_final.copy()
                T_final[:3, 3] += self.base_position
                self._viz[f"robot/{link.name}/visual_{vis_idx}"].set_transform(T_final)

    def visualize_configuration(self, q: np.ndarray, duration: float = 0.0):
        """Visualize a single robot configuration."""
        self.set_joint_positions(q)
        if duration > 0:
            time.sleep(duration)

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (position (3,), rotation_matrix (3,3)) of EE from last set_joint_positions."""
        return self._last_ee_pos.copy(), self._last_ee_rot.copy()

    def visualize_trajectory(
        self,
        trajectory: np.ndarray,
        time_stamps: Optional[np.ndarray] = None,
        real_time: bool = True,
        speed: float = 1.0,
        show_ee_trail: bool = True,
        trail_length: int = 100,
    ):
        """Visualize a complete trajectory."""
        n = len(trajectory)
        if trajectory.ndim != 2 or trajectory.shape[1] != 6:
            raise ValueError(f"trajectory must be shape (N, 6), got {trajectory.shape}")
        if time_stamps is None:
            time_stamps = np.linspace(0, 1, n)

        ee_trail: List[np.ndarray] = []
        last_frame = time.time()

        for i, q in enumerate(trajectory):
            self.set_joint_positions(q)

            if show_ee_trail:
                ee_trail.append(self._last_ee_pos.copy())
                while len(ee_trail) > trail_length:
                    ee_trail.pop(0)
                if len(ee_trail) >= 2:
                    pts = np.array(ee_trail).T
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
                if pos.shape != (3,):
                    raise ValueError(
                        f"fk_func must return a (3,) position vector (or tuple with one). "
                        f"Got shape {pos.shape}. If returning a 4x4 matrix, pass it directly "
                        f"(not wrapped in a tuple)."
                    )
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
        lifetime: float = 0,  # not supported by meshcat; reserved for API compatibility
    ) -> List[str]:
        """Visualize a planned path as a 3D EE trajectory line."""
        self.clear_path()
        if len(path) < 2:
            return []

        ee_positions = self._compute_ee_positions(path, fk_func)

        hex_line = _rgba_to_hex([*color, 1.0])
        pts = np.array(ee_positions).T.astype(np.float32)
        line_mat = g.LineBasicMaterial(color=hex_line, linewidth=int(line_width))
        self._viz["path/line"].set_object(g.Line(g.PointsGeometry(pts), line_mat))
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
        lifetime: float = 0,  # not supported by meshcat; reserved for API compatibility
    ) -> List[str]:
        """Visualize a dense trajectory as a smooth 3D curve (no waypoint markers)."""
        sampled = trajectory[::sample_every]
        if not np.array_equal(sampled[-1], trajectory[-1]):
            sampled = np.vstack([sampled, trajectory[-1]])
        return self.visualize_path(
            sampled, fk_func=fk_func, color=color, line_width=line_width, show_waypoints=False,
        )

    def clear_path(self):
        """Delete all path visualization objects from the scene."""
        self._viz["path"].delete()
        self._path_object_paths = []

    def add_box(
        self,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.7),
    ) -> str:
        """Add a box obstacle. Returns meshcat path string."""
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
        """Add a sphere obstacle. Returns meshcat path string."""
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
        """Add a vertical cylinder obstacle. Returns meshcat path string."""
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
        lifetime: float = 0,  # not supported by meshcat; reserved for API compatibility
    ) -> str:
        """Add a small sphere marker. Returns meshcat path string."""
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
        lifetime: float = 0,  # not supported by meshcat; reserved for API compatibility
    ) -> str:
        """Add a label marker at a 3D position (prints text to stdout; no native text in meshcat)."""
        n = self._obstacle_count["text"]
        self._obstacle_count["text"] += 1
        path = f"labels/text_{n}"
        hex_color = _rgba_to_hex(list(color) + [1.0])
        sphere_radius = max(0.005, size * 0.008)
        self._viz[path].set_object(g.Sphere(sphere_radius), g.MeshLambertMaterial(color=hex_color))
        self._viz[path].set_transform(tf.translation_matrix(list(position)))
        print(f"[label] {text} @ {tuple(round(float(v), 3) for v in position)}")
        return path

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
        """Draw all planner trajectories simultaneously with different colors."""
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
        """Stop the meshcat server (no-op; GC handles cleanup)."""
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
