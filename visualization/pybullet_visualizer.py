"""PyBullet-based 3D robot arm visualization.

This module provides realistic 3D visualization using PyBullet physics engine.
It can visualize robot trajectories with proper 3D rendering, lighting, and
interactive camera controls.

Requirements:
    pybullet>=3.2.0
"""

from typing import Optional, List, Tuple, Callable
import numpy as np
import tempfile
import os
from pathlib import Path

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False


def get_default_ur5_urdf() -> str:
    """Get path to the default UR5 URDF file included with this package."""
    module_dir = Path(__file__).parent.parent
    urdf_path = module_dir / "robots" / "ur5" / "ur5.urdf"
    if urdf_path.exists():
        return str(urdf_path)
    return None


class PyBulletVisualizer:
    """
    3D visualizer for robot arm trajectories using PyBullet.

    This visualizer creates a realistic 3D environment with proper physics
    rendering. It can load URDF files or create simple robot models programmatically.
    """

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        gui: bool = True,
        base_position: Tuple[float, float, float] = (0, 0, 0),
        camera_distance: float = 1.5,
        camera_yaw: float = 45,
        camera_pitch: float = -30,
    ):
        """
        Initialize PyBullet visualizer.

        Args:
            urdf_path: Path to URDF file. If None, creates a simple UR5-like model
            gui: If True, show GUI window. If False, run headless
            base_position: (x, y, z) position of robot base
            camera_distance: Initial camera distance from robot
            camera_yaw: Initial camera yaw angle (degrees)
            camera_pitch: Initial camera pitch angle (degrees)
        """
        if not PYBULLET_AVAILABLE:
            raise ImportError(
                "PyBullet is not installed. Install with: pip install pybullet"
            )

        self.base_position = np.array(base_position)
        self.gui = gui
        self.robot_id = None
        self.urdf_path = urdf_path

        # Initialize PyBullet
        if gui:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        # Set camera
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=base_position,
        )

        # Load robot
        # Priority: 1) explicit urdf_path, 2) default UR5 URDF, 3) simple generated model
        if urdf_path and os.path.exists(urdf_path):
            actual_urdf_path = urdf_path
        else:
            actual_urdf_path = get_default_ur5_urdf()

        if actual_urdf_path and os.path.exists(actual_urdf_path):
            self.robot_id = p.loadURDF(
                actual_urdf_path,
                basePosition=base_position,
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=True,
            )
        else:
            self.robot_id = self._create_simple_ur5_model()

        # Track debug line IDs for cleanup
        self._path_line_ids = []

        # Get joint info
        self.joint_indices = []
        self.joint_names = []
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_indices.append(i)
                self.joint_names.append(joint_info[1].decode('utf-8'))

        if len(self.joint_indices) != 6:
            raise ValueError(
                f"Expected 6 revolute joints, found {len(self.joint_indices)}"
            )

        # Find end-effector link (last link in the chain)
        self.ee_link_index = num_joints - 1

        # Create ground plane
        p.loadURDF("plane.urdf")

    def _create_simple_ur5_model(self) -> int:
        """
        Create a simple UR5-like robot model programmatically.

        Returns:
            PyBullet body ID
        """
        # UR5 link lengths (from OPW parameters)
        # c1=0.089159, c2=0.425, c3=0.39225, c4=0.09465
        link_lengths = [0.089159, 0.425, 0.39225, 0.109, 0.095, 0.09465]

        # Create URDF content
        urdf_content = self._generate_ur5_urdf(link_lengths)

        # Write to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(urdf_content)
            temp_urdf_path = f.name

        try:
            robot_id = p.loadURDF(
                temp_urdf_path,
                basePosition=self.base_position,
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=True,
            )
        finally:
            os.unlink(temp_urdf_path)

        return robot_id

    def _generate_ur5_urdf(self, link_lengths: List[float]) -> str:
        """Generate URDF XML for UR5-like robot."""
        d1, a2, a3, d4, d5, d6 = link_lengths

        urdf = f"""<?xml version="1.0"?>
<robot name="ur5_simple">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="base">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 {d1/2}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="150" velocity="3.15"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="{d1}"/>
      </geometry>
      <material name="link1">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="{d1}"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 {d1/2}" rpy="0 1.5708 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="150" velocity="3.15"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <box size="0.06 0.06 {a2}"/>
      </geometry>
      <material name="link2">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.06 0.06 {a2}"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 {a2}" rpy="0 -1.5708 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-3.14" upper="3.14" effort="150" velocity="3.15"/>
  </joint>

  <link name="link3">
    <visual>
      <geometry>
        <box size="0.06 0.06 {a3}"/>
      </geometry>
      <material name="link3">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.06 0.06 {a3}"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 {a3}" rpy="1.5708 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit lower="-6.28" upper="6.28" effort="28" velocity="6.28"/>
  </joint>

  <link name="link4">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="{d4}"/>
      </geometry>
      <material name="link4">
        <color rgba="0.8 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="{d4}"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 {d4}" rpy="0 -1.5708 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-3.14" upper="3.14" effort="28" velocity="6.28"/>
  </joint>

  <link name="link5">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="{d5}"/>
      </geometry>
      <material name="link5">
        <color rgba="0.8 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="{d5}"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 {d5}" rpy="1.5708 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit lower="-6.28" upper="6.28" effort="28" velocity="6.28"/>
  </joint>

  <link name="link6">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="{d6}"/>
      </geometry>
      <material name="link6">
        <color rgba="0.2 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="{d6}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
        return urdf

    def set_joint_positions(self, q: np.ndarray):
        """
        Set robot joint positions.

        Args:
            q: Joint angles, shape (6,)
        """
        if len(q) != len(self.joint_indices):
            raise ValueError(
                f"Expected {len(self.joint_indices)} joints, got {len(q)}"
            )

        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, q[i])

    def visualize_configuration(self, q: np.ndarray, duration: float = 0.0):
        """
        Visualize a single robot configuration.

        Args:
            q: Joint angles, shape (6,)
            duration: Time to hold this configuration (seconds)
        """
        self.set_joint_positions(q)
        if duration > 0:
            import time
            time.sleep(duration)

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
            time_stamps: Optional time stamps, shape (N,). If None, uses uniform timing
            real_time: If True, play at real-time speed. If False, as fast as possible
            speed: Speed multiplier (1.0 = normal speed, 2.0 = 2x speed, etc.)
            show_ee_trail: If True, show end-effector trail
            trail_length: Maximum number of points in trail
        """
        n_points = len(trajectory)

        if time_stamps is None:
            time_stamps = np.linspace(0, 1, n_points)

        ee_trail = []
        import time

        start_time = time.time()
        last_frame_time = start_time

        for i, q in enumerate(trajectory):
            self.set_joint_positions(q)

            if show_ee_trail:
                # Get end-effector position
                ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
                ee_pos = ee_state[0]
                ee_trail.append(ee_pos)

                if len(ee_trail) > trail_length:
                    ee_trail.pop(0)

                # Draw trail
                if len(ee_trail) > 1:
                    for j in range(len(ee_trail) - 1):
                        p.addUserDebugLine(
                            ee_trail[j],
                            ee_trail[j + 1],
                            lineColorRGB=[0, 1, 0],
                            lineWidth=2,
                            lifeTime=0,
                        )

            if real_time and i < n_points - 1:
                dt = (time_stamps[i + 1] - time_stamps[i]) / speed
                elapsed = time.time() - last_frame_time
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                last_frame_time = time.time()
            else:
                p.stepSimulation()

            if self.gui:
                # Check for exit
                keys = p.getKeyboardEvents()
                if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                    break

    def add_marker(
        self,
        position: Tuple[float, float, float],
        color: Tuple[float, float, float] = (1, 0, 0),
        size: float = 0.05,
        lifetime: float = 0,
    ):
        """
        Add a visual marker in the scene.

        Args:
            position: (x, y, z) position
            color: RGB color tuple (0-1)
            size: Marker size
            lifetime: How long marker persists (0 = forever)
        """
        p.addUserDebugLine(
            [position[0] - size, position[1], position[2]],
            [position[0] + size, position[1], position[2]],
            lineColorRGB=color,
            lineWidth=3,
            lifeTime=lifetime,
        )
        p.addUserDebugLine(
            [position[0], position[1] - size, position[2]],
            [position[0], position[1] + size, position[2]],
            lineColorRGB=color,
            lineWidth=3,
            lifeTime=lifetime,
        )
        p.addUserDebugLine(
            [position[0], position[1], position[2] - size],
            [position[0], position[1], position[2] + size],
            lineColorRGB=color,
            lineWidth=3,
            lifeTime=lifetime,
        )

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
    ):
        """
        Visualize a planned path as a 3D line showing the end-effector trajectory.

        This displays the path waypoints BEFORE trajectory execution, useful for
        previewing where the end-effector will travel.

        Args:
            path: Joint space path, shape (N, 6) - the waypoints from planner
            fk_func: Forward kinematics function. If None, uses PyBullet's FK.
                     Should take joint angles and return (position, rotation_matrix)
            color: RGB color for the path line (0-1)
            line_width: Width of the path line
            show_waypoints: If True, show markers at each waypoint
            waypoint_color: RGB color for waypoint markers
            waypoint_size: Size of waypoint markers
            lifetime: How long path persists (0 = forever)

        Returns:
            List of debug line IDs (for manual cleanup if needed)
        """
        # Clear previous path visualization
        self.clear_path()

        if len(path) < 2:
            return []

        ee_positions = []

        # Compute end-effector positions for each waypoint
        for q in path:
            if fk_func is not None:
                # Use provided FK function
                pose = fk_func(q)
                if isinstance(pose, tuple):
                    pos = pose[0]  # position is first element
                else:
                    pos = pose[:3, 3]  # Extract from 4x4 matrix
                ee_positions.append(pos)
            else:
                # Use PyBullet's FK by temporarily setting joint positions
                original_state = []
                for idx in self.joint_indices:
                    state = p.getJointState(self.robot_id, idx)
                    original_state.append(state[0])

                self.set_joint_positions(q)
                ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
                ee_positions.append(ee_state[0])

                # Restore original state
                for i, idx in enumerate(self.joint_indices):
                    p.resetJointState(self.robot_id, idx, original_state[i])

        # Draw path line
        line_ids = []
        for i in range(len(ee_positions) - 1):
            line_id = p.addUserDebugLine(
                ee_positions[i],
                ee_positions[i + 1],
                lineColorRGB=color,
                lineWidth=line_width,
                lifeTime=lifetime,
            )
            line_ids.append(line_id)

        # Draw waypoint markers
        if show_waypoints:
            for pos in ee_positions:
                # Draw small cross at each waypoint
                for axis in range(3):
                    offset = [0, 0, 0]
                    offset[axis] = waypoint_size
                    start = [pos[j] - offset[j] for j in range(3)]
                    end = [pos[j] + offset[j] for j in range(3)]
                    line_id = p.addUserDebugLine(
                        start,
                        end,
                        lineColorRGB=waypoint_color,
                        lineWidth=2,
                        lifeTime=lifetime,
                    )
                    line_ids.append(line_id)

        self._path_line_ids = line_ids
        return line_ids

    def visualize_trajectory_path(
        self,
        trajectory: np.ndarray,
        fk_func: Optional[Callable] = None,
        color: Tuple[float, float, float] = (0.0, 0.7, 1.0),
        line_width: float = 2,
        sample_every: int = 5,
        lifetime: float = 0,
    ):
        """
        Visualize a dense trajectory as a smooth 3D curve (no waypoint markers).

        This is useful for showing the smooth interpolated trajectory after
        time-parameterization.

        Args:
            trajectory: Joint trajectory, shape (N, 6)
            fk_func: Forward kinematics function. If None, uses PyBullet's FK
            color: RGB color for the trajectory line
            line_width: Width of the line
            sample_every: Sample every N points (for performance with dense trajectories)
            lifetime: How long path persists (0 = forever)

        Returns:
            List of debug line IDs
        """
        # Sample the trajectory for performance
        sampled = trajectory[::sample_every]
        if len(sampled) < len(trajectory) and not np.array_equal(sampled[-1], trajectory[-1]):
            sampled = np.vstack([sampled, trajectory[-1]])

        return self.visualize_path(
            sampled,
            fk_func=fk_func,
            color=color,
            line_width=line_width,
            show_waypoints=False,
            lifetime=lifetime,
        )

    def clear_path(self):
        """Clear all path visualization lines."""
        for line_id in self._path_line_ids:
            p.removeUserDebugItem(line_id)
        self._path_line_ids = []

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current end-effector position and orientation.

        Returns:
            (position, orientation) tuple
            position: (x, y, z)
            orientation: quaternion (x, y, z, w)
        """
        ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
        return np.array(ee_state[0]), np.array(ee_state[1])

    def reset_camera(
        self,
        distance: float = 1.5,
        yaw: float = 45,
        pitch: float = -30,
        target: Optional[Tuple[float, float, float]] = None,
    ):
        """
        Reset camera view.

        Args:
            distance: Camera distance
            yaw: Camera yaw angle (degrees)
            pitch: Camera pitch angle (degrees)
            target: Camera target position. If None, uses base position
        """
        if target is None:
            target = self.base_position
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target,
        )

    def add_box(
        self,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.7),
    ) -> int:
        """
        Add a box obstacle to the scene.

        Args:
            center: (x, y, z) center position
            size: (width, depth, height) dimensions
            color: RGBA color (0-1), default semi-transparent red

        Returns:
            PyBullet body ID for the obstacle
        """
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[s / 2 for s in size],
            rgbaColor=color,
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[s / 2 for s in size],
        )
        body_id = p.createMultiBody(
            baseMass=0,  # Static object
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=center,
        )
        return body_id

    def add_sphere(
        self,
        center: Tuple[float, float, float],
        radius: float,
        color: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.7),
    ) -> int:
        """
        Add a sphere obstacle to the scene.

        Args:
            center: (x, y, z) center position
            radius: Sphere radius
            color: RGBA color (0-1), default semi-transparent green

        Returns:
            PyBullet body ID for the obstacle
        """
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color,
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=radius,
        )
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=center,
        )
        return body_id

    def add_cylinder(
        self,
        center: Tuple[float, float, float],
        radius: float,
        height: float,
        color: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.7),
    ) -> int:
        """
        Add a cylinder obstacle to the scene (vertical, along Z axis).

        Args:
            center: (x, y, z) center position
            radius: Cylinder radius
            height: Cylinder height
            color: RGBA color (0-1), default semi-transparent blue

        Returns:
            PyBullet body ID for the obstacle
        """
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=color,
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height,
        )
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=center,
        )
        return body_id

    def remove_body(self, body_id: int):
        """Remove a body (obstacle) from the scene."""
        p.removeBody(body_id)

    def create_collision_checker(self, obstacle_ids: List[int]):
        """
        Create a collision checking function that uses PyBullet's FK and collision detection.

        This ensures collision checking is consistent with the visualization.

        Args:
            obstacle_ids: List of PyBullet body IDs for obstacles to check against

        Returns:
            A function that takes joint angles and returns True if in collision
        """
        robot_id = self.robot_id
        joint_indices = self.joint_indices

        def check_collision(q: np.ndarray) -> bool:
            """Check if joint configuration q collides with any obstacle."""
            # Set joint positions
            for i, idx in enumerate(joint_indices):
                p.resetJointState(robot_id, idx, q[i])

            # Step simulation to update collision state
            p.performCollisionDetection()

            # Check for collisions between robot and each obstacle
            for obs_id in obstacle_ids:
                contacts = p.getContactPoints(bodyA=robot_id, bodyB=obs_id)
                if contacts:
                    return True
            return False

        return check_collision

    def add_text(
        self,
        text: str,
        position: Tuple[float, float, float],
        color: Tuple[float, float, float] = (1, 1, 1),
        size: float = 1.5,
        lifetime: float = 0,
    ) -> int:
        """
        Add 3D text label in the scene.

        Args:
            text: Text to display
            position: (x, y, z) position in world coordinates
            color: RGB color (0-1)
            size: Text size
            lifetime: How long text persists (0 = forever)

        Returns:
            Debug text ID
        """
        return p.addUserDebugText(
            text,
            position,
            textColorRGB=color,
            textSize=size,
            lifeTime=lifetime,
        )

    def visualize_multi_planner_trajectories(
        self,
        planner_results: dict,
        fk_func: Optional[Callable] = None,
        show_labels: bool = True,
        line_width: float = 3,
        sample_every: int = 3,
    ) -> dict:
        """
        Visualize trajectories from multiple planners with different colors.

        This draws all planner trajectories simultaneously in the scene,
        allowing visual comparison of different planning approaches.

        Args:
            planner_results: Dict mapping planner names to result dicts with keys:
                - 'trajectory': Dense joint trajectory (N, 6)
                - 'path': (optional) Original path waypoints
            fk_func: Forward kinematics function. If None, uses PyBullet's FK
            show_labels: If True, add text labels for each trajectory
            line_width: Width of trajectory lines
            sample_every: Sample every N points (for performance)

        Returns:
            Dict mapping planner names to their line IDs (for cleanup)

        Example:
            results = {
                'RRT': {'trajectory': traj1},
                'RRT*': {'trajectory': traj2},
            }
            viz.visualize_multi_planner_trajectories(results, robot.fk)
        """
        # Colorblind-friendly palette
        colors = [
            (0.12, 0.47, 0.71),  # blue
            (1.00, 0.50, 0.05),  # orange
            (0.17, 0.63, 0.17),  # green
            (0.84, 0.15, 0.16),  # red
            (0.58, 0.40, 0.74),  # purple
            (0.55, 0.34, 0.29),  # brown
            (0.89, 0.47, 0.76),  # pink
        ]

        all_line_ids = {}
        label_offset = 0.05

        for idx, (planner_name, result) in enumerate(planner_results.items()):
            if result.get('trajectory') is None:
                continue

            color = colors[idx % len(colors)]
            trajectory = result['trajectory']

            # Sample trajectory for performance
            sampled = trajectory[::sample_every]
            if len(sampled) < len(trajectory) and not np.array_equal(sampled[-1], trajectory[-1]):
                sampled = np.vstack([sampled, trajectory[-1]])

            # Compute EE positions
            ee_positions = []
            for q in sampled:
                if fk_func is not None:
                    pose = fk_func(q)
                    if isinstance(pose, tuple):
                        pos = pose[0]
                    else:
                        pos = pose[:3, 3]
                    ee_positions.append(pos)
                else:
                    # Save and restore joint state
                    original_state = [p.getJointState(self.robot_id, idx)[0]
                                      for idx in self.joint_indices]
                    self.set_joint_positions(q)
                    ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
                    ee_positions.append(ee_state[0])
                    for i, joint_idx in enumerate(self.joint_indices):
                        p.resetJointState(self.robot_id, joint_idx, original_state[i])

            # Draw trajectory line
            line_ids = []
            for i in range(len(ee_positions) - 1):
                line_id = p.addUserDebugLine(
                    ee_positions[i],
                    ee_positions[i + 1],
                    lineColorRGB=color,
                    lineWidth=line_width,
                    lifeTime=0,
                )
                line_ids.append(line_id)

            # Add label at midpoint of trajectory
            if show_labels and ee_positions:
                mid_idx = len(ee_positions) // 2
                label_pos = [
                    ee_positions[mid_idx][0],
                    ee_positions[mid_idx][1],
                    ee_positions[mid_idx][2] + label_offset * (idx + 1),
                ]
                text_id = self.add_text(planner_name, label_pos, color=color, size=1.2)
                line_ids.append(text_id)

            all_line_ids[planner_name] = line_ids
            self._path_line_ids.extend(line_ids)

        return all_line_ids

    def animate_planner_comparison(
        self,
        planner_results: dict,
        timestamps_key: str = 'timestamps',
        trajectory_key: str = 'trajectory',
        speed: float = 1.0,
        pause_between: float = 1.0,
        show_name: bool = True,
    ):
        """
        Animate through each planner's trajectory sequentially.

        Args:
            planner_results: Dict mapping planner names to result dicts
            timestamps_key: Key for timestamps in result dict
            trajectory_key: Key for trajectory in result dict
            speed: Playback speed multiplier
            pause_between: Pause duration between trajectories (seconds)
            show_name: If True, display planner name during animation
        """
        import time

        colors = [
            (0.12, 0.47, 0.71), (1.00, 0.50, 0.05), (0.17, 0.63, 0.17),
            (0.84, 0.15, 0.16), (0.58, 0.40, 0.74), (0.55, 0.34, 0.29),
        ]

        for idx, (planner_name, result) in enumerate(planner_results.items()):
            trajectory = result.get(trajectory_key)
            timestamps = result.get(timestamps_key)

            if trajectory is None:
                continue

            color = colors[idx % len(colors)]

            # Show planner name
            if show_name:
                text_id = p.addUserDebugText(
                    f"Playing: {planner_name}",
                    [0.5, 0, 0.8],
                    textColorRGB=color,
                    textSize=2.0,
                    lifeTime=0,
                )

            # Animate trajectory
            self.visualize_trajectory(
                trajectory,
                timestamps,
                real_time=True,
                speed=speed,
                show_ee_trail=True,
                trail_length=50,
            )

            # Remove text and pause
            if show_name:
                p.removeUserDebugItem(text_id)

            if pause_between > 0:
                time.sleep(pause_between)

    def close(self):
        """Close PyBullet connection."""
        p.disconnect(self.client_id)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
