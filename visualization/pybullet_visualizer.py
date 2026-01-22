"""PyBullet-based 3D robot arm visualization.

This module provides realistic 3D visualization using PyBullet physics engine.
It can visualize robot trajectories with proper 3D rendering, lighting, and
interactive camera controls.

Requirements:
    pybullet>=3.2.0
"""

from typing import Optional, List, Tuple
import numpy as np
import tempfile
import os

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False


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
        if urdf_path and os.path.exists(urdf_path):
            self.robot_id = p.loadURDF(
                urdf_path,
                basePosition=base_position,
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=True,
            )
        else:
            self.robot_id = self._create_simple_ur5_model()

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

    def close(self):
        """Close PyBullet connection."""
        p.disconnect(self.client_id)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
