"""
URDF-based forward kinematics for the UR5.

This implements FK by walking the exact joint chain from robots/ur5/ur5.urdf,
matching PyBullet's FK exactly. OPW is used for IK; this is used for FK.

Key insight: OPW is an approximate analytical IK solver. It works well for
finding joint angles but has an inherent approximation error for FK on robots
(like UR5) that don't have a true spherical wrist. This URDF-based FK has
zero approximation error relative to the URDF geometry.
"""

import numpy as np


def _rpy_to_matrix(roll, pitch, yaw):
    """Convert RPY angles to rotation matrix (Rz @ Ry @ Rx convention)."""
    cx, cy, cz = np.cos(roll), np.cos(pitch), np.cos(yaw)
    sx, sy, sz = np.sin(roll), np.sin(pitch), np.sin(yaw)
    return np.array([
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,             cy*cx],
    ])


def _axis_rotation(axis, angle):
    """Rotation matrix for angle around unit axis."""
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c],
    ])


def _transform(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


class URDFKinematics:
    """
    Forward kinematics for UR5 derived directly from the URDF joint chain.

    Matches robots/ur5/ur5.urdf exactly. Zero approximation error.
    """

    # Joint definitions: (xyz, rpy, axis) — from robots/ur5/ur5.urdf
    _JOINTS = [
        ([0, 0, 0.089159],    [0, 0, 0],               [0, 0, 1]),  # shoulder_pan
        ([0, 0.13585, 0],     [0, np.pi/2, 0],          [0, 1, 0]),  # shoulder_lift
        ([0, -0.1197, 0.425], [0, 0, 0],                [0, 1, 0]),  # elbow
        ([0, 0, 0.39225],     [0, np.pi/2, 0],          [0, 1, 0]),  # wrist_1
        ([0, 0.09465, 0],     [0, 0, 0],                [0, 0, 1]),  # wrist_2
        ([0, 0, 0.09465],     [0, 0, 0],                [0, 1, 0]),  # wrist_3
    ]
    # ee_fixed_joint (fixed, no joint angle)
    _EE_XYZ = [0, 0.0823, 0]
    _EE_RPY = [0, 0, np.pi/2]

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """
        Compute EE pose from joint angles.

        Args:
            q: Joint angles, shape (6,), in radians

        Returns:
            4x4 transformation matrix (EE pose in world frame)
        """
        T = np.eye(4)
        for i, (xyz, rpy, axis) in enumerate(self._JOINTS):
            R_frame = _rpy_to_matrix(*rpy)
            R_joint = _axis_rotation(axis, float(q[i]))
            T = T @ _transform(xyz, R_frame @ R_joint)
        # Fixed EE joint
        R_ee = _rpy_to_matrix(*self._EE_RPY)
        T = T @ _transform(self._EE_XYZ, R_ee)
        return T

    def link_positions(self, q: np.ndarray) -> np.ndarray:
        """
        Return world-space positions of 5 key link frames.

        Returns shape (5, 3): base, shoulder, elbow, wrist_1, end-effector.
        Used by collision managers to check the full robot body.
        """
        positions = [np.zeros(3)]  # base at origin
        T = np.eye(4)
        for i, (xyz, rpy, axis) in enumerate(self._JOINTS):
            R_frame = _rpy_to_matrix(*rpy)
            R_joint = _axis_rotation(axis, float(q[i]))
            T = T @ _transform(xyz, R_frame @ R_joint)
            if i in (0, 1, 2):  # shoulder, elbow, wrist_1 origins (joints 1,2,3)
                positions.append(T[:3, 3].copy())
        # EE (after all revolute joints + fixed joint)
        R_ee = _rpy_to_matrix(*self._EE_RPY)
        T_ee = T @ _transform(self._EE_XYZ, R_ee)
        positions.append(T_ee[:3, 3].copy())
        return np.array(positions)  # shape (5, 3)
