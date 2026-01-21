"""
OPW Kinematics wrapper.

OPW provides closed-form inverse kinematics for industrial 6R arms.

This module intentionally:
- Does NOT do planning
- Does NOT do collision checking
- Does NOT choose solutions

It only answers:
"Given a pose, what joint configurations are kinematically valid?"
"""

from core.robot_model import RobotModel
from core.kinematics.opw_parameters import OPWParameters

import numpy as np
from typing import Optional


class OPWKinematics:
    """
    OPW closed-form inverse kinematics for 6DOF industrial robots.

    Based on the algorithm from opw_kinematics C++ library.
    Returns up to 8 kinematically unique solutions.
    """

    def __init__(self, robot_model: RobotModel, params: Optional[OPWParameters] = None):
        """Initialize the OPW kinematics solver."""
        self.robot = robot_model
        if self.robot.dof() != 6:
            raise ValueError('OPW kinematics only supports 6DOF arms')

        if params is None:
            params = OPWParameters()
        self.params = params

    def inverse_kinematics(self, pose):
        """
        Compute all IK solutions for a desired end-effector pose.

        Args:
            pose: End-effector pose as 4x4 transformation matrix or dict with
                  'translation' (3,) and 'rotation' (3x3) or 'quaternion'

        Returns:
            Array of shape (8, 6) with up to 8 solutions. Invalid solutions
            contain NaN values.
        """
        # Convert pose to transformation matrix
        if isinstance(pose, dict):
            T = self._pose_dict_to_matrix(pose)
        elif isinstance(pose, np.ndarray):
            if pose.shape == (4, 4):
                T = pose
            else:
                raise ValueError('Pose array must be 4x4 transformation matrix')
        else:
            raise ValueError('Pose must be 4x4 array or dict with translation/rotation')

        # Extract translation and rotation
        translation = T[:3, 3]
        rotation = T[:3, :3]

        # Adjust to wrist center
        z_axis = rotation @ np.array([0, 0, 1])
        c = translation - self.params.c4 * z_axis

        # Compute theta1 solutions
        nx1 = np.sqrt(c[0] * c[0] + c[1] * c[1] - self.params.b * self.params.b) - self.params.a1

        tmp1 = np.arctan2(c[1], c[0])
        tmp2 = np.arctan2(self.params.b, nx1 + self.params.a1)
        theta1_i = tmp1 - tmp2
        theta1_ii = tmp1 + tmp2 - np.pi

        # Compute theta2 and theta3 solutions
        tmp3 = c[2] - self.params.c1
        s1_2 = nx1 * nx1 + tmp3 * tmp3
        tmp4 = nx1 + 2.0 * self.params.a1
        s2_2 = tmp4 * tmp4 + tmp3 * tmp3
        kappa_2 = self.params.a2 * self.params.a2 + self.params.c3 * self.params.c3
        c2_2 = self.params.c2 * self.params.c2

        tmp5 = s1_2 + c2_2 - kappa_2
        s1 = np.sqrt(s1_2)
        s2 = np.sqrt(s2_2)

        theta2_i = -np.arccos(tmp5 / (2.0 * s1 * self.params.c2)) + np.arctan2(nx1, tmp3)
        theta2_ii = np.arccos(tmp5 / (2.0 * s1 * self.params.c2)) + np.arctan2(nx1, tmp3)

        tmp6 = s2_2 + c2_2 - kappa_2
        theta2_iii = -np.arccos(tmp6 / (2.0 * s2 * self.params.c2)) - np.arctan2(tmp4, tmp3)
        theta2_iv = np.arccos(tmp6 / (2.0 * s2 * self.params.c2)) - np.arctan2(tmp4, tmp3)

        # theta3
        tmp7 = s1_2 - c2_2 - kappa_2
        tmp8 = s2_2 - c2_2 - kappa_2
        tmp9 = 2.0 * self.params.c2 * np.sqrt(kappa_2)
        theta3_i = np.arccos(tmp7 / tmp9) - np.arctan2(self.params.a2, self.params.c3)
        theta3_ii = -np.arccos(tmp7 / tmp9) - np.arctan2(self.params.a2, self.params.c3)
        theta3_iii = np.arccos(tmp8 / tmp9) - np.arctan2(self.params.a2, self.params.c3)
        theta3_iv = -np.arccos(tmp8 / tmp9) - np.arctan2(self.params.a2, self.params.c3)

        # Orientation part
        sin1 = np.array([np.sin(theta1_i), np.sin(theta1_i), np.sin(theta1_ii), np.sin(theta1_ii)])
        cos1 = np.array([np.cos(theta1_i), np.cos(theta1_i), np.cos(theta1_ii), np.cos(theta1_ii)])

        s23 = np.array([
            np.sin(theta2_i + theta3_i),
            np.sin(theta2_ii + theta3_ii),
            np.sin(theta2_iii + theta3_iii),
            np.sin(theta2_iv + theta3_iv),
        ])
        c23 = np.array([
            np.cos(theta2_i + theta3_i),
            np.cos(theta2_ii + theta3_ii),
            np.cos(theta2_iii + theta3_iii),
            np.cos(theta2_iv + theta3_iv),
        ])

        m = np.array([
            rotation[0, 2] * s23[0] * cos1[0] + rotation[1, 2] * s23[0] * sin1[0] + rotation[2, 2] * c23[0],
            rotation[0, 2] * s23[1] * cos1[1] + rotation[1, 2] * s23[1] * sin1[1] + rotation[2, 2] * c23[1],
            rotation[0, 2] * s23[2] * cos1[2] + rotation[1, 2] * s23[2] * sin1[2] + rotation[2, 2] * c23[2],
            rotation[0, 2] * s23[3] * cos1[3] + rotation[1, 2] * s23[3] * sin1[3] + rotation[2, 2] * c23[3],
        ])

        # theta4
        theta4_i = np.arctan2(
            rotation[1, 2] * cos1[0] - rotation[0, 2] * sin1[0],
            rotation[0, 2] * c23[0] * cos1[0] + rotation[1, 2] * c23[0] * sin1[0] - rotation[2, 2] * s23[0]
        )
        theta4_ii = np.arctan2(
            rotation[1, 2] * cos1[1] - rotation[0, 2] * sin1[1],
            rotation[0, 2] * c23[1] * cos1[1] + rotation[1, 2] * c23[1] * sin1[1] - rotation[2, 2] * s23[1]
        )
        theta4_iii = np.arctan2(
            rotation[1, 2] * cos1[2] - rotation[0, 2] * sin1[2],
            rotation[0, 2] * c23[2] * cos1[2] + rotation[1, 2] * c23[2] * sin1[2] - rotation[2, 2] * s23[2]
        )
        theta4_iv = np.arctan2(
            rotation[1, 2] * cos1[3] - rotation[0, 2] * sin1[3],
            rotation[0, 2] * c23[3] * cos1[3] + rotation[1, 2] * c23[3] * sin1[3] - rotation[2, 2] * s23[3]
        )

        theta4_v = theta4_i + np.pi
        theta4_vi = theta4_ii + np.pi
        theta4_vii = theta4_iii + np.pi
        theta4_viii = theta4_iv + np.pi

        # theta5
        theta5_i = np.arctan2(np.sqrt(np.clip(1 - m[0] * m[0], 0, 1)), m[0])
        theta5_ii = np.arctan2(np.sqrt(np.clip(1 - m[1] * m[1], 0, 1)), m[1])
        theta5_iii = np.arctan2(np.sqrt(np.clip(1 - m[2] * m[2], 0, 1)), m[2])
        theta5_iv = np.arctan2(np.sqrt(np.clip(1 - m[3] * m[3], 0, 1)), m[3])

        theta5_v = -theta5_i
        theta5_vi = -theta5_ii
        theta5_vii = -theta5_iii
        theta5_viii = -theta5_iv

        # theta6
        theta6_i = np.arctan2(
            rotation[0, 1] * s23[0] * cos1[0] + rotation[1, 1] * s23[0] * sin1[0] + rotation[2, 1] * c23[0],
            -rotation[0, 0] * s23[0] * cos1[0] - rotation[1, 0] * s23[0] * sin1[0] - rotation[2, 0] * c23[0]
        )
        theta6_ii = np.arctan2(
            rotation[0, 1] * s23[1] * cos1[1] + rotation[1, 1] * s23[1] * sin1[1] + rotation[2, 1] * c23[1],
            -rotation[0, 0] * s23[1] * cos1[1] - rotation[1, 0] * s23[1] * sin1[1] - rotation[2, 0] * c23[1]
        )
        theta6_iii = np.arctan2(
            rotation[0, 1] * s23[2] * cos1[2] + rotation[1, 1] * s23[2] * sin1[2] + rotation[2, 1] * c23[2],
            -rotation[0, 0] * s23[2] * cos1[2] - rotation[1, 0] * s23[2] * sin1[2] - rotation[2, 0] * c23[2]
        )
        theta6_iv = np.arctan2(
            rotation[0, 1] * s23[3] * cos1[3] + rotation[1, 1] * s23[3] * sin1[3] + rotation[2, 1] * c23[3],
            -rotation[0, 0] * s23[3] * cos1[3] - rotation[1, 0] * s23[3] * sin1[3] - rotation[2, 0] * c23[3]
        )

        theta6_v = theta6_i - np.pi
        theta6_vi = theta6_ii - np.pi
        theta6_vii = theta6_iii - np.pi
        theta6_viii = theta6_iv - np.pi

        # Build all 8 solutions
        solutions = np.zeros((8, 6))
        solutions[0] = np.array([
            (theta1_i + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_i + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_i + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_i + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_i + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_i + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[1] = np.array([
            (theta1_i + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_ii + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_ii + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_ii + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_ii + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_ii + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[2] = np.array([
            (theta1_ii + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_iii + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_iii + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_iii + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_iii + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_iii + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[3] = np.array([
            (theta1_ii + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_iv + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_iv + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_iv + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_iv + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_iv + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[4] = np.array([
            (theta1_i + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_i + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_i + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_v + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_v + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_v + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[5] = np.array([
            (theta1_i + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_ii + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_ii + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_vi + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_vi + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_vi + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[6] = np.array([
            (theta1_ii + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_iii + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_iii + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_vii + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_vii + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_vii + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])
        solutions[7] = np.array([
            (theta1_ii + self.params.offsets[0]) * self.params.sign_corrections[0],
            (theta2_iv + self.params.offsets[1]) * self.params.sign_corrections[1],
            (theta3_iv + self.params.offsets[2]) * self.params.sign_corrections[2],
            (theta4_viii + self.params.offsets[3]) * self.params.sign_corrections[3],
            (theta5_viii + self.params.offsets[4]) * self.params.sign_corrections[4],
            (theta6_viii + self.params.offsets[5]) * self.params.sign_corrections[5],
        ])

        return solutions

    def forward_kinematics(self, q):
        """
        Compute forward kinematics for a joint configuration.

        Args:
            q: Joint configuration, shape (6,)

        Returns:
            4x4 transformation matrix
        """
        # Apply sign corrections and offsets
        q_corrected = q.copy()
        for i in range(6):
            q_corrected[i] = q[i] * self.params.sign_corrections[i] - self.params.offsets[i]

        psi3 = np.arctan2(self.params.a2, self.params.c3)
        k = np.sqrt(self.params.a2 * self.params.a2 + self.params.c3 * self.params.c3)

        cx1 = self.params.c2 * np.sin(q_corrected[1]) + k * np.sin(q_corrected[1] + q_corrected[2] + psi3) + self.params.a1
        cy1 = self.params.b
        cz1 = self.params.c2 * np.cos(q_corrected[1]) + k * np.cos(q_corrected[1] + q_corrected[2] + psi3)

        cx0 = cx1 * np.cos(q_corrected[0]) - cy1 * np.sin(q_corrected[0])
        cy0 = cx1 * np.sin(q_corrected[0]) + cy1 * np.cos(q_corrected[0])
        cz0 = cz1 + self.params.c1

        s = np.sin(q_corrected)
        c = np.cos(q_corrected)

        # Rotation matrix r_0c
        r_0c = np.array([
            [c[0] * c[1] * c[2] - c[0] * s[1] * s[2], -s[0], c[0] * c[1] * s[2] + c[0] * s[1] * c[2]],
            [s[0] * c[1] * c[2] - s[0] * s[1] * s[2], c[0], s[0] * c[1] * s[2] + s[0] * s[1] * c[2]],
            [-s[1] * c[2] - c[1] * s[2], 0, -s[1] * s[2] + c[1] * c[2]],
        ])

        # Rotation matrix r_ce
        r_ce = np.array([
            [c[3] * c[4] * c[5] - s[3] * s[5], -c[3] * c[4] * s[5] - s[3] * c[5], c[3] * s[4]],
            [s[3] * c[4] * c[5] + c[3] * s[5], -s[3] * c[4] * s[5] + c[3] * c[5], s[3] * s[4]],
            [-s[4] * c[5], s[4] * s[5], c[4]],
        ])

        r_oe = r_0c @ r_ce
        u = np.array([cx0, cy0, cz0]) + self.params.c4 * (r_oe @ np.array([0, 0, 1]))

        T = np.eye(4)
        T[:3, :3] = r_oe
        T[:3, 3] = u

        return T

    def _pose_dict_to_matrix(self, pose):
        """Convert pose dict to 4x4 transformation matrix."""
        T = np.eye(4)
        T[:3, 3] = np.array(pose['translation'])

        if 'rotation' in pose:
            T[:3, :3] = np.array(pose['rotation'])
        elif 'quaternion' in pose:
            q = np.array(pose['quaternion'])
            # Convert quaternion to rotation matrix
            w, x, y, z = q
            T[:3, :3] = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
            ])
        else:
            raise ValueError('Pose dict must have rotation or quaternion')

        return T

    def solve(self, pose):
        """
        Compute all IK solutions for a desired end-effector pose.

        Args:
            pose: End-effector pose (4x4 matrix, or dict with translation/rotation)

        Returns:
            List of valid joint vectors, each shape (6,)
        """
        solutions = self.inverse_kinematics(pose)
        # Filter out NaN solutions
        valid = []
        for sol in solutions:
            if not np.any(np.isnan(sol)):
                valid.append(sol)
        return valid

    def filter_joint_limits(self, solutions):
        """Discard solutions outside joint limits."""
        lower, upper = self.robot.joint_limits()
        valid = []
        for q in solutions:
            if np.all(q >= lower) and np.all(q <= upper):
                valid.append(q)
        return valid
