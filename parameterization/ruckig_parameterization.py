"""
Ruckig time parameterizer.

This module uses the Ruckig Python bindings to generate a
jerk‑limited trajectory from a joint‑space path.

Install Ruckig first:
    pip install ruckig
"""

import numpy as np

try:
    import ruckig
except ImportError as e:
    raise ImportError(
        "Ruckig Python bindings are required. Install via:\n"
        "    pip install ruckig"
    ) from e


class RuckigTimeParameterizer:
    """
    Ruckig‑based time parameterizer.

    Accepts:
      - path: list of joint configurations [q0, ..., qN]
      - velocity limits (rad/s)
      - acceleration limits (rad/s^2)
      - jerk limits (rad/s^3)

    Produces:
      - time stamps
      - trajectory array [t, q(t)]
    """

    def __init__(self, v_max: np.ndarray, a_max: np.ndarray, j_max: np.ndarray):
        assert v_max.shape == a_max.shape == j_max.shape
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max
        self.dof = v_max.shape[0]

    def compute(self, path, dt=0.01):
        """
        Time‑parameterize a joint path using Ruckig.

        Args:
            path: list of joint configurations
            dt: sample interval in seconds

        Returns:
            time_stamps: list of times
            trajectory: numpy array (len(time_stamps), dof)
        """
        # Convert to sequences of segments
        q0 = np.array(path[0])
        qf = np.array(path[-1])

        # Create a Ruckig instance with our DOF and control cycle dt
        otg = ruckig.Ruckig(self.dof, dt)

        # Input and output parameter objects
        inp = ruckig.InputParameter(self.dof)
        out = ruckig.OutputParameter(self.dof)

        # Set initial state: zero velocity & acceleration
        inp.current_position = q0.tolist()
        inp.current_velocity = [0.0]*self.dof
        inp.current_acceleration = [0.0]*self.dof

        # Target state: end of path (we only use positions here)
        inp.target_position = qf.tolist()
        inp.target_velocity = [0.0]*self.dof
        inp.target_acceleration = [0.0]*self.dof

        # Apply limits
        inp.max_velocity = self.v_max.tolist()
        inp.max_acceleration = self.a_max.tolist()
        inp.max_jerk = self.j_max.tolist()

        # Validate input (optional but useful)
        otg.validate_input(inp)

        # Iterate to generate trajectory
        time_stamps = [0.0]
        trajectory = [q0.copy()]

        result = otg.update(inp, out)

        # Loop until the motion is finished
        while result == ruckig.Result.Working:
            # Append outgoing state
            trajectory.append(np.array(out.new_position))
            time_stamps.append(out.time)
            # Pass output to next input step
            out.pass_to_input(inp)
            result = otg.update(inp, out)

        return np.array(time_stamps), np.vstack(trajectory)
