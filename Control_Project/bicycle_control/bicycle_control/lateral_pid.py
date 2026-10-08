"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = None

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians."""
        #
        heading_err_norm = math.atan2(math.sin(heading_err), math.cos(heading_err))

        #
        #
        error_cte = -float(cte)

        #
        self.integral_cte += error_cte * self.dt
        self.integral_cte = float(np.clip(self.integral_cte, -self.integral_limit, self.integral_limit))

        #
        d_cte = 0.0 if self.prev_cte is None else (error_cte - self.prev_cte) / self.dt
        self.prev_cte = error_cte

        #
        p_term = self.kp * error_cte
        i_term = self.ki * self.integral_cte
        d_term = self.kd * d_cte
        yaw_term = -self.k_yaw * heading_err_norm

        raw_steer = p_term + i_term + d_term + yaw_term

        #
        steer_rad = float(np.clip(raw_steer, -self.max_steer_rad, self.max_steer_rad))

        return steer_rad

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = None
