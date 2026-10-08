"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.1, kd=0.0, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_vel: float, current_vel: float) -> float:
        error = target_vel - current_vel

        # Integral with anti-windup: clamp the accumulated error
        self.integral = float(np.clip(self.integral + error * self.dt,
                                      -self.integral_limit, self.integral_limit))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp the command to the actuator range [-max_brake, +max_throttle]
        return float(np.clip(u, -self.max_brake, self.max_throttle))

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
