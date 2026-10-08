"""
Target velocity profiler based on path curvature.
Limits the cornering speed with a lateral-acceleration budget: v_max = sqrt(a_lat_max / |kappa|).
"""

import math


class VelocityProfiler:
    """Generates curvature-limited target speeds."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Curvature-limited speed.

        `fallback_speed` is the cruise speed cap (e.g. the controller's target_speed). Without
        it, a straight road yields `max_speed`. The result is always within [0, max_speed].
        """
        base_speed = self.max_speed if fallback_speed is None else float(fallback_speed)
        abs_kappa = abs(float(kappa))
        if abs_kappa > 1e-6:
            target_speed = min(base_speed, math.sqrt(self.max_lat_accel / abs_kappa))
        else:
            target_speed = base_speed
        return float(max(0.0, min(target_speed, self.max_speed)))
