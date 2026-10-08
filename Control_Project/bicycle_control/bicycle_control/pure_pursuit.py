import math
import numpy as np

class PurePursuitController:
    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0), l_base=None):
        self.L = wheelbase
        self.kv = kv                                        # look-ahead time [s]
        self.l_min = l_min
        self.l_max = l_max
        self.l_base = l_min if l_base is None else l_base   # L_d0: offset at v = 0 [m]
        self.max_steer_rad = max_steer_rad

        # Path cache + search state used by find_target_waypoint()
        self._cache_key = None
        self._xy = None
        self._yaw = None
        self._ds = 1.0
        self._closed = False
        self._idx_hint = None

    def compute_lookahead(self, v):
        ld = self.kv * max(v, 0.0) + self.l_base
        return min(max(ld, self.l_min), self.l_max)

    def find_target_waypoint(self, x, y, lookahead, path_points):
        # --- (re)build numpy arrays only when the path actually changes -------------
        key = (len(path_points), tuple(path_points[0][:2]), tuple(path_points[-1][:2]))
        if key != self._cache_key:
            arr = np.asarray(path_points, dtype=float)
            self._xy, self._yaw = arr[:, :2], arr[:, 2]
            self._ds = max(float(np.mean(np.hypot(*np.diff(self._xy, axis=0).T))), 1e-3)
            gap = float(np.hypot(*(self._xy[0] - self._xy[-1])))
            self._closed = gap < max(2.0 * self._ds, 1.0)   # closed loop vs. open track
            self._cache_key, self._idx_hint = key, None
        xy, yaw, n, ds = self._xy, self._yaw, len(self._xy), self._ds

        # --- 1) nearest waypoint: windowed around last index, global if lost ---------
        i0 = None
        if self._idx_hint is not None:
            cand = self._idx_hint + np.arange(-(int(1.0 / ds) + 2), int(3.0 * self.l_max / ds) + 5)
            cand = cand % n if self._closed else np.clip(cand, 0, n - 1)
            dist2 = np.sum((xy[cand] - (x, y)) ** 2, axis=1)
            if dist2.min() < (2.0 * self.l_max) ** 2:
                i0 = int(cand[np.argmin(dist2)])
        if i0 is None:
            i0 = int(np.argmin(np.sum((xy - (x, y)) ** 2, axis=1)))
        self._idx_hint = i0

        # --- 2) walk forward from i0 until the distance to (x, y) reaches `lookahead` -
        j = i0 + np.arange(int(2.5 * lookahead / ds) + 3)
        j = j % n if self._closed else np.minimum(j, n - 1)
        d = np.hypot(xy[j, 0] - x, xy[j, 1] - y)
        hit = np.flatnonzero(d >= lookahead)

        if hit.size == 0:                  # open path ends / path shorter than horizon
            k = int(np.argmax(d))
            return int(j[k]), (xy[j[k], 0], xy[j[k], 1], yaw[j[k]])
        k = int(hit[0])
        if k == 0:                         # already farther than L_d from the path
            return int(j[0]), (xy[j[0], 0], xy[j[0], 1], yaw[j[0]])

        # --- 3) circle-segment intersection -> continuous target (no waypoint jumps) -
        p0, p1 = xy[j[k - 1]], xy[j[k]]
        seg, f = p1 - p0, p0 - (x, y)
        a, b, c = seg @ seg, 2.0 * (f @ seg), f @ f - lookahead ** 2
        t = (-b + math.sqrt(max(b * b - 4.0 * a * c, 0.0))) / (2.0 * a) if a > 1e-12 else 1.0
        t = min(max(t, 0.0), 1.0)
        dyaw = yaw[j[k]] - yaw[j[k - 1]]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        tgt = (p0[0] + t * seg[0], p0[1] + t * seg[1], yaw[j[k - 1]] + t * dyaw)
        return int(j[k]), tgt

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        dx, dy = target_pt[0] - x, target_pt[1] - y
        d2 = dx * dx + dy * dy                       # chord length^2 to the target (= L_d^2)
        if d2 < 1e-6:
            return 0.0
        # target lateral offset in the body frame (x forward, y left): y_l = L_d * sin(alpha)
        y_local = -math.sin(yaw) * dx + math.cos(yaw) * dy
        kappa = 2.0 * y_local / d2                   # arc curvature = 2 sin(alpha) / L_d
        delta = math.atan(self.L * kappa)            # = atan(2 L sin(alpha) / L_d)
        return float(min(max(delta, -self.max_steer_rad), self.max_steer_rad))