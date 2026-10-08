"""Pure-NumPy path helpers shared by the controller node and the offline benchmark."""

import numpy as np


def path_curvature(xy, window=1.5, closed=True):
    """Signed curvature (1/m, + = left turn) at every waypoint.

    Waypoint spacing in the track CSVs is highly non-uniform (3 mm ... 1 m), so a per-point
    heading difference is dominated by noise. Instead the Menger curvature of three points
    located `window` metres behind / at / ahead of waypoint i (by arc length) is used:
        kappa = 2 * cross(b - a, c - b) / (|b - a| |c - b| |c - a|).
    """
    xy = np.asarray(xy, dtype=float)
    n = len(xy)
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    total = s[-1]
    kappa = np.zeros(n)
    for i in range(n):
        sa, sc = s[i] - window, s[i] + window
        if closed:
            sa, sc = sa % total, sc % total
        elif sa < 0.0 or sc > total:
            continue
        a = np.array([np.interp(sa, s, xy[:, 0]), np.interp(sa, s, xy[:, 1])])
        c = np.array([np.interp(sc, s, xy[:, 0]), np.interp(sc, s, xy[:, 1])])
        b = xy[i]
        ab, bc, ac = b - a, c - b, c - a
        den = np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ac)
        if den > 1e-9:
            kappa[i] = 2.0 * (ab[0] * bc[1] - ab[1] * bc[0]) / den
    return kappa


def mpc_reference(pts, kappa, ds, i0, v, profiler, target_speed, horizon=10, dt=0.1,
                  accel=1.5, closed=True, constant_speed=False):
    """Time-indexed MPC reference rows [x, y, yaw, v_ref]; row k is the pose k+1 steps ahead.

    The speed profile is curvature-limited AND dynamically reachable (v_k <= v0 + accel*(k+1)*dt),
    and the rows are spaced by the integrated reference speed (sum v_ref*dt), NOT by the measured
    speed. Otherwise a slow car asks for nearby targets and never accelerates.
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    idx = (i0 + np.arange(n + 1)) % n if closed else np.arange(i0, n)
    seg = pts[idx]
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(seg[:, :2], axis=0).T))]
    yaw = np.unwrap(seg[:, 2])
    v0 = max(float(v), 0.8)
    dist, vref, d = np.zeros(horizon), np.zeros(horizon), 0.0
    for k in range(horizon):
        j = (i0 + int(round(d / max(ds, 1e-3)))) % n
        v_lim = target_speed if constant_speed else profiler.compute_target_speed(
            kappa[j], fallback_speed=target_speed)
        vref[k] = min(v_lim, v0 + accel * (k + 1) * dt)
        d = min(d + vref[k] * dt, s[-1])
        dist[k] = d
    return np.column_stack([np.interp(dist, s, seg[:, 0]), np.interp(dist, s, seg[:, 1]),
                            np.interp(dist, s, yaw), vref])
