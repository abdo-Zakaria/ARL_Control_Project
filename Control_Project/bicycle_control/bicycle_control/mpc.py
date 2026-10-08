"""
Kinematic-bicycle MPC (simplified).

Model (Euler, dt):  x+ = x + v cos(psi) dt,  y+ = y + v sin(psi) dt,
                    psi+ = psi + v/L tan(delta) dt,  v+ = clip(v + a dt, 0, v_max)
Pose (x, y) is the REAR AXLE. REP-103: x forward, y left, delta > 0 = left.

Optimisation variables (all box-bounded, so L-BFGS-B stays exactly feasible):
    z = [ dd_0..dd_{N-1} | a_0..a_{N-1} ]
    delta_k = clip(delta_prev + dr * cumsum(dd)_k, +-max_steer)   dd in [-1, 1]  -> steering rate limit
    a_k     = sa * z_{N+k}  in [-max_brake, max_accel]
Cost: tracking error in the reference frame (lateral, longitudinal, heading, speed)
      + penalties on a and on the rates of delta and a.
The gradient is a forward difference evaluated for all 2N perturbations in ONE batched
NumPy rollout, so there is no hand-written adjoint to maintain.

Reference: rows [x, y, yaw, v_ref]; row i is the desired state i*dt after "now".
Sample it along the path at spacing v_ref*dt (NOT measured v*dt).
"""

import math
import time

import numpy as np
from scipy.optimize import minimize


class KinematicBicycleMPC:
    def __init__(self, wheelbase=1.25, dt=0.1, horizon=20,
                 max_steer_rad=math.radians(35.0), max_steer_rate=1.5,
                 max_accel=2.5, max_brake=3.0, max_speed=5.0,
                 k_a=4.0, c_drag=0.005, c_roll=0.05,
                 delay_steps=1, v_min=0.8, launch_accel=0.6, v_stop=0.1,
                 w_lat=30.0, w_lon=1.0, w_yaw=10.0, w_v=3.0,
                 w_accel=0.1, w_dsteer=3.0, w_daccel=0.5, terminal_scale=2.0,
                 maxiter=30, maxfun=100):

        
        self.L, self.dt, self.N = float(wheelbase), float(dt), int(horizon)
        self.max_steer = float(max_steer_rad)
        self.max_rate = float(max_steer_rate)
        self.max_accel, self.max_brake = float(max_accel), float(max_brake)
        self.max_speed = float(max_speed)
        self.k_a, self.c_drag, self.c_roll = float(k_a), float(c_drag), float(c_roll)
        self.delay_steps = max(int(delay_steps), 0)
        self.v_min, self.launch_accel, self.v_stop = float(v_min), float(launch_accel), float(v_stop)
        self.w = dict(lat=w_lat, lon=w_lon, yaw=w_yaw, v=w_v,
                      accel=w_accel, dsteer=w_dsteer, daccel=w_daccel)
        self.terminal_scale = float(terminal_scale)
        self.maxiter, self.maxfun = int(maxiter), int(maxfun)
        self.dr = self.max_rate * self.dt            # max steering change per step [rad]
        self.sa = max(self.max_accel, self.max_brake)
        self.reset()

    def reset(self):
        self._plan = None            # (delta[N], accel[N]) of the last solution
        self._a_applied = 0.0        # acceleration command in flight
        self.last_info = {}

    # ------------------------------------------------------------------ helpers
    def accel_to_throttle(self, accel, v):
        """Plant: v_dot = k_a*u - c_drag*v^2 - c_roll*v  ->  u = (a + drag(v)) / k_a."""
        v = max(float(v), 0.0)
        u = (accel + self.c_drag * v * v + self.c_roll * v) / self.k_a
        u = min(max(u, -1.0), 1.0)
        return 0.0 if (v < self.v_stop and u < 0.0) else float(u)   # never reverse

    def _rollout(self, x0, delta, acc):
        """Batched rollout. delta, acc: (B, n). Returns x, y, psi, v arrays of shape (B, n)."""
        B, n = delta.shape
        x = np.full(B, x0[0]); y = np.full(B, x0[1])
        psi = np.full(B, x0[2]); v = np.full(B, x0[3])
        out = np.empty((4, B, n))
        k_tan = np.tan(delta) / self.L
        for k in range(n):
            x = x + v * np.cos(psi) * self.dt
            y = y + v * np.sin(psi) * self.dt
            psi = psi + v * k_tan[:, k] * self.dt
            v = np.clip(v + acc[:, k] * self.dt, 0.0, self.max_speed)
            out[0, :, k], out[1, :, k], out[2, :, k], out[3, :, k] = x, y, psi, v
        return out

    def _decode(self, Z, delta_prev, n):
        delta = np.clip(delta_prev + np.cumsum(Z[:, :n] * self.dr, axis=1),
                        -self.max_steer, self.max_steer)
        return delta, Z[:, n:] * self.sa

    def _cost(self, Z, ctx):
        """Cost for every row of Z (B, 2n)."""
        n, w = ctx['n'], self.w
        delta, acc = self._decode(Z, ctx['delta_prev'], n)
        x, y, psi, v = self._rollout(ctx['x0'], delta, acc)
        ref = ctx['ref']                                    # (n, 4)
        c, s = np.cos(ref[:, 2]), np.sin(ref[:, 2])
        dx, dy = x - ref[:, 0], y - ref[:, 1]
        e_lon = c * dx + s * dy
        e_lat = -s * dx + c * dy
        e_yaw = np.arctan2(np.sin(psi - ref[:, 2]), np.cos(psi - ref[:, 2]))
        stage = (w['lat'] * e_lat ** 2 + w['lon'] * e_lon ** 2
                 + w['yaw'] * e_yaw ** 2 + w['v'] * (v - ref[:, 3]) ** 2) @ ctx['w_stage']
        d_delta = np.diff(delta, axis=1, prepend=ctx['delta_prev'])
        d_acc = np.diff(acc, axis=1, prepend=self._a_applied)
        return (stage + w['accel'] * np.sum(acc ** 2, axis=1)
                + w['dsteer'] * np.sum(d_delta ** 2, axis=1)
                + w['daccel'] * np.sum(d_acc ** 2, axis=1))

    # ------------------------------------------------------------------ public API
    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """x0 = [x, y, yaw, v]; current_steer = LAST COMMANDED steering [rad].
        Returns (steer_rad, throttle in [-1, 1]); diagnostics in self.last_info."""
        t0 = time.perf_counter()
        x0 = np.asarray(x0, dtype=float).ravel()[:4].copy()
        ref = np.atleast_2d(np.asarray(ref_trajectory, dtype=float))
        delta_prev = float(np.clip(current_steer, -self.max_steer, self.max_steer)) \
            if np.isfinite(current_steer) else 0.0
        if (x0.size < 4 or not np.all(np.isfinite(x0)) or ref.shape[1] < 3
                or len(ref) < 2 or not np.all(np.isfinite(ref))):
            self.last_info = {'status': 'bad_input'}          # hold steering, creep if stopped
            v = float(x0[3]) if x0.size >= 4 and np.isfinite(x0[3]) else 0.0
            a = self.launch_accel if v < 0.8 * self.v_min else 0.0
            self._a_applied = a
            return delta_prev, self.accel_to_throttle(a, v)
        if ref.shape[1] < 4:
            ref = np.c_[ref[:, :3], np.full(len(ref), x0[3])]
        x0[3] = max(x0[3], 0.0)

        # actuator delay: propagate with the command already in flight
        d = self.delay_steps
        if d > 0:
            xd = self._rollout(x0, np.full((1, d), delta_prev), np.full((1, d), self._a_applied))
            x0 = xd[:, 0, -1]

        n = min(self.N, max(len(ref) - d, 2))
        ref = ref[np.minimum(np.arange(n) + d, len(ref) - 1), :4]
        w_stage = np.ones(n)
        w_stage[-1] = self.terminal_scale
        ctx = dict(x0=x0, ref=ref, n=n, delta_prev=delta_prev, w_stage=w_stage)

        # bounds (+ launch push when (almost) stopped and the reference wants motion)
        lo = np.r_[np.full(n, -1.0), np.full(n, -self.max_brake / self.sa)]
        hi = np.r_[np.full(n, 1.0), np.full(n, self.max_accel / self.sa)]
        if x0[3] < 0.8 * self.v_min and ref[0, 3] > self.v_min:
            lo[n] = min(self.launch_accel, self.max_accel) / self.sa

        # warm start: previous plan shifted by one step, else hold steering + P-speed accel
        if self._plan is not None:
            ds = np.r_[self._plan[0][1:], self._plan[0][-1]][:n]
            as_ = np.r_[self._plan[1][1:], self._plan[1][-1]][:n]
        else:
            ds = np.full(n, delta_prev)
            as_ = np.full(n, np.clip(0.5 * (ref[:, 3].mean() - x0[3]), -self.max_brake, self.max_accel))
        z0 = np.clip(np.r_[np.diff(np.r_[delta_prev, ds]) / self.dr, as_ / self.sa], lo, hi)

        eye = 1e-5 * np.eye(2 * n)

        def fun(z):                                 # value + forward-difference gradient, 1 batch
            J = self._cost(np.vstack([z, z + eye]), ctx)
            return J[0], (J[1:] - J[0]) / 1e-5

        res = minimize(fun, z0, jac=True, method='L-BFGS-B', bounds=list(zip(lo, hi)),
                       options={'maxiter': self.maxiter, 'maxfun': self.maxfun, 'maxls': 10})
        delta, acc = self._decode(np.clip(res.x, lo, hi)[None, :], delta_prev, n)
        pad = self.N - n
        self._plan = (np.r_[delta[0], np.full(pad, delta[0, -1])],
                      np.r_[acc[0], np.full(pad, acc[0, -1])])
        self._a_applied = float(acc[0, 0])
        msg = str(res.message)
        status = 'optimal' if res.success else ('maxiter' if 'LIMIT' in msg.upper() else msg)
        self.last_info = {'status': status,
                          'nit': int(res.nit), 'nfev': int(res.nfev), 'cost': float(res.fun),
                          'time_ms': 1e3 * (time.perf_counter() - t0), 'horizon': n}
        return float(delta[0, 0]), self.accel_to_throttle(self._a_applied, x0[3])