"""
OFFLINE (ROS-free) closed-loop benchmark.

Replays the 10 Hz sim/controller loop with the SAME controller classes used by
bicycle_control.controller_node (PID, VelocityProfiler, LateralPID, PurePursuit, MPC) and the
SAME plant equations/parameters as bicycle_sim.bicycle_model.Car (forward Euler, dt = 0.1 s).
It is NOT a ROS 2 run: no DDS latency, no timer jitter, command applied on the next tick.

Usage: python3 tools/offline_benchmark.py [--laps 3] [--modes lateral_pid pure_pursuit mpc]
"""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'bicycle_control'), str(ROOT / 'track_environment')]
from bicycle_control.longitudinal_pid import PIDLongitudinalController
from bicycle_control.velocity_profiler import VelocityProfiler
from bicycle_control.lateral_pid import LateralPIDController
from bicycle_control.pure_pursuit import PurePursuitController
from bicycle_control.mpc import KinematicBicycleMPC
from bicycle_control.path_utils import path_curvature, mpc_reference
from track_environment.track import Track

L, DT, K_A, C_D, C_R, VMAX, MAXS = 1.25, 0.1, 4.0, 0.005, 0.05, 25.0, math.radians(35)
TARGET = 4.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def step(x, thr, delta):
    thr, delta = min(max(thr, -1), 1), min(max(delta, -MAXS), MAXS)
    px, py, th, v = x
    dv = K_A * thr - (C_D * v * v + C_R * v)
    x = np.array([px + v * math.cos(th) * DT, py + v * math.sin(th) * DT,
                  wrap(th + v / L * math.tan(delta) * DT), min(max(v + dv * DT, 0.0), VMAX)])
    return x


class Path:
    def __init__(self, wps):
        a = np.array([(w['x'], w['y'], w['psi']) for w in wps])
        self.pts, self.xy = a, a[:, :2]
        n = len(a)
        self.kappa = path_curvature(a[:, :2], window=1.5, closed=True)
        self.ds = float(np.mean(np.hypot(*np.diff(a[:, :2], axis=0).T)))
        seg = np.hypot(*np.diff(a[:, :2], axis=0).T)
        self.cum = np.r_[0, np.cumsum(seg)]
        self.length = self.cum[-1]
        self.n = n

    def nearest(self, x, y):
        return int(np.argmin(np.sum((self.xy - (x, y)) ** 2, axis=1)))

    def project(self, x, y, yaw):
        i = self.nearest(x, y)
        best = None
        for a, b in (((i - 1) % self.n, i), (i, (i + 1) % self.n)):
            p1, p2 = self.xy[a], self.xy[b]
            d = p2 - p1
            l2 = d @ d
            if l2 < 1e-9:
                continue
            t = min(max(((x - p1[0]) * d[0] + (y - p1[1]) * d[1]) / l2, 0), 1)
            pr = p1 + t * d
            dist = math.hypot(x - pr[0], y - pr[1])
            if best is None or dist < best[0]:
                cross = d[0] * (y - p1[1]) - d[1] * (x - p1[0])
                s = self.cum[a] + t * math.sqrt(l2)
                best = (math.copysign(dist, cross), s, wrap(yaw - math.atan2(d[1], d[0])))
        return best  # cte, s, heading_err

    def preview_speed(self, prof, idx, v):
        span = int(max(3.0, 1.5 * v) / self.ds) + 1
        w = self.kappa[(idx + np.arange(span)) % self.n]
        return prof.compute_target_speed(float(w[np.argmax(np.abs(w))]), fallback_speed=TARGET)

    def mpc_ref(self, x, y, v, prof, horizon=10):
        return mpc_reference(self.pts, self.kappa, self.ds, self.nearest(x, y), v, prof, TARGET,
                             horizon=horizon, dt=DT)


def run(mode, path, laps, max_time=900.0):
    prof = VelocityProfiler(default_speed=TARGET, max_speed=7.5)
    pid = PIDLongitudinalController(kp=1.0, ki=0.1, kd=0.0, dt=DT)
    lat = LateralPIDController(kp=0.48, ki=0.0, kd=0.15, k_yaw=0.7, dt=DT)
    pp = PurePursuitController(wheelbase=L, kv=0.5, l_min=0.8, l_max=3.5, l_base=0.8)
    mpc = KinematicBicycleMPC(wheelbase=L, dt=DT, horizon=10, k_a=K_A, c_drag=C_D, c_roll=C_R,
                              delay_steps=1)
    p0 = path.pts[0]
    x = np.array([p0[0], p0[1], p0[2], 0.0])
    t, steer, thr = 0.0, 0.0, 0.0
    lap_start, last_s, started = 0.0, 0.0, False
    cur = dict(cte=[], spd=[], head=[])
    laps_out, all_cte, top, solve_ms = [], [], 0.0, []
    while t < max_time and len(laps_out) < laps:
        x = step(x, thr, steer); t += DT          # plant integrates last command
        v = x[3]
        cte, s, herr = path.project(x[0], x[1], x[2])
        if v > 0.1:
            started = True
        if not started:
            lap_start = t
        top = max(top, v)
        cur['cte'].append(abs(cte)); cur['spd'].append(v); cur['head'].append(abs(herr)); all_cte.append(abs(cte))
        if started and v > 0.1 and last_s > 0.75 * path.length and s < 0.25 * path.length:
            frac = (path.length - last_s) / max((path.length - last_s) + s, 1e-4)
            tc = t - DT + frac * DT
            c = np.array(cur['cte'])
            laps_out.append(dict(lap=len(laps_out) + 1, lap_time=tc - lap_start,
                                 mean_cte=c.mean(), max_cte=c.max(), rms_cte=float(np.sqrt((c ** 2).mean())),
                                 mean_speed=float(np.mean(cur['spd'])), max_speed=float(max(cur['spd']))))
            lap_start = tc; cur = dict(cte=[], spd=[], head=[])
        last_s = s
        if abs(cte) > 5.0:                          # left the track
            return dict(mode=mode, status='LEFT_TRACK at t=%.1f s' % t, laps=laps_out)
        # ---- controller tick
        if mode == 'lateral_pid':
            delta = lat.compute_steering(cte, herr)
            tv = path.preview_speed(prof, path.nearest(x[0], x[1]), v)
            thr = pid.compute(tv, v)
        elif mode == 'pure_pursuit':
            ld = pp.compute_lookahead(v)
            idx, tgt = pp.find_target_waypoint(x[0], x[1], ld, [tuple(r) for r in path.pts])
            delta = pp.compute_steering(x[0], x[1], x[2], tgt, ld)
            thr = pid.compute(path.preview_speed(prof, idx, v), v)
        else:
            ref = path.mpc_ref(x[0], x[1], v, prof)
            delta, thr = mpc.solve([x[0], x[1], x[2], v], ref, current_steer=steer)
            solve_ms.append(mpc.last_info.get('time_ms', 0.0))
        steer = float(delta)
    return dict(mode=mode, status='OK' if len(laps_out) >= laps else 'TIMEOUT', laps=laps_out,
                top_speed=top, mean_cte_all=float(np.mean(all_cte)), max_cte_all=float(np.max(all_cte)),
                rms_cte_all=float(np.sqrt(np.mean(np.square(all_cte)))),
                mpc_solve_ms_mean=float(np.mean(solve_ms)) if solve_ms else None,
                mpc_solve_ms_max=float(np.max(solve_ms)) if solve_ms else None)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--modes', nargs='+', default=['lateral_pid', 'pure_pursuit', 'mpc'])
    ap.add_argument('--track', default='centerline_0.csv')
    ap.add_argument('--out', default='benchmark_offline.json')
    a = ap.parse_args()
    path = Path_ = Path(Track(track_file=a.track).waypoints)
    print('track length %.1f m, %d pts' % (path.length, path.n))
    res = []
    for m in a.modes:
        t0 = time.time(); r = run(m, path, a.laps); r['wall_s'] = time.time() - t0; res.append(r)
        print(m, r['status'], [round(l['lap_time'], 2) for l in r['laps']], 'wall %.0fs' % r['wall_s'], flush=True)
    json.dump(res, open(a.out, 'w'), indent=2, default=float)
