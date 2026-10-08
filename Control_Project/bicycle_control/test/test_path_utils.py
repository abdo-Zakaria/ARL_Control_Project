import numpy as np
from bicycle_control.path_utils import path_curvature, mpc_reference
from bicycle_control.velocity_profiler import VelocityProfiler


def _circle(r=10.0, n=400):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[r * np.cos(a), r * np.sin(a)], a


def test_curvature_of_circle_is_one_over_r_positive_left():
    xy, _ = _circle(10.0)
    k = path_curvature(xy, window=1.5, closed=True)
    assert np.allclose(k, 0.1, rtol=0.02)


def test_curvature_of_straight_line_is_zero():
    xy = np.c_[np.linspace(0, 50, 100), np.zeros(100)]
    assert np.allclose(path_curvature(xy, window=1.5, closed=False), 0.0, atol=1e-9)


def test_mpc_reference_is_reachable_and_spaced_by_reference_speed():
    xy = np.c_[np.linspace(0, 100, 201), np.zeros(201)]
    pts = np.c_[xy, np.zeros(201)]
    kappa = np.zeros(201)
    prof = VelocityProfiler(default_speed=4.0, max_speed=7.5)
    ref = mpc_reference(pts, kappa, 0.5, 0, 0.0, prof, 4.0, horizon=10, dt=0.1, closed=False)
    assert ref.shape == (10, 4)
    assert np.all(np.diff(ref[:, 3]) >= 0) and ref[0, 3] <= 0.8 + 0.15 + 1e-9
    assert np.all(np.diff(ref[:, 0]) > 0)
