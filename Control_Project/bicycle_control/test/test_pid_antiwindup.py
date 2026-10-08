from bicycle_control.longitudinal_pid import PIDLongitudinalController


def test_integrator_is_clamped_under_saturation():
    pid = PIDLongitudinalController(kp=1.0, ki=0.5, dt=0.1, integral_limit=2.0)
    for _ in range(500):
        u = pid.compute(10.0, 0.0)
    assert u == 1.0 and abs(pid.integral) <= 2.0
