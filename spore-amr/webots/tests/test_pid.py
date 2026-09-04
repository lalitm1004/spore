import pytest

from robot.pid import PID


def test_proportional_term_scales_the_error():
    pid = PID(kp=2.0, ki=0.0, kd=0.0)

    output = pid.update(error=0.5, dt=0.1)

    assert output.p == pytest.approx(1.0)
    assert output.u == pytest.approx(1.0)


def test_integral_term_accumulates_error_over_time():
    pid = PID(kp=0.0, ki=1.0, kd=0.0)

    pid.update(error=1.0, dt=0.1)
    output = pid.update(error=1.0, dt=0.1)

    assert output.i == pytest.approx(0.2)


def test_derivative_term_responds_to_the_rate_of_change():
    pid = PID(kp=0.0, ki=0.0, kd=1.0)

    first = pid.update(error=0.0, dt=0.1)
    second = pid.update(error=0.5, dt=0.1)

    assert first.d == pytest.approx(0.0)  # no kick on the first step
    assert second.d == pytest.approx(5.0)


def test_reset_clears_accumulated_state():
    pid = PID(kp=0.0, ki=1.0, kd=0.0)
    pid.update(error=1.0, dt=0.1)

    pid.reset()
    output = pid.update(error=1.0, dt=0.1)

    assert output.i == pytest.approx(0.1)


def test_output_is_clamped_to_the_limit():
    pid = PID(kp=10.0, ki=0.0, kd=0.0, output_limit=2.0)

    assert pid.update(error=1.0, dt=0.1).u == pytest.approx(2.0)
    assert pid.update(error=-1.0, dt=0.1).u == pytest.approx(-2.0)


def test_integral_stops_growing_while_the_output_is_saturated():
    # Without anti-windup the integral would reach 2.0 over 20 steps and the
    # controller would stay saturated long after the error reversed.
    pid = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=0.5)

    for _ in range(20):
        output = pid.update(error=1.0, dt=0.1)

    assert output.i == pytest.approx(0.5)


def test_integral_unwinds_once_the_error_reverses():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=0.5)
    for _ in range(20):
        pid.update(error=1.0, dt=0.1)

    output = pid.update(error=-1.0, dt=0.1)

    assert output.i == pytest.approx(0.4)
