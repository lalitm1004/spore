"""Acceleration is ramped; deceleration is not.

Speed and turn accuracy are coupled. Coming out of a turn the robot carries a
few degrees of heading error, and lateral drift is v*sin(e) against 10 mm of
line -- at 0.12 m/s the PID has 1.19 s to recover it, at 0.36 m/s only 0.40 s.
Measured at 18 rad/s with no ramp, two robots of eight lost the line
immediately after a turn and halted. Leaving every turn from rest buys the loop
that time back without giving up speed on the straight.

This is the arithmetic of the ramp itself. The firmware loop it lives in needs
Webots, so it is reproduced here rather than imported.
"""

import pytest

from robot.config import ControlConfig


def ramp(commanded, target, *, accel, dt, turning=False):
    """The rule in robot/main.py, in one place so a test can hold it."""
    if turning:
        return 0.0
    if target <= commanded:
        return target          # slowing is immediate
    return min(target, commanded + accel * dt)


DT = 0.016


def test_a_stopped_robot_does_not_jump_to_cruise():
    assert ramp(0.0, 12.0, accel=12.0, dt=DT) == pytest.approx(12.0 * DT)


def test_reaching_cruise_takes_the_time_the_rate_implies():
    speed, t = 0.0, 0.0
    while speed < 12.0 - 1e-9 and t < 5.0:
        speed = ramp(speed, 12.0, accel=12.0, dt=DT)
        t += DT
    assert speed == pytest.approx(12.0)
    assert t == pytest.approx(1.0, abs=0.02), "12 rad/s at 12 rad/s^2 is one second"


def test_slowing_down_is_immediate():
    """A ramp on the way down would make the robot take a second to stop for an
    obstacle it can see now."""
    assert ramp(12.0, 0.0, accel=12.0, dt=DT) == 0.0
    assert ramp(12.0, 3.0, accel=12.0, dt=DT) == 3.0


def test_a_turn_puts_the_robot_back_to_rest():
    """The turn controller drives the wheels itself, and what matters is that
    line following resumes from rest rather than at whatever cruise was."""
    assert ramp(12.0, 12.0, accel=12.0, dt=DT, turning=True) == 0.0


def test_the_ramp_is_configurable_and_has_a_default():
    assert ControlConfig().accel_rad_s2 > 0.0
