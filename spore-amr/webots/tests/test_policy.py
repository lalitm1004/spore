import pytest

from robot.policy import CompanionPolicy
from robot.protocol import Message


def status(t):
    return Message(kind="EVT", name="STATUS", fields={"t": t, "err": 0.0, "u": 0.0, "lost": 0})


def line_lost(t):
    return Message(kind="EVT", name="LINE_LOST", fields={"t": t})


def policy(**overrides):
    kwargs = dict(cruise_speed=6.0, min_speed=2.0, slowdown=0.5, mission_duration_s=60.0)
    kwargs.update(overrides)
    return CompanionPolicy(**kwargs)


def test_the_run_starts_by_commanding_the_cruise_speed():
    (command,) = policy().start()

    assert command.kind == "CMD"
    assert command.name == "SET_SPEED"
    assert command.fields["value"] == pytest.approx(6.0)


def test_losing_the_line_makes_the_companion_slow_the_robot_down():
    # Deciding the robot is going too fast is policy, not tight-loop control,
    # so it belongs here rather than in the firmware.
    pilot = policy()
    pilot.start()

    (command,) = pilot.on_event(line_lost(t=5.0))

    assert command.name == "SET_SPEED"
    assert command.fields["value"] == pytest.approx(3.0)


def test_the_commanded_speed_never_falls_below_the_floor():
    pilot = policy()
    pilot.start()

    for i in range(6):
        commands = pilot.on_event(line_lost(t=float(i)))

    assert commands[0].fields["value"] == pytest.approx(2.0)


def test_the_mission_stops_once_its_duration_has_elapsed():
    pilot = policy(mission_duration_s=10.0)
    pilot.start()

    assert pilot.on_event(status(t=9.0)) == []
    assert [c.name for c in pilot.on_event(status(t=10.0))] == ["STOP"]


def test_the_stop_command_is_only_sent_once():
    pilot = policy(mission_duration_s=10.0)
    pilot.start()
    pilot.on_event(status(t=10.0))

    assert pilot.on_event(status(t=11.0)) == []
