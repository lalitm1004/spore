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


# --------------------------------------------------------- getting it back ---

def test_a_clean_run_of_line_restores_the_speed_that_was_given_up():
    """The throttle used to be a one-way ratchet: nothing ever raised the
    speed again, so a single transient left the robot at the floor for the
    whole run. Slowing down is a response to conditions, and conditions pass."""
    pilot = policy(recover_after_s=1.0, speedup=2.0)
    pilot.start()
    pilot.on_event(line_lost(t=5.0))          # 6.0 -> 3.0

    assert pilot.on_event(status(t=5.5)) == []          # not long enough yet
    (command,) = pilot.on_event(status(t=6.0))

    assert command.name == "SET_SPEED"
    assert command.fields["value"] == pytest.approx(6.0)


def test_recovery_never_exceeds_the_cruise_speed():
    pilot = policy(recover_after_s=1.0, speedup=2.0)
    pilot.start()
    pilot.on_event(line_lost(t=5.0))

    pilot.on_event(status(t=6.0))             # back to cruise
    assert pilot.on_event(status(t=7.0)) == []


def test_losing_the_line_again_restarts_the_clean_run():
    pilot = policy(recover_after_s=1.0, speedup=2.0)
    pilot.start()
    pilot.on_event(line_lost(t=5.0))
    pilot.on_event(line_lost(t=5.5))          # 3.0 -> 2.0 (floor)

    assert pilot.on_event(status(t=6.0)) == []   # clock restarted at 5.5


def test_recovery_is_off_by_default():
    pilot = policy()
    pilot.start()
    pilot.on_event(line_lost(t=5.0))

    assert pilot.on_event(status(t=50.0)) == []
