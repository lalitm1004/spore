import pytest

from robot.events import EventDetector


def names(messages):
    return [m.name for m in messages]


def edges(messages):
    """Transition events only; STATUS is periodic and reported separately."""
    return [m.name for m in messages if m.name != "STATUS"]


def test_line_lost_is_reported_once_on_the_transition():
    detector = EventDetector(status_period_s=100.0)

    assert edges(detector.update(t=0.0, lost=False, error=0.0, u=0.0)) == []
    assert edges(detector.update(t=0.1, lost=True, error=0.0, u=0.0)) == ["LINE_LOST"]
    assert edges(detector.update(t=0.2, lost=True, error=0.0, u=0.0)) == []


def test_line_found_is_reported_when_the_line_is_reacquired():
    detector = EventDetector(status_period_s=100.0)
    detector.update(t=0.0, lost=True, error=0.0, u=0.0)

    assert edges(detector.update(t=0.3, lost=False, error=0.0, u=0.0)) == ["LINE_FOUND"]


def test_events_carry_the_time_they_happened():
    detector = EventDetector(status_period_s=100.0)

    messages = detector.update(t=4.25, lost=True, error=0.01, u=1.5)
    (event,) = [m for m in messages if m.name == "LINE_LOST"]

    assert event.kind == "EVT"
    assert event.fields["t"] == 4.25


def test_status_is_reported_on_its_own_schedule():
    detector = EventDetector(status_period_s=1.0)

    assert names(detector.update(t=0.0, lost=False, error=0.0, u=0.0)) == ["STATUS"]
    assert names(detector.update(t=0.5, lost=False, error=0.0, u=0.0)) == []
    assert names(detector.update(t=1.0, lost=False, error=0.002, u=0.3)) == ["STATUS"]


def test_status_carries_the_tracking_numbers():
    detector = EventDetector(status_period_s=1.0)

    (status,) = [m for m in detector.update(t=0.0, lost=False, error=0.0025, u=0.4)
                 if m.name == "STATUS"]

    assert status.fields["err"] == pytest.approx(0.0025)
    assert status.fields["u"] == pytest.approx(0.4)
    assert status.fields["lost"] == 0
