import pytest

from robot.telemetry import summarise


def rows(*errors, dt=0.1, lost=()):
    return [
        {"t": i * dt, "error": e, "lost": i in lost}
        for i, e in enumerate(errors)
    ]


def test_summary_reports_the_run_length_and_step_count():
    summary = summarise(rows(0.0, 0.1, -0.1, 0.0))

    assert summary["steps"] == 4
    assert summary["duration_s"] == pytest.approx(0.3)


def test_summary_reports_mean_and_max_absolute_error():
    summary = summarise(rows(0.0, 0.2, -0.4, 0.2))

    assert summary["mean_abs_error"] == pytest.approx(0.2)
    assert summary["max_abs_error"] == pytest.approx(0.4)


def test_summary_reports_how_long_the_line_was_lost():
    summary = summarise(rows(0.0, 0.0, 0.0, 0.0, lost=(1, 2)))

    assert summary["lost_time_s"] == pytest.approx(0.2)


def test_summarising_an_empty_run_does_not_divide_by_zero():
    summary = summarise([])

    assert summary["steps"] == 0
    assert summary["mean_abs_error"] == 0.0
    assert summary["duration_s"] == 0.0
