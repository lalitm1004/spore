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


import json

from robot.telemetry import TelemetryLog


def test_rows_reach_disk_before_the_run_ends(tmp_path):
    # A run with no fixed duration must still produce telemetry, and buffering
    # every row would grow without bound under a container memory limit.
    log = TelemetryLog(tmp_path / "bot.csv", ["t", "error", "lost"])

    log.record({"t": 0.0, "error": 0.1, "lost": False})
    log.record({"t": 0.1, "error": -0.2, "lost": False})

    written = (tmp_path / "bot.csv").read_text().splitlines()
    assert written[0] == "t,error,lost"
    assert len(written) == 3


def test_summary_is_accumulated_without_retaining_every_row(tmp_path):
    log = TelemetryLog(tmp_path / "bot.csv", ["t", "error", "lost"])
    for i, error in enumerate([0.0, 0.2, -0.4, 0.2]):
        log.record({"t": i * 0.1, "error": error, "lost": i == 1})

    summary = log.close()

    assert summary["steps"] == 4
    assert summary["mean_abs_error"] == pytest.approx(0.2)
    assert summary["max_abs_error"] == pytest.approx(0.4)
    assert summary["duration_s"] == pytest.approx(0.3)
    assert summary["lost_time_s"] == pytest.approx(0.1)
    assert json.loads((tmp_path / "bot.summary.json").read_text()) == summary
