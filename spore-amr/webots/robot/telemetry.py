"""Telemetry recording and run summaries."""

import csv
import json
import pathlib
from typing import Sequence


def summarise(rows: Sequence[dict]) -> dict:
    """Reduce a run's telemetry rows to the numbers a comparison needs."""
    if not rows:
        return {
            "steps": 0,
            "duration_s": 0.0,
            "mean_abs_error": 0.0,
            "max_abs_error": 0.0,
            "lost_time_s": 0.0,
        }

    errors = [abs(row["error"]) for row in rows]
    duration = rows[-1]["t"] - rows[0]["t"]
    step = duration / (len(rows) - 1) if len(rows) > 1 else 0.0
    lost_steps = sum(1 for row in rows if row.get("lost"))

    return {
        "steps": len(rows),
        "duration_s": duration,
        "mean_abs_error": sum(errors) / len(errors),
        "max_abs_error": max(errors),
        "lost_time_s": lost_steps * step,
    }


class TelemetryLog:
    """Streams rows to CSV as they are recorded.

    Rows are written and flushed immediately rather than buffered: a run with no
    fixed duration must still produce telemetry, and holding every row would
    grow without bound inside a container memory limit. The summary is
    accumulated incrementally for the same reason.
    """

    def __init__(self, path: pathlib.Path, columns: Sequence[str]):
        self.path = pathlib.Path(path)
        self.columns = list(columns)
        self.path.parent.mkdir(parents=True, exist_ok=True)

        self._handle = self.path.open("w", newline="")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.columns)
        self._writer.writeheader()
        self._handle.flush()

        self._steps = 0
        self._abs_error_total = 0.0
        self._max_abs_error = 0.0
        self._lost_steps = 0
        self._first_t = None
        self._last_t = None

    def record(self, row: dict) -> None:
        self._writer.writerow(row)
        self._handle.flush()

        error = abs(row["error"])
        self._steps += 1
        self._abs_error_total += error
        self._max_abs_error = max(self._max_abs_error, error)
        if row.get("lost"):
            self._lost_steps += 1
        if self._first_t is None:
            self._first_t = row["t"]
        self._last_t = row["t"]

    def summary(self) -> dict:
        if self._steps == 0:
            return summarise([])

        duration = self._last_t - self._first_t
        step = duration / (self._steps - 1) if self._steps > 1 else 0.0
        return {
            "steps": self._steps,
            "duration_s": duration,
            "mean_abs_error": self._abs_error_total / self._steps,
            "max_abs_error": self._max_abs_error,
            "lost_time_s": self._lost_steps * step,
        }

    def close(self) -> dict:
        self._handle.close()
        summary = self.summary()
        self.path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2))
        return summary
