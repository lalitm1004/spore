"""Telemetry recording and run summaries."""

import csv
import json
import pathlib
from typing import Dict, List, Sequence


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
    """Buffers rows in memory and writes a CSV plus a summary on close."""

    def __init__(self, path: pathlib.Path, columns: Sequence[str]):
        self.path = pathlib.Path(path)
        self.columns = list(columns)
        self.rows: List[Dict] = []

    def record(self, row: dict) -> None:
        self.rows.append(row)

    def close(self) -> dict:
        self.path.parent.mkdir(parents=True, exist_ok=True)

        with self.path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.columns)
            writer.writeheader()
            writer.writerows(self.rows)

        summary = summarise(self.rows)
        self.path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2))
        return summary
